#!/usr/bin/env python3
"""
track_navigator.py — PiCar-X Oval Track Navigator v3

Architecture:
  - Runs on Fedora machine
  - Polls Pi agent at /api/sensors
  - Sends drive commands via /api/drive
  - Runs for SESSION_DURATION seconds then stops
  - Writes a detailed JSONL log for post-session analysis

Track: masking tape on garage floor, oval shape, ~15" lane
Direction: clockwise

Grayscale sensor orientation (confirmed by calibration):
  - gs[0] = RIGHT sensor
  - gs[1] = CENTER sensor
  - gs[2] = LEFT sensor

Thresholds (masking tape on garage floor):
  - Garage floor reads LOW (~316-1145)
  - Masking tape reads HIGH (~868-1463)
  - Values ABOVE threshold = tape detected

Corner detection:
  - Primary: grayscale both sensors on tape simultaneously
  - Secondary: LiDAR front distance below CORNER_TRIGGER_MM
  - Car slows when front distance below CORNER_SLOW_MM

Off-track detection:
  - All sensors below OFF_TRACK_THRESHOLD = car off expected surface
  - Car stops and waits for manual replacement
"""

import requests
import time
import json
import sys
import os
from datetime import datetime

# ── Load config ────────────────────────────────────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from config import PI_IP, AGENT_PORT
    print("Config loaded from config.py")
except ImportError:
    print("config.py not found — using defaults.")
    PI_IP      = "YOUR_PI_IP"
    AGENT_PORT = 8000

AGENT_URL = f"http://{PI_IP}:{AGENT_PORT}"

# ── Session config ─────────────────────────────────────────────────────────────
SESSION_DURATION    = 180     # seconds (3 minutes)
POLL_RATE_HZ        = 10      # sensor polls per second
DRIVE_SPEED         = 20      # cruising speed
CORNER_SPEED        = 15      # speed during corner turns
CORRECTION_ANGLE    = 15      # steering angle for boundary correction
CORNER_ANGLE        = 30      # steering angle for corner turns
CORNER_DURATION     = 0.8     # seconds to hold corner turn
CORRECTION_COOLDOWN = 0.2     # minimum seconds between corrections

# ── LiDAR corner detection ─────────────────────────────────────────────────────
CORNER_SLOW_MM      = 1200    # front distance to start slowing
CORNER_TRIGGER_MM   = 700     # front distance to execute turn

# ── Grayscale thresholds (masking tape on garage floor) ───────────────────────
# Tape reads HIGH (~868-1463), floor reads LOW (~316-1145)
# Values ABOVE threshold = tape detected
GS_THRESHOLD_LEFT   = 700
GS_THRESHOLD_CENTER = 850     # weak signal — confirmation only
GS_THRESHOLD_RIGHT  = 1000

# Zero/dropout detection
GS_ZERO_IGNORE      = 50

# Off-track: all sensors below this = car off expected surface
# Set below minimum tape reading but above typical floor reading
OFF_TRACK_THRESHOLD = 600

# ── Log file ───────────────────────────────────────────────────────────────────
LOG_DIR  = os.path.expanduser("~/picar-logs")
LOG_FILE = os.path.join(
    LOG_DIR,
    f"track_session_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
)


# ── API helpers ────────────────────────────────────────────────────────────────
def api(endpoint, method="get", params=None):
    url = f"{AGENT_URL}{endpoint}"
    try:
        if method == "get":
            r = requests.get(url, params=params, timeout=1.0)
        else:
            r = requests.post(url, params=params, timeout=1.0)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        print(f"  [API ERROR] {endpoint}: {e}")
        return None

def drive(speed, angle):
    api("/api/drive", method="post", params={"speed": speed, "angle": angle})

def stop():
    api("/api/stop", method="post")

def set_mode(mode):
    api(f"/api/mode/{mode}", method="post")


# ── Grayscale helpers ──────────────────────────────────────────────────────────
def read_grayscale(gs):
    """
    Parse grayscale readings, ignoring dropouts.
    Returns (right_val, center_val, left_val) with None for dropped readings.
    gs = [right, center, left] per PiCar-X library ordering.
    """
    def valid(v):
        return v if v is not None and v > GS_ZERO_IGNORE else None

    right  = valid(gs[0]) if len(gs) > 0 else None
    center = valid(gs[1]) if len(gs) > 1 else None
    left   = valid(gs[2]) if len(gs) > 2 else None
    return right, center, left


def on_tape(val, threshold):
    """True if valid reading and ABOVE tape threshold."""
    return val is not None and val > threshold


def detect_boundaries(gs):
    """
    Analyze grayscale readings and return navigation decision.

    Returns one of:
      'OFF_TRACK'    — all sensors below OFF_TRACK_THRESHOLD
      'CORNER'       — both left and right on tape simultaneously
      'RIGHT_OUTER'  — right sensor on outer boundary tape
      'LEFT_OUTER'   — left sensor on outer boundary tape
      'CENTER_TAPE'  — center sensor on tape only
      'CLEAR'        — no tape detected
    """
    right, center, left = read_grayscale(gs)

    # Off-track: all valid sensors reading low
    valid_readings = [v for v in [right, center, left] if v is not None]
    if len(valid_readings) >= 2 and all(v < OFF_TRACK_THRESHOLD for v in valid_readings):
        return 'OFF_TRACK'

    right_tape  = on_tape(right,  GS_THRESHOLD_RIGHT)
    center_tape = on_tape(center, GS_THRESHOLD_CENTER)
    left_tape   = on_tape(left,   GS_THRESHOLD_LEFT)

    # Both outer sensors on tape = corner
    if right_tape and left_tape:
        return 'CORNER'

    if right_tape:
        return 'RIGHT_OUTER'

    if left_tape:
        return 'LEFT_OUTER'

    if center_tape:
        return 'CENTER_TAPE'

    return 'CLEAR'


# ── Logging ────────────────────────────────────────────────────────────────────
def build_log_entry(sensors, decision, speed, angle, event=None):
    gs    = sensors.get("grayscale", [0, 0, 0])
    lidar = sensors.get("lidar", {})
    return {
        "ts":            datetime.now().isoformat(timespec="milliseconds"),
        "decision":      decision,
        "speed":         speed,
        "angle":         angle,
        "event":         event,
        "grayscale": {
            "right":     gs[0] if len(gs) > 0 else 0,
            "center":    gs[1] if len(gs) > 1 else 0,
            "left":      gs[2] if len(gs) > 2 else 0,
        },
        "lidar": {
            "points":    lidar.get("points", 0),
            "front_mm":  lidar.get("front", 0),
            "left_mm":   lidar.get("left", 0),
            "right_mm":  lidar.get("right", 0),
            "back_mm":   lidar.get("back", 0),
        },
        "ultrasonic_cm":  sensors.get("ultrasonic_cm", 0),
        "cliff_detected": sensors.get("cliff_detected", False),
        "obstacle_close": sensors.get("obstacle_close", False),
        "reflex_active":  sensors.get("reflex_active", False),
    }

def write_log(f, entry):
    f.write(json.dumps(entry) + "\n")
    f.flush()

def write_summary(entries, duration):
    decisions = {}
    for e in entries:
        d = e.get("decision", "UNKNOWN")
        decisions[d] = decisions.get(d, 0) + 1

    corners      = sum(1 for e in entries if e.get("event") == "CORNER")
    left_events  = sum(1 for e in entries if e.get("event") == "LEFT_OUTER")
    right_events = sum(1 for e in entries if e.get("event") == "RIGHT_OUTER")
    off_track    = sum(1 for e in entries if e.get("event") == "OFF_TRACK")
    lidar_corners = sum(1 for e in entries if e.get("event") == "LIDAR_CORNER")
    errors       = sum(1 for e in entries if e.get("event") == "SENSOR_ERROR")

    return {
        "type":                   "session_summary",
        "session_duration":       duration,
        "total_entries":          len(entries),
        "decision_counts":        decisions,
        "corner_turns_grayscale": corners,
        "corner_turns_lidar":     lidar_corners,
        "left_boundary_events":   left_events,
        "right_boundary_events":  right_events,
        "off_track_events":       off_track,
        "sensor_errors":          errors,
        "estimated_laps":         round((corners + lidar_corners) / 4, 1),
    }


# ── Main navigator loop ────────────────────────────────────────────────────────
def main():
    print("=" * 60)
    print("  PiCar-X Oval Track Navigator v3")
    print(f"  Agent:    {AGENT_URL}")
    print(f"  Duration: {SESSION_DURATION}s")
    print(f"  Speed:    {DRIVE_SPEED}  Corner speed: {CORNER_SPEED}")
    print(f"  GS thresholds: L>{GS_THRESHOLD_LEFT} C>{GS_THRESHOLD_CENTER} R>{GS_THRESHOLD_RIGHT}")
    print(f"  LiDAR corner trigger: <{CORNER_TRIGGER_MM}mm  slow: <{CORNER_SLOW_MM}mm")
    print(f"  Log:      {LOG_FILE}")
    print("=" * 60)

    # Verify agent
    print("\nConnecting to Pi agent...")
    status = api("/api/status")
    if status is None:
        print("ERROR: Cannot reach Pi agent.")
        sys.exit(1)
    print(f"  Connected. Battery: {status.get('battery_v','?')}V "
          f"({status.get('battery_pct','?')}%)")

    # Check estop
    estop = api("/api/estop")
    if estop and estop.get("estop_active"):
        print("ERROR: ESTOP is active. Clear it before running.")
        sys.exit(1)

    os.makedirs(LOG_DIR, exist_ok=True)

    print("\nStarting in 3 seconds — place car on track facing clockwise...")
    print("Press Ctrl+C at any time to stop safely.\n")
    time.sleep(3)

    set_mode("autonomous")
    print("Autonomous mode set.\n")

    log_entries     = []
    last_correction = 0
    session_start   = time.time()
    loop_interval   = 1.0 / POLL_RATE_HZ
    current_speed   = DRIVE_SPEED
    current_angle   = 0
    in_corner       = False

    try:
        with open(LOG_FILE, "w") as log_f:

            # Session header
            write_log(log_f, {
                "type":       "session_header",
                "start_time": datetime.now().isoformat(),
                "config": {
                    "duration":            SESSION_DURATION,
                    "drive_speed":         DRIVE_SPEED,
                    "corner_speed":        CORNER_SPEED,
                    "correction_angle":    CORRECTION_ANGLE,
                    "corner_angle":        CORNER_ANGLE,
                    "corner_duration":     CORNER_DURATION,
                    "gs_threshold_left":   GS_THRESHOLD_LEFT,
                    "gs_threshold_center": GS_THRESHOLD_CENTER,
                    "gs_threshold_right":  GS_THRESHOLD_RIGHT,
                    "off_track_threshold": OFF_TRACK_THRESHOLD,
                    "corner_slow_mm":      CORNER_SLOW_MM,
                    "corner_trigger_mm":   CORNER_TRIGGER_MM,
                }
            })

            # ── Main loop ──────────────────────────────────────────────────────
            while True:
                loop_start = time.time()
                elapsed    = loop_start - session_start

                if elapsed >= SESSION_DURATION:
                    print(f"\nSession complete ({SESSION_DURATION}s).")
                    break

                remaining = int(SESSION_DURATION - elapsed)
                print(f"  [{remaining:3d}s] ", end="")

                # ── Read sensors ───────────────────────────────────────────────
                sensors = api("/api/sensors")
                if sensors is None:
                    print("sensor error — holding course")
                    entry = build_log_entry(
                        {}, "HOLD", current_speed, current_angle, "SENSOR_ERROR"
                    )
                    log_entries.append(entry)
                    write_log(log_f, entry)
                    time.sleep(loop_interval)
                    continue

                gs     = sensors.get("grayscale", [0, 0, 0])
                lidar  = sensors.get("lidar", {})
                front  = lidar.get("front", 0) or 0
                reflex = sensors.get("reflex_active", False)

                # ── Reflex override ────────────────────────────────────────────
                if reflex:
                    print(f"reflex active gs={gs} us={sensors.get('ultrasonic_cm',0):.1f}")
                    entry = build_log_entry(
                        sensors, "REFLEX", current_speed, current_angle, "REFLEX"
                    )
                    log_entries.append(entry)
                    write_log(log_f, entry)
                    time.sleep(loop_interval)
                    continue

                # ── Boundary detection ─────────────────────────────────────────
                boundary         = detect_boundaries(gs)
                now              = time.time()
                correction_ready = (now - last_correction) > CORRECTION_COOLDOWN

                # ── OFF TRACK ──────────────────────────────────────────────────
                if boundary == 'OFF_TRACK':
                    print(f"OFF TRACK — stopping  gs={gs}")
                    stop()
                    current_speed = 0
                    current_angle = 0

                    entry = build_log_entry(sensors, "STOP", 0, 0, "OFF_TRACK")
                    log_entries.append(entry)
                    write_log(log_f, entry)

                    print("  Waiting to be placed back on track...")
                    while True:
                        time.sleep(0.5)
                        sensors2 = api("/api/sensors")
                        if sensors2 is None:
                            continue
                        gs2 = sensors2.get("grayscale", [0, 0, 0])
                        r2, c2, l2 = read_grayscale(gs2)
                        valid2 = [v for v in [r2, c2, l2] if v is not None]
                        if valid2 and any(v >= OFF_TRACK_THRESHOLD for v in valid2):
                            print("  Back on surface — resuming in 2 seconds...")
                            time.sleep(2)
                            set_mode("autonomous")
                            drive(DRIVE_SPEED, 0)
                            current_speed = DRIVE_SPEED
                            current_angle = 0
                            in_corner     = False
                            break

                    time.sleep(loop_interval)
                    continue

                # ── GRAYSCALE CORNER ───────────────────────────────────────────
                elif boundary == 'CORNER' and correction_ready and not in_corner:
                    print(f"GS CORNER — turning right  gs={gs}")
                    in_corner = True
                    drive(CORNER_SPEED, CORNER_ANGLE)
                    current_speed = CORNER_SPEED
                    current_angle = CORNER_ANGLE
                    last_correction = now

                    entry = build_log_entry(
                        sensors, "CORNER_TURN", CORNER_SPEED, CORNER_ANGLE, "CORNER"
                    )
                    log_entries.append(entry)
                    write_log(log_f, entry)

                    time.sleep(CORNER_DURATION)
                    drive(DRIVE_SPEED, 0)
                    current_speed = DRIVE_SPEED
                    current_angle = 0
                    in_corner     = False

                # ── LIDAR CORNER ───────────────────────────────────────────────
                elif not in_corner and front > 0 and front < CORNER_TRIGGER_MM and correction_ready:
                    print(f"LIDAR CORNER (front={front:.0f}mm) — turning right")
                    in_corner = True
                    drive(CORNER_SPEED, CORNER_ANGLE)
                    current_speed = CORNER_SPEED
                    current_angle = CORNER_ANGLE
                    last_correction = now

                    entry = build_log_entry(
                        sensors, "CORNER_TURN", CORNER_SPEED, CORNER_ANGLE, "LIDAR_CORNER"
                    )
                    log_entries.append(entry)
                    write_log(log_f, entry)

                    time.sleep(CORNER_DURATION)
                    drive(DRIVE_SPEED, 0)
                    current_speed = DRIVE_SPEED
                    current_angle = 0
                    in_corner     = False

                # ── LIDAR SLOW ─────────────────────────────────────────────────
                elif not in_corner and front > 0 and front < CORNER_SLOW_MM:
                    slow_speed = max(CORNER_SPEED,
                                     int(DRIVE_SPEED * (front / CORNER_SLOW_MM)))
                    drive(slow_speed, current_angle)
                    current_speed = slow_speed
                    print(f"slowing for corner (front={front:.0f}mm speed={slow_speed})")

                    entry = build_log_entry(
                        sensors, "CORNER_SLOW", slow_speed, current_angle, "CORNER_SLOW"
                    )
                    log_entries.append(entry)
                    write_log(log_f, entry)

                # ── RIGHT BOUNDARY ─────────────────────────────────────────────
                elif boundary == 'RIGHT_OUTER' and correction_ready:
                    print(f"RIGHT boundary — correcting left  gs={gs}")
                    drive(DRIVE_SPEED, -CORRECTION_ANGLE)
                    current_angle   = -CORRECTION_ANGLE
                    last_correction = now

                    entry = build_log_entry(
                        sensors, "CORRECT_LEFT", DRIVE_SPEED, -CORRECTION_ANGLE, "RIGHT_OUTER"
                    )
                    log_entries.append(entry)
                    write_log(log_f, entry)

                # ── LEFT BOUNDARY ──────────────────────────────────────────────
                elif boundary == 'LEFT_OUTER' and correction_ready:
                    print(f"LEFT boundary  — correcting right gs={gs}")
                    drive(DRIVE_SPEED, CORRECTION_ANGLE)
                    current_angle   = CORRECTION_ANGLE
                    last_correction = now

                    entry = build_log_entry(
                        sensors, "CORRECT_RIGHT", DRIVE_SPEED, CORRECTION_ANGLE, "LEFT_OUTER"
                    )
                    log_entries.append(entry)
                    write_log(log_f, entry)

                # ── CENTER TAPE ────────────────────────────────────────────────
                elif boundary == 'CENTER_TAPE' and correction_ready:
                    print(f"CENTER tape — steering right  gs={gs}")
                    drive(DRIVE_SPEED, CORRECTION_ANGLE)
                    current_angle   = CORRECTION_ANGLE
                    last_correction = now

                    entry = build_log_entry(
                        sensors, "CORRECT_RIGHT", DRIVE_SPEED, CORRECTION_ANGLE, "CENTER_TAPE"
                    )
                    log_entries.append(entry)
                    write_log(log_f, entry)

                # ── FORWARD ────────────────────────────────────────────────────
                else:
                    if current_angle != 0:
                        drive(DRIVE_SPEED, 0)
                        current_angle = 0
                    current_speed = DRIVE_SPEED
                    print(f"forward  gs={gs}  front={front:.0f}mm")

                    entry = build_log_entry(sensors, "FORWARD", DRIVE_SPEED, 0)
                    log_entries.append(entry)
                    write_log(log_f, entry)

                # ── Loop timing ────────────────────────────────────────────────
                elapsed_loop = time.time() - loop_start
                time.sleep(max(0, loop_interval - elapsed_loop))

            # ── Session end ────────────────────────────────────────────────────
            stop()
            set_mode("manual")

            summary = write_summary(log_entries, SESSION_DURATION)
            write_log(log_f, summary)

            print("\n" + "=" * 60)
            print("  SESSION SUMMARY")
            print("=" * 60)
            print(f"  Duration:           {SESSION_DURATION}s")
            print(f"  Total entries:      {summary['total_entries']}")
            print(f"  GS corner turns:    {summary['corner_turns_grayscale']}")
            print(f"  LiDAR corner turns: {summary['corner_turns_lidar']}")
            print(f"  Estimated laps:     {summary['estimated_laps']}")
            print(f"  Left boundary:      {summary['left_boundary_events']}")
            print(f"  Right boundary:     {summary['right_boundary_events']}")
            print(f"  Off track:          {summary['off_track_events']}")
            print(f"  Sensor errors:      {summary['sensor_errors']}")
            print(f"  Log saved:          {LOG_FILE}")
            print("=" * 60)

    except KeyboardInterrupt:
        print("\n\nInterrupted — stopping safely...")
        stop()
        set_mode("manual")
        print("Car stopped. Log saved to:", LOG_FILE)


if __name__ == "__main__":
    if PI_IP == "YOUR_PI_IP":
        print("ERROR: config.py not found at repo root.")
        sys.exit(1)
    main()

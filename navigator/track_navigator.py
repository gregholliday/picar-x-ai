#!/usr/bin/env python3
"""
track_navigator.py — PiCar-X Clockwise Oval Track Navigator

Architecture:
  - Runs on Fedora machine
  - Polls Pi agent at /api/sensors
  - Sends drive commands via /api/drive and /api/turn
  - Runs for SESSION_DURATION seconds then stops
  - Writes a detailed log file for post-session analysis

Track layout (clockwise):
  - Start on left straight, traveling upward
  - Right turn at each corner (4 total)
  - Grayscale handles boundary detection on straights
  - LiDAR front distance triggers corner turns

Grayscale sensor orientation (as confirmed by calibration):
  - gs[0] = RIGHT sensor
  - gs[1] = CENTER sensor
  - gs[2] = LEFT sensor

Thresholds (black duct tape on concrete, values BELOW = tape detected):
  - LEFT:   < 320  (floor ~373, tape ~199)
  - CENTER: < 237  (floor ~233, tape ~237 — weak signal, used as confirmation only)
  - RIGHT:  < 500  (floor ~635, tape ~367)
"""

import requests
import time
import json
import sys
import os
from datetime import datetime

# ── Load config (same pattern as picar_agent_v7.py) ───────────────────────────
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from config import PI_IP, AGENT_PORT
    print("Config loaded from config.py")
except ImportError:
    print("config.py not found — using defaults. Copy config.py to repo root and edit it.")
    PI_IP      = "YOUR_PI_IP"
    AGENT_PORT = 8000

AGENT_URL = f"http://{PI_IP}:{AGENT_PORT}"

SESSION_DURATION  = 180       # seconds (3 minutes)
POLL_RATE_HZ      = 10        # sensor polls per second
DRIVE_SPEED       = 35        # normal cruising speed
TURN_SPEED        = 25        # speed during corner turns
CORRECTION_ANGLE  = 15        # steering angle for boundary correction
TURN_ANGLE        = 30        # steering angle for corner turns
TURN_DURATION     = 0.6       # seconds to hold corner turn

# LiDAR corner detection
CORNER_TRIGGER_MM = 400       # front LiDAR distance that triggers a corner turn
CORNER_SLOW_MM    = 600       # front LiDAR distance to start slowing

# Grayscale thresholds — values BELOW these indicate tape (black duct tape)
GS_THRESHOLD_LEFT   = 320
GS_THRESHOLD_CENTER = 220     # weak signal — used for confirmation only
GS_THRESHOLD_RIGHT  = 500

# Minimum gap between boundary corrections (seconds)
CORRECTION_COOLDOWN = 0.3

# Log file
LOG_DIR  = os.path.expanduser("~/picar-logs")
LOG_FILE = os.path.join(
    LOG_DIR,
    f"track_session_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
)


# ── Helpers ────────────────────────────────────────────────────────────────────
def api(endpoint, method="get", params=None):
    """Make a request to the Pi agent."""
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
    """Send a drive command."""
    api("/api/drive", method="post", params={"speed": speed, "angle": angle})


def stop():
    """Stop the car."""
    api("/api/stop", method="post")


def set_mode(mode):
    """Set autonomous or manual mode."""
    api(f"/api/mode/{mode}", method="post")


# ── Grayscale detection ────────────────────────────────────────────────────────
def detect_boundaries(gs):
    """
    Returns (left_boundary, right_boundary) booleans.
    gs = [right_val, center_val, left_val] per PiCar-X library ordering.
    Tape is detected when value DROPS BELOW threshold (black tape = less reflective).
    """
    right_val  = gs[0]
    center_val = gs[1]
    left_val   = gs[2]

    # Ignore zero readings — sensor dropout, not tape
    if right_val == 0 or left_val == 0:
        return False, False

    right_boundary = right_val < GS_THRESHOLD_RIGHT
    left_boundary  = (left_val < GS_THRESHOLD_LEFT) or \
                     (center_val < GS_THRESHOLD_CENTER and left_val < 350)

    return left_boundary, right_boundary


# ── Logging ────────────────────────────────────────────────────────────────────
def build_log_entry(sensors, decision, speed, angle, event=None):
    """Build a structured log entry."""
    gs = sensors.get("grayscale", [0, 0, 0])
    lidar = sensors.get("lidar", {})

    entry = {
        "ts":              datetime.now().isoformat(timespec="milliseconds"),
        "decision":        decision,
        "speed":           speed,
        "angle":           angle,
        "event":           event,
        "grayscale": {
            "right":       gs[0] if len(gs) > 0 else 0,
            "center":      gs[1] if len(gs) > 1 else 0,
            "left":        gs[2] if len(gs) > 2 else 0,
        },
        "lidar": {
            "points":      lidar.get("points", 0),
            "front_mm":    lidar.get("front", 0),
            "left_mm":     lidar.get("left", 0),
            "right_mm":    lidar.get("right", 0),
            "back_mm":     lidar.get("back", 0),
        },
        "ultrasonic_cm":   sensors.get("ultrasonic_cm", 0),
        "cliff_detected":  sensors.get("cliff_detected", False),
        "obstacle_close":  sensors.get("obstacle_close", False),
        "reflex_active":   sensors.get("reflex_active", False),
    }
    return entry


def write_log(f, entry):
    """Write a log entry as JSONL."""
    f.write(json.dumps(entry) + "\n")
    f.flush()


# ── Session summary ────────────────────────────────────────────────────────────
def write_summary(log_entries, session_duration):
    """Write a human-readable session summary at the end of the log."""
    total         = len(log_entries)
    corners       = sum(1 for e in log_entries if e.get("event") == "CORNER_TURN")
    left_events   = sum(1 for e in log_entries if e.get("event") == "LEFT_BOUNDARY")
    right_events  = sum(1 for e in log_entries if e.get("event") == "RIGHT_BOUNDARY")
    reflex_events = sum(1 for e in log_entries if e.get("reflex_active"))
    errors        = sum(1 for e in log_entries if e.get("event") == "SENSOR_ERROR")

    summary = {
        "type":             "session_summary",
        "session_duration": session_duration,
        "total_entries":    total,
        "corner_turns":     corners,
        "left_boundary_events":  left_events,
        "right_boundary_events": right_events,
        "reflex_activations":    reflex_events,
        "sensor_errors":         errors,
        "estimated_laps":        round(corners / 4, 1) if corners >= 4 else 0,
    }
    return summary


# ── Main navigator loop ────────────────────────────────────────────────────────
def main():
    print("=" * 60)
    print("  PiCar-X Track Navigator")
    print(f"  Agent:    {AGENT_URL}")
    print(f"  Duration: {SESSION_DURATION}s ({SESSION_DURATION//60}m {SESSION_DURATION%60}s)")
    print(f"  Speed:    {DRIVE_SPEED}")
    print(f"  Log:      {LOG_FILE}")
    print("=" * 60)

    # Verify agent is reachable
    print("\nConnecting to Pi agent...")
    status = api("/api/status")
    if status is None:
        print("ERROR: Cannot reach Pi agent. Check PI_IP and that agent is running.")
        sys.exit(1)
    print(f"  Connected. Battery: {status.get('battery_v', '?')}V "
          f"({status.get('battery_pct', '?')}%)")

    # Check estop
    estop = api("/api/estop")
    if estop and estop.get("estop_active"):
        print("ERROR: ESTOP is active. Clear it before running.")
        sys.exit(1)

    # Setup log directory
    os.makedirs(LOG_DIR, exist_ok=True)

    print("\nPreparing to start in 3 seconds — place car on track now...")
    print("Press Ctrl+C at any time to stop safely.\n")
    time.sleep(3)

    # Set autonomous mode
    set_mode("autonomous")
    print("Autonomous mode set.\n")

    log_entries   = []
    last_correction = 0
    session_start   = time.time()
    loop_interval   = 1.0 / POLL_RATE_HZ
    current_speed   = DRIVE_SPEED
    current_angle   = 0
    in_corner       = False

    try:
        with open(LOG_FILE, "w") as log_f:

            # Write session header
            header = {
                "type":        "session_header",
                "start_time":  datetime.now().isoformat(),
                "config": {
                    "duration":       SESSION_DURATION,
                    "drive_speed":    DRIVE_SPEED,
                    "turn_speed":     TURN_SPEED,
                    "correction_angle": CORRECTION_ANGLE,
                    "turn_angle":     TURN_ANGLE,
                    "gs_threshold_left":   GS_THRESHOLD_LEFT,
                    "gs_threshold_center": GS_THRESHOLD_CENTER,
                    "gs_threshold_right":  GS_THRESHOLD_RIGHT,
                    "corner_trigger_mm":   CORNER_TRIGGER_MM,
                }
            }
            write_log(log_f, header)

            # ── Main loop ──────────────────────────────────────────────────────
            while True:
                loop_start = time.time()
                elapsed    = loop_start - session_start

                # Session time check
                if elapsed >= SESSION_DURATION:
                    print(f"\nSession complete ({SESSION_DURATION}s).")
                    break

                # Remaining time display
                remaining = int(SESSION_DURATION - elapsed)
                print(f"  [{remaining:3d}s left] ", end="")

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
                front  = lidar.get("front", 9999)
                reflex = sensors.get("reflex_active", False)

                # ── Corner detection ───────────────────────────────────────────
                if not in_corner and front > 0 and front < CORNER_TRIGGER_MM:
                    print(f"CORNER (front={front}mm) — turning right")
                    in_corner = True

                    # Slow and turn right
                    drive(TURN_SPEED, TURN_ANGLE)
                    current_speed = TURN_SPEED
                    current_angle = TURN_ANGLE

                    entry = build_log_entry(
                        sensors, "CORNER_TURN", TURN_SPEED, TURN_ANGLE, "CORNER_TURN"
                    )
                    log_entries.append(entry)
                    write_log(log_f, entry)

                    time.sleep(TURN_DURATION)

                    # Resume straight
                    drive(DRIVE_SPEED, 0)
                    current_speed = DRIVE_SPEED
                    current_angle = 0
                    in_corner     = False

                    time.sleep(loop_interval)
                    continue

                # Approaching corner — slow down
                elif not in_corner and front > 0 and front < CORNER_SLOW_MM:
                    slow_speed = max(TURN_SPEED, int(DRIVE_SPEED * (front / CORNER_SLOW_MM)))
                    drive(slow_speed, 0)
                    current_speed = slow_speed
                    current_angle = 0
                    print(f"slowing for corner (front={front}mm, speed={slow_speed})")

                    entry = build_log_entry(
                        sensors, "CORNER_SLOW", slow_speed, 0, "CORNER_SLOW"
                    )
                    log_entries.append(entry)
                    write_log(log_f, entry)
                    time.sleep(loop_interval)
                    continue

                # ── Boundary detection ─────────────────────────────────────────
                now = time.time()
                left_boundary, right_boundary = detect_boundaries(gs)
                correction_ready = (now - last_correction) > CORRECTION_COOLDOWN

                if reflex:
                    # Reflex loop is handling something — let it
                    print(f"reflex active — gs={gs}")
                    entry = build_log_entry(
                        sensors, "REFLEX", current_speed, current_angle, "REFLEX"
                    )
                    log_entries.append(entry)
                    write_log(log_f, entry)

                elif right_boundary and correction_ready:
                    # Drifting right — steer left to correct
                    corrected_angle = -CORRECTION_ANGLE
                    drive(DRIVE_SPEED, corrected_angle)
                    current_angle   = corrected_angle
                    last_correction = now
                    print(f"RIGHT boundary — correcting left  gs={gs}")

                    entry = build_log_entry(
                        sensors, "CORRECT_LEFT", DRIVE_SPEED, corrected_angle, "RIGHT_BOUNDARY"
                    )
                    log_entries.append(entry)
                    write_log(log_f, entry)

                elif left_boundary and correction_ready:
                    # Drifting left — steer right to correct
                    corrected_angle = CORRECTION_ANGLE
                    drive(DRIVE_SPEED, corrected_angle)
                    current_angle   = corrected_angle
                    last_correction = now
                    print(f"LEFT boundary  — correcting right gs={gs}")

                    entry = build_log_entry(
                        sensors, "CORRECT_RIGHT", DRIVE_SPEED, corrected_angle, "LEFT_BOUNDARY"
                    )
                    log_entries.append(entry)
                    write_log(log_f, entry)

                else:
                    # All clear — drive straight
                    if current_angle != 0:
                        drive(DRIVE_SPEED, 0)
                        current_angle = 0
                    else:
                        drive(DRIVE_SPEED, 0)
                    current_speed = DRIVE_SPEED
                    print(f"forward  gs={gs}  front={front}mm")

                    entry = build_log_entry(
                        sensors, "FORWARD", DRIVE_SPEED, 0
                    )
                    log_entries.append(entry)
                    write_log(log_f, entry)

                # ── Loop timing ────────────────────────────────────────────────
                elapsed_loop = time.time() - loop_start
                sleep_time   = max(0, loop_interval - elapsed_loop)
                time.sleep(sleep_time)

            # ── Session end ────────────────────────────────────────────────────
            stop()
            set_mode("manual")

            summary = write_summary(log_entries, SESSION_DURATION)
            write_log(log_f, summary)

            print("\n" + "=" * 60)
            print("  SESSION SUMMARY")
            print("=" * 60)
            print(f"  Duration:         {SESSION_DURATION}s")
            print(f"  Total entries:    {summary['total_entries']}")
            print(f"  Corner turns:     {summary['corner_turns']}")
            print(f"  Estimated laps:   {summary['estimated_laps']}")
            print(f"  Left boundary:    {summary['left_boundary_events']}")
            print(f"  Right boundary:   {summary['right_boundary_events']}")
            print(f"  Reflex events:    {summary['reflex_activations']}")
            print(f"  Sensor errors:    {summary['sensor_errors']}")
            print(f"  Log saved:        {LOG_FILE}")
            print("=" * 60)

    except KeyboardInterrupt:
        print("\n\nInterrupted — stopping car safely...")
        stop()
        set_mode("manual")
        print("Car stopped. Log saved to:", LOG_FILE)


if __name__ == "__main__":
    if PI_IP == "YOUR_PI_IP":
        print("ERROR: PI_IP not set. Make sure config.py exists at the repo root.")
        print("  Expected: /mnt/ai-lab/picar-x-ai/config.py")
        sys.exit(1)
    main()

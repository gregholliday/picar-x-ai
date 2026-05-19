#!/usr/bin/env python3
"""
roam_navigator.py — PiCar-X Free Roam Navigator

LiDAR-based free roaming within defined environment boundaries.
Grayscale tape barriers act as emergency stops only.
Logs all decisions in KB-ingestible format.

Usage:
  python3 roam_navigator.py
  python3 roam_navigator.py --duration 120
  python3 roam_navigator.py --env garage --speed 25

Architecture:
  - Runs on Fedora, polls Pi agent
  - LiDAR handles wall avoidance on 3 sides
  - Grayscale handles tape emergency stop on open side
  - Ultrasonic handles close obstacle avoidance
  - All decisions logged for KB ingestion
"""

import requests
import time
import json
import sys
import os
import argparse
import random
from datetime import datetime
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from config import PI_IP, AGENT_PORT
except ImportError:
    PI_IP      = "YOUR_PI_IP"
    AGENT_PORT = 8000

AGENT_URL = f"http://{PI_IP}:{AGENT_PORT}"

# ── Environment ────────────────────────────────────────────────────────────────
ENVIRONMENT  = os.environ.get("PICAR_ENV", "garage")
LOG_DIR      = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")

# ── Navigation config ──────────────────────────────────────────────────────────
DRIVE_SPEED         = 20    # normal cruising speed
TURN_SPEED          = 15    # speed while turning
SESSION_DURATION    = 180   # seconds
POLL_HZ             = 10    # sensor polls per second

# LiDAR wall avoidance thresholds (mm)
WALL_STOP_MM        = 400   # stop if wall this close
WALL_SLOW_MM        = 800   # slow down if wall this close
WALL_STEER_MM       = 600   # steer away if side wall this close

# Grayscale tape emergency stop
GS_TAPE_THRESHOLD_LEFT   = 950
GS_TAPE_THRESHOLD_CENTER = 850
GS_TAPE_THRESHOLD_RIGHT  = 1000
GS_ZERO_IGNORE           = 50

# Turn behavior
TURN_DURATION_MIN   = 0.5   # minimum turn time (seconds)
TURN_DURATION_MAX   = 1.5   # maximum turn time (seconds)
CORRECTION_ANGLE    = 15    # gentle wall avoidance angle
TURN_ANGLE          = 30    # full turn angle


def api(endpoint, method="get", params=None):
    try:
        url = f"{AGENT_URL}{endpoint}"
        r = requests.get(url, timeout=1.0) if method == "get" else requests.post(url, params=params, timeout=1.0)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        return None


def drive(speed, angle):
    api("/api/drive", method="post", params={"speed": speed, "angle": angle})


def stop():
    api("/api/stop", method="post")


def set_mode(mode):
    api(f"/api/mode/{mode}", method="post")


# ── Sensor parsing ─────────────────────────────────────────────────────────────
def parse_sensors(sensors):
    lidar = sensors.get("lidar", {})
    gs    = sensors.get("grayscale", [0, 0, 0])
    return {
        "front_mm":     lidar.get("front", 0) or 0,
        "back_mm":      lidar.get("back",  0) or 0,
        "left_mm":      lidar.get("left",  0) or 0,
        "right_mm":     lidar.get("right", 0) or 0,
        "lidar_points": lidar.get("points", 0),
        "us_cm":        sensors.get("ultrasonic_cm", 0) or 0,
        "gs_left":      gs[0] if len(gs) > 0 else 0,
        "gs_center":    gs[1] if len(gs) > 1 else 0,
        "gs_right":     gs[2] if len(gs) > 2 else 0,
        "reflex":       sensors.get("reflex_active", False),
        "obstacle":     sensors.get("obstacle_close", False),
    }


def check_tape_emergency(s):
    """Check if grayscale sensors detect tape emergency stop."""
    def valid(v):
        return v if v and v > GS_ZERO_IGNORE else None

    left   = valid(s["gs_left"])
    center = valid(s["gs_center"])
    right  = valid(s["gs_right"])

    left_tape   = left   is not None and left   > GS_TAPE_THRESHOLD_LEFT
    center_tape = center is not None and center > GS_TAPE_THRESHOLD_CENTER
    right_tape  = right  is not None and right  > GS_TAPE_THRESHOLD_RIGHT

    # Tape detected if any outer sensor confirmed by center
    if (left_tape and center_tape) or (right_tape and center_tape):
        return True
    # Or single sensor well above threshold
    if left_tape  and left  > GS_TAPE_THRESHOLD_LEFT  * 1.2:
        return True
    if right_tape and right > GS_TAPE_THRESHOLD_RIGHT * 1.2:
        return True
    return False


def decide(s, last_turn_time):
    """
    Make a navigation decision based on sensor readings.
    Returns (action, speed, angle, reason)
    """
    now = time.time()

    # Tape emergency stop
    if check_tape_emergency(s):
        return ("TAPE_STOP", 0, 0, "Tape boundary detected — emergency stop")

    # Reflex active — let Pi handle it
    if s["reflex"]:
        return ("REFLEX", s.get("speed", 0), s.get("angle", 0), "Pi reflex active")

    # Front wall too close — turn away
    front = s["front_mm"]
    if 0 < front < WALL_STOP_MM:
        direction = "left" if s["left_mm"] > s["right_mm"] else "right"
        angle     = -TURN_ANGLE if direction == "left" else TURN_ANGLE
        return ("WALL_TURN", TURN_SPEED, angle, f"Front wall {front:.0f}mm — turning {direction}")

    # Front wall approaching — slow and steer
    if 0 < front < WALL_SLOW_MM:
        slow    = max(TURN_SPEED, int(DRIVE_SPEED * (front / WALL_SLOW_MM)))
        # Bias toward more open side
        if s["left_mm"] > s["right_mm"] + 300:
            return ("WALL_SLOW_LEFT", slow, -CORRECTION_ANGLE, f"Front {front:.0f}mm — slowing, biasing left")
        elif s["right_mm"] > s["left_mm"] + 300:
            return ("WALL_SLOW_RIGHT", slow, CORRECTION_ANGLE, f"Front {front:.0f}mm — slowing, biasing right")
        return ("WALL_SLOW", slow, 0, f"Front wall {front:.0f}mm — slowing")

    # Left wall too close — steer right
    left = s["left_mm"]
    if 0 < left < WALL_STEER_MM:
        return ("WALL_STEER_RIGHT", DRIVE_SPEED, CORRECTION_ANGLE, f"Left wall {left:.0f}mm — steering right")

    # Right wall too close — steer left
    right = s["right_mm"]
    if 0 < right < WALL_STEER_MM:
        return ("WALL_STEER_LEFT", DRIVE_SPEED, -CORRECTION_ANGLE, f"Right wall {right:.0f}mm — steering left")

    # All clear — drive forward
    return ("FORWARD", DRIVE_SPEED, 0, "Clear")


# ── Logging ────────────────────────────────────────────────────────────────────
def build_log_entry(ts, elapsed, action, speed, angle, reason, sensors):
    s = parse_sensors(sensors) if sensors else {}
    return {
        "ts":       ts,
        "elapsed":  round(elapsed, 2),
        "action":   action,
        "speed":    speed,
        "angle":    angle,
        "reason":   reason,
        "lidar": {
            "front_mm": s.get("front_mm", 0),
            "back_mm":  s.get("back_mm",  0),
            "left_mm":  s.get("left_mm",  0),
            "right_mm": s.get("right_mm", 0),
        },
        "grayscale": {
            "left":   s.get("gs_left",   0),
            "center": s.get("gs_center", 0),
            "right":  s.get("gs_right",  0),
        },
        "ultrasonic_cm": s.get("us_cm", 0),
    }


def write_kb_log(session_data, log_path):
    """Write KB-ingestible session summary."""
    entries  = session_data["entries"]
    duration = session_data["duration_s"]

    action_counts = {}
    for e in entries:
        a = e.get("action", "UNKNOWN")
        action_counts[a] = action_counts.get(a, 0) + 1

    tape_stops    = action_counts.get("TAPE_STOP", 0)
    wall_turns    = action_counts.get("WALL_TURN", 0) + action_counts.get("WALL_SLOW", 0)
    forward_count = action_counts.get("FORWARD", 0)
    total         = len(entries)

    summary = (
        f"Roam session in {ENVIRONMENT} environment. "
        f"Duration: {duration:.0f}s, {total} decisions at {POLL_HZ}Hz. "
        f"Forward: {forward_count} ({100*forward_count//total if total else 0}%), "
        f"Wall avoidance: {wall_turns}, "
        f"Tape stops: {tape_stops}. "
        f"{'Car stayed within bounds.' if tape_stops == 0 else f'Hit tape boundary {tape_stops} time(s).'}"
    )

    log = {
        "type":        "roam_session",
        "environment": ENVIRONMENT,
        "timestamp":   session_data["start_time"],
        "summary":     summary,
        "metadata": {
            "duration_s":     duration,
            "total_decisions": total,
            "action_counts":  action_counts,
            "tape_stops":     tape_stops,
            "wall_avoidances": wall_turns,
            "config": session_data["config"],
        },
        "entries": entries,
    }

    with open(log_path, "w") as f:
        json.dump(log, f, indent=2)


# ── Main ───────────────────────────────────────────────────────────────────────
def main(duration=SESSION_DURATION, speed=DRIVE_SPEED, env=None):
    global ENVIRONMENT, DRIVE_SPEED
    if env:
        ENVIRONMENT = env
    DRIVE_SPEED = speed

    log_filename = f"roam_{ENVIRONMENT}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    log_path     = os.path.join(LOG_DIR, log_filename)
    os.makedirs(LOG_DIR, exist_ok=True)

    print("=" * 60)
    print("  PiCar-X Roam Navigator")
    print(f"  Environment: {ENVIRONMENT}")
    print(f"  Agent:       {AGENT_URL}")
    print(f"  Duration:    {duration}s")
    print(f"  Speed:       {speed}")
    print(f"  Log:         {log_path}")
    print("=" * 60)

    status = api("/api/status")
    if not status:
        print("ERROR: Cannot reach Pi agent.")
        sys.exit(1)
    print(f"\nBattery: {status.get('battery_v','?')}V ({status.get('battery_pct','?')}%)")

    estop = api("/api/estop")
    if estop and estop.get("estop_active"):
        print("ERROR: ESTOP active.")
        sys.exit(1)

    print("\nStarting in 3 seconds — place car in starting position...")
    print("Press Ctrl+C to stop safely.\n")
    time.sleep(3)

    set_mode("autonomous")

    session_data = {
        "start_time": datetime.now().isoformat(),
        "environment": ENVIRONMENT,
        "duration_s": 0,
        "config": {
            "drive_speed":    speed,
            "duration":       duration,
            "wall_stop_mm":   WALL_STOP_MM,
            "wall_slow_mm":   WALL_SLOW_MM,
            "wall_steer_mm":  WALL_STEER_MM,
        },
        "entries": [],
    }

    session_start  = time.time()
    loop_interval  = 1.0 / POLL_HZ
    current_speed  = 0
    current_angle  = 0
    last_turn_time = 0
    tape_stopped   = False

    try:
        while True:
            loop_start = time.time()
            elapsed    = loop_start - session_start

            if elapsed >= duration:
                print(f"\nSession complete ({duration}s).")
                break

            remaining = int(duration - elapsed)
            sensors   = api("/api/sensors")

            if sensors is None:
                print(f"  [{remaining:3d}s] sensor error")
                time.sleep(loop_interval)
                continue

            s      = parse_sensors(sensors)
            action, spd, ang, reason = decide(s, last_turn_time)
            now    = time.time()
            ts     = datetime.now().isoformat(timespec="milliseconds")

            # Handle tape emergency
            if action == "TAPE_STOP":
                if not tape_stopped:
                    stop()
                    tape_stopped  = True
                    current_speed = 0
                    current_angle = 0
                    print(f"\n  [{remaining:3d}s] TAPE STOP — backing up...")

                    # Back up
                    time.sleep(0.5)
                    drive(-DRIVE_SPEED, 0)
                    time.sleep(0.8)

                    # Turn toward open space
                    open_dir  = "left" if s["left_mm"] > s["right_mm"] else "right"
                    turn_ang  = -TURN_ANGLE if open_dir == "left" else TURN_ANGLE
                    drive(TURN_SPEED, turn_ang)
                    time.sleep(random.uniform(TURN_DURATION_MIN, TURN_DURATION_MAX))

                    drive(DRIVE_SPEED, 0)
                    tape_stopped  = False
                    last_turn_time = now

            elif action != "REFLEX":
                drive(spd, ang)
                current_speed = spd
                current_angle = ang
                if action in ("WALL_TURN",):
                    last_turn_time = now

            print(f"  [{remaining:3d}s] {action:<18} spd={spd:2d} ang={ang:3d}  "
                  f"front={s['front_mm']:.0f}mm  L={s['left_mm']:.0f}  R={s['right_mm']:.0f}",
                  end="\r")

            entry = build_log_entry(ts, elapsed, action, spd, ang, reason, sensors)
            session_data["entries"].append(entry)

            elapsed_loop = time.time() - loop_start
            time.sleep(max(0, loop_interval - elapsed_loop))

        stop()
        set_mode("manual")
        session_data["duration_s"] = round(time.time() - session_start, 1)
        write_kb_log(session_data, log_path)

        entries       = session_data["entries"]
        action_counts = {}
        for e in entries:
            a = e["action"]
            action_counts[a] = action_counts.get(a, 0) + 1

        print("\n" + "=" * 60)
        print("  SESSION SUMMARY")
        print("=" * 60)
        print(f"  Duration:       {session_data['duration_s']}s")
        print(f"  Total decisions: {len(entries)}")
        for action, count in sorted(action_counts.items(), key=lambda x: -x[1]):
            print(f"  {action:<20} {count}")
        print(f"\n  Log: {log_path}")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\nStopping...")
        stop()
        set_mode("manual")
        session_data["duration_s"] = round(time.time() - session_start, 1)
        write_kb_log(session_data, log_path)
        print(f"Log saved: {log_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PiCar-X Free Roam Navigator")
    parser.add_argument("--duration", type=int,   default=SESSION_DURATION, help="Session duration in seconds")
    parser.add_argument("--speed",    type=int,   default=DRIVE_SPEED,      help="Drive speed (0-100)")
    parser.add_argument("--env",      type=str,   default=None,             help="Environment name")
    args = parser.parse_args()

    if PI_IP == "YOUR_PI_IP":
        print("ERROR: config.py not found.")
        sys.exit(1)

    main(duration=args.duration, speed=args.speed, env=args.env)

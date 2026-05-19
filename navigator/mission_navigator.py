#!/usr/bin/env python3
"""
mission_navigator.py — PiCar-X Mission Navigator

Navigate autonomously from current position to a target zone.
Uses stored zone definitions and recorded path data as guidance.
Adapts to obstacles by using real-time LiDAR rather than following
the recorded path exactly.

Usage:
  python3 mission_navigator.py --target B
  python3 mission_navigator.py --target B --then C
  python3 mission_navigator.py --target B --then C --then A

All decisions logged in KB-ingestible format.
"""

import requests
import time
import json
import sys
import os
import argparse
from datetime import datetime
from statistics import mean

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))), )
try:
    from config import PI_IP, AGENT_PORT
except ImportError:
    PI_IP      = "YOUR_PI_IP"
    AGENT_PORT = 8000

AGENT_URL = f"http://{PI_IP}:{AGENT_PORT}"

ENVIRONMENT  = os.environ.get("PICAR_ENV", "garage")
DATA_DIR     = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
LOG_DIR      = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
ZONES_FILE   = os.path.join(DATA_DIR, f"zones_{ENVIRONMENT}.json")

# Navigation config
DRIVE_SPEED       = 20
TURN_SPEED        = 15
POLL_HZ           = 10
ZONE_REACHED_MM   = 450    # tolerance to consider zone reached (mm)
MAX_MISSION_TIME  = 300    # max seconds per zone leg
WALL_STOP_MM      = 400
WALL_SLOW_MM      = 800
WALL_STEER_MM     = 600
CORRECTION_ANGLE  = 15
TURN_ANGLE        = 30

# Grayscale tape thresholds
GS_THRESHOLD_LEFT   = 950
GS_THRESHOLD_CENTER = 850
GS_THRESHOLD_RIGHT  = 1000
GS_ZERO_IGNORE      = 50


def api(endpoint, method="get", params=None):
    try:
        url = f"{AGENT_URL}{endpoint}"
        r = requests.get(url, timeout=1.0) if method == "get" else requests.post(url, params=params, timeout=1.0)
        r.raise_for_status()
        return r.json()
    except Exception:
        return None


def drive(speed, angle):
    api("/api/drive", method="post", params={"speed": speed, "angle": angle})


def stop():
    api("/api/stop", method="post")


def set_mode(mode):
    api(f"/api/mode/{mode}", method="post")


def load_zones():
    if not os.path.exists(ZONES_FILE):
        print(f"ERROR: No zones file found at {ZONES_FILE}")
        print(f"Run zone_calibrate.py first to define zones.")
        sys.exit(1)
    with open(ZONES_FILE) as f:
        return json.load(f)


def get_current_lidar():
    """Get current LiDAR readings."""
    sensors = api("/api/sensors")
    if not sensors:
        return None
    lidar = sensors.get("lidar", {})
    return {
        "front_mm": lidar.get("front", 0) or 0,
        "back_mm":  lidar.get("back",  0) or 0,
        "left_mm":  lidar.get("left",  0) or 0,
        "right_mm": lidar.get("right", 0) or 0,
    }


def in_zone(current, zone):
    """Check if current LiDAR position matches zone definition within tolerance."""
    tol     = zone.get("tolerance_mm", ZONE_REACHED_MM)
    z_lidar = zone.get("lidar", {})
    checks  = []

    for axis in ["front_mm", "back_mm", "left_mm", "right_mm"]:
        z_val = z_lidar.get(axis, 0)
        c_val = current.get(axis, 0)
        if z_val > 0 and c_val > 0:
            checks.append(abs(z_val - c_val) < tol)

    # Need at least 2 axes to agree
    return len(checks) >= 2 and sum(checks) >= 2


def steer_toward_zone(current, zone):
    """
    Determine steering direction to move toward target zone.
    Compares current LiDAR readings to zone target readings.
    Returns (angle, reason)
    """
    z_lidar = zone.get("lidar", {})

    front_target = z_lidar.get("front_mm", 0)
    left_target  = z_lidar.get("left_mm",  0)
    right_target = z_lidar.get("right_mm", 0)
    back_target  = z_lidar.get("back_mm",  0)

    front_current = current.get("front_mm", 0)
    left_current  = current.get("left_mm",  0)
    right_current = current.get("right_mm", 0)
    back_current  = current.get("back_mm",  0)

    # Determine if target is to the left or right based on wall distances
    left_delta  = left_current  - left_target   # positive = currently further from left wall than target
    right_delta = right_current - right_target  # positive = currently further from right wall than target

    # Need to move left — steer left
    if left_delta > 300 and abs(left_delta) > abs(right_delta):
        return (-CORRECTION_ANGLE, f"Steering left toward zone (left delta={left_delta:.0f}mm)")

    # Need to move right — steer right
    if right_delta > 300 and abs(right_delta) > abs(left_delta):
        return (CORRECTION_ANGLE, f"Steering right toward zone (right delta={right_delta:.0f}mm)")

    return (0, "Driving straight toward zone")


def check_tape(sensors):
    """Check grayscale for tape emergency."""
    gs = sensors.get("grayscale", [0,0,0])
    def valid(v): return v if v and v > GS_ZERO_IGNORE else None
    left   = valid(gs[0] if len(gs) > 0 else None)
    center = valid(gs[1] if len(gs) > 1 else None)
    right  = valid(gs[2] if len(gs) > 2 else None)
    left_tape   = left   is not None and left   > GS_THRESHOLD_LEFT
    center_tape = center is not None and center > GS_THRESHOLD_CENTER
    right_tape  = right  is not None and right  > GS_THRESHOLD_RIGHT
    return (left_tape and center_tape) or (right_tape and center_tape)


def navigate_to_zone(zone_name, zone, mission_log):
    """Navigate to a specific zone. Returns True if reached, False if timeout."""
    print(f"\nNavigating to Zone {zone_name}...")
    print(f"  Target: front={zone['lidar']['front_mm']:.0f}mm  "
          f"left={zone['lidar']['left_mm']:.0f}mm  "
          f"right={zone['lidar']['right_mm']:.0f}mm")

    start_time = time.time()
    leg_entries = []

    while True:
        elapsed = time.time() - start_time

        if elapsed > MAX_MISSION_TIME:
            print(f"\n  Timeout reaching Zone {zone_name} ({MAX_MISSION_TIME}s)")
            return False, leg_entries

        sensors = api("/api/sensors")
        if not sensors:
            time.sleep(1 / POLL_HZ)
            continue

        lidar = sensors.get("lidar", {})
        current = {
            "front_mm": lidar.get("front", 0) or 0,
            "back_mm":  lidar.get("back",  0) or 0,
            "left_mm":  lidar.get("left",  0) or 0,
            "right_mm": lidar.get("right", 0) or 0,
        }

        # Check if we've arrived
        if in_zone(current, zone):
            stop()
            print(f"\n  ✓ Zone {zone_name} reached in {elapsed:.1f}s")
            entry = {
                "ts":      datetime.now().isoformat(timespec="milliseconds"),
                "elapsed": round(elapsed, 2),
                "action":  "ZONE_REACHED",
                "zone":    zone_name,
                "lidar":   current,
            }
            leg_entries.append(entry)
            return True, leg_entries

        # Check tape emergency
        if check_tape(sensors):
            stop()
            print(f"\n  TAPE STOP — backing up...")
            time.sleep(0.5)
            drive(-DRIVE_SPEED, 0)
            time.sleep(0.8)
            drive(TURN_SPEED, TURN_ANGLE)
            time.sleep(0.8)
            entry = {
                "ts":      datetime.now().isoformat(timespec="milliseconds"),
                "elapsed": round(elapsed, 2),
                "action":  "TAPE_STOP",
                "lidar":   current,
            }
            leg_entries.append(entry)
            continue

        # Wall avoidance
        front = current["front_mm"]
        left  = current["left_mm"]
        right = current["right_mm"]

        if 0 < front < WALL_STOP_MM:
            turn_dir = "left" if left > right else "right"
            angle    = -TURN_ANGLE if turn_dir == "left" else TURN_ANGLE
            drive(TURN_SPEED, angle)
            action = f"WALL_AVOID_{turn_dir.upper()}"
            reason = f"Front wall {front:.0f}mm"
        elif 0 < left < WALL_STEER_MM:
            drive(DRIVE_SPEED, CORRECTION_ANGLE)
            action = "WALL_STEER_RIGHT"
            reason = f"Left wall {left:.0f}mm"
        elif 0 < right < WALL_STEER_MM:
            drive(DRIVE_SPEED, -CORRECTION_ANGLE)
            action = "WALL_STEER_LEFT"
            reason = f"Right wall {right:.0f}mm"
        else:
            # Steer toward zone
            spd   = DRIVE_SPEED if front == 0 or front > WALL_SLOW_MM else max(TURN_SPEED, int(DRIVE_SPEED * (front / WALL_SLOW_MM)))
            angle, reason = steer_toward_zone(current, zone)
            drive(spd, angle)
            action = "DRIVE_TO_ZONE"

        print(f"  [{elapsed:5.1f}s] {action:<22} front={front:.0f}mm L={left:.0f} R={right:.0f}  {reason[:30]}", end="\r")

        entry = {
            "ts":      datetime.now().isoformat(timespec="milliseconds"),
            "elapsed": round(elapsed, 2),
            "action":  action,
            "reason":  reason,
            "lidar":   current,
        }
        leg_entries.append(entry)
        time.sleep(1 / POLL_HZ)


def main(targets, env=None):
    global ENVIRONMENT, ZONES_FILE
    if env:
        ENVIRONMENT = env
        ZONES_FILE  = os.path.join(DATA_DIR, f"zones_{ENVIRONMENT}.json")

    log_path = os.path.join(
        LOG_DIR,
        f"mission_{ENVIRONMENT}_{'-'.join(targets)}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )
    os.makedirs(LOG_DIR, exist_ok=True)

    print("=" * 60)
    print("  PiCar-X Mission Navigator")
    print(f"  Environment: {ENVIRONMENT}")
    print(f"  Mission:     {'→'.join(targets)}")
    print(f"  Log:         {log_path}")
    print("=" * 60)

    # Load zones
    zone_data = load_zones()
    zones     = zone_data.get("zones", {})

    for target in targets:
        if target.upper() not in zones:
            print(f"ERROR: Zone {target} not defined. Run zone_calibrate.py first.")
            sys.exit(1)

    status = api("/api/status")
    if not status:
        print("ERROR: Cannot reach Pi agent.")
        sys.exit(1)
    print(f"\nBattery: {status.get('battery_v','?')}V")

    print("\nStarting mission in 3 seconds...")
    time.sleep(3)
    set_mode("autonomous")

    mission_start = time.time()
    mission_log   = {
        "type":        "mission",
        "environment": ENVIRONMENT,
        "timestamp":   datetime.now().isoformat(),
        "targets":     targets,
        "legs":        [],
    }

    all_reached = True
    for target in targets:
        zone     = zones[target.upper()]
        reached, leg_entries = navigate_to_zone(target.upper(), zone, mission_log)

        leg = {
            "target":   target.upper(),
            "reached":  reached,
            "entries":  leg_entries,
            "duration": leg_entries[-1]["elapsed"] if leg_entries else 0,
        }
        mission_log["legs"].append(leg)

        if not reached:
            all_reached = False
            print(f"\nMission aborted — could not reach Zone {target}.")
            break

        if target != targets[-1]:
            print(f"\nPausing 2 seconds before next leg...")
            time.sleep(2)

    stop()
    set_mode("manual")

    total_duration = round(time.time() - mission_start, 1)
    mission_log["duration_s"] = total_duration
    mission_log["completed"]  = all_reached

    reached_zones = [l["target"] for l in mission_log["legs"] if l["reached"]]
    summary = (
        f"Mission {'completed' if all_reached else 'aborted'} in {ENVIRONMENT} environment. "
        f"Targets: {' → '.join(targets)}. "
        f"Reached: {', '.join(reached_zones) if reached_zones else 'none'}. "
        f"Total duration: {total_duration}s."
    )
    mission_log["summary"] = summary

    with open(log_path, "w") as f:
        json.dump(mission_log, f, indent=2)

    print("\n" + "=" * 60)
    print("  MISSION SUMMARY")
    print("=" * 60)
    print(f"  Status:   {'COMPLETED' if all_reached else 'ABORTED'}")
    print(f"  Duration: {total_duration}s")
    print(f"  Reached:  {', '.join(reached_zones) if reached_zones else 'none'}")
    print(f"  Log:      {log_path}")
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PiCar-X Mission Navigator")
    parser.add_argument("--target", required=True,      help="First target zone (A, B, C)")
    parser.add_argument("--then",   action="append",    help="Additional zones in sequence", default=[])
    parser.add_argument("--env",    default=None,       help="Environment name")
    args = parser.parse_args()

    if PI_IP == "YOUR_PI_IP":
        print("ERROR: config.py not found.")
        sys.exit(1)

    targets = [args.target] + args.then
    main(targets, env=args.env)

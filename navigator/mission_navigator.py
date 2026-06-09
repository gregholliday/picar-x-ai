#!/usr/bin/env python3
"""
mission_navigator.py — PiCar-X Mission Navigator v2

World-coordinate based navigation using compass + LiDAR.
Zones are defined by N/S/E/W wall distances (heading-independent).
Navigation uses compass heading for reliable orientation.

Usage:
  python3 mission_navigator.py --target B
  python3 mission_navigator.py --target B --then C --then A
  python3 mission_navigator.py --target B --env garage
"""

import requests
import time
import json
import sys
import os
import math
import argparse
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from config import PI_IP, AGENT_PORT
except ImportError:
    PI_IP      = "YOUR_PI_IP"
    AGENT_PORT = 8000

AGENT_URL   = f"http://{PI_IP}:{AGENT_PORT}"
ENVIRONMENT = os.environ.get("PICAR_ENV", "garage")
DATA_DIR    = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
LOG_DIR     = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
ZONES_FILE  = os.path.join(DATA_DIR, f"zones_{ENVIRONMENT}.json")

# ── Navigation config ──────────────────────────────────────────────────────────
DRIVE_SPEED         = 15
TURN_SPEED          = 12
POLL_HZ             = 10
MAX_MISSION_TIME    = 300     # max seconds per zone leg
ZONE_REACHED_MM     = 400     # world distance match tolerance
HEADING_TOLERANCE   = 15      # degrees — acceptable heading error
WALL_STOP_MM        = 300     # emergency stop distance
WALL_SLOW_MM        = 800     # slow down distance
WALL_STEER_MM       = 250     # side wall steer distance
CORRECTION_ANGLE    = 15      # gentle correction steering angle
TURN_ANGLE          = 30      # full turn angle

# Grayscale tape emergency stop thresholds
GS_THRESHOLD_LEFT   = 950
GS_THRESHOLD_CENTER = 850
GS_THRESHOLD_RIGHT  = 1000
GS_ZERO_IGNORE      = 50


# ── API helpers ────────────────────────────────────────────────────────────────
def api(endpoint, method="get", params=None):
    try:
        url = f"{AGENT_URL}{endpoint}"
        r   = requests.get(url, timeout=1.0) if method == "get" else requests.post(url, params=params, timeout=1.0)
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


# ── Coordinate conversion ──────────────────────────────────────────────────────
def lidar_to_world(lidar, heading_deg):
    """
    Convert car-relative LiDAR to world coordinates (N/S/E/W).
    heading_deg = compass heading of car's front.
    """
    h     = round(heading_deg / 90) * 90 % 360
    front = lidar.get("front_mm", 0) or 0
    back  = lidar.get("back_mm",  0) or 0
    left  = lidar.get("left_mm",  0) or 0
    right = lidar.get("right_mm", 0) or 0

    if h == 0:
        return {"N": front, "E": right, "S": back,  "W": left}
    elif h == 90:
        return {"N": left,  "E": front, "S": right, "W": back}
    elif h == 180:
        return {"N": back,  "E": left,  "S": front, "W": right}
    else:  # 270
        return {"N": right, "E": back,  "S": left,  "W": front}


def heading_diff(current, target):
    """Signed heading difference (-180 to +180). Positive = turn right."""
    return (target - current + 180) % 360 - 180


def bearing_to_zone(current_world, zone_world):
    """
    Calculate the compass bearing to drive toward the target zone.

    World coordinates tell us distance to each wall.
    To find relative position between current and target:
      - If zone S < current S: zone is further from south wall = zone is north of us
      - If zone N < current N: zone is further from north wall = zone is south of us
      - If zone W < current W: zone is further from west wall  = zone is east of us
      - If zone E < current E: zone is further from east wall  = zone is west of us

    Use both N/S pair and E/W pair and average for robustness.
    Skip open-space readings (>2000mm) as they are unreliable for position.
    """
    # North/South component
    # Larger S = closer to south wall = further north
    # Zone S < current S means zone is further north of us → need_north positive
    c_s = current_world.get("S", 0) or 0
    z_s = zone_world.get("S", 0) or 0
    c_n = current_world.get("N", 0) or 0
    z_n = zone_world.get("N", 0) or 0

    # East/West component
    c_w = current_world.get("W", 0) or 0
    z_w = zone_world.get("W", 0) or 0
    c_e = current_world.get("E", 0) or 0
    z_e = zone_world.get("E", 0) or 0

    # Calculate N/S delta — use S wall if both are valid wall readings
    delta_n = 0
    if 0 < c_s < 2000 and 0 < z_s < 2000:
        delta_n = c_s - z_s   # positive = zone is further from S wall = zone is north
    elif 0 < c_n < 2000 and 0 < z_n < 2000:
        delta_n = z_n - c_n   # positive = zone is further from N wall = zone is north

    # Calculate E/W delta — use W wall if both are valid wall readings
    delta_e = 0
    if 0 < c_w < 2000 and 0 < z_w < 2000:
        delta_e = c_w - z_w   # positive = zone is further from W wall = zone is east
    elif 0 < c_e < 2000 and 0 < z_e < 2000:
        delta_e = z_e - c_e   # positive = zone is further from E wall = zone is east

    if abs(delta_n) < 150 and abs(delta_e) < 150:
        return None  # Already close enough

    bearing = math.degrees(math.atan2(delta_e, delta_n)) % 360
    return round(bearing, 1)


# ── Zone detection ─────────────────────────────────────────────────────────────
def in_zone(current_world, zone):
    """
    Check if current world position matches zone definition.
    Heading-independent — compares N/S/E/W distances directly.
    Requires 3 of 4 world directions to match within tolerance.
    """
    tol        = ZONE_REACHED_MM
    zone_world = zone.get("world_distances")

    if not zone_world:
        # Fall back to car-relative matching if no world distances
        return in_zone_lidar_fallback(current_world, zone)

    matched = 0
    checked = 0

    for direction in ["N", "S", "E", "W"]:
        z_val = zone_world.get(direction, 0) or 0
        c_val = current_world.get(direction, 0) or 0

        if z_val <= 0 or c_val <= 0:
            continue

        checked += 1

        if z_val > 2000:
            # Zone expects open space — current must also see open space
            if c_val > 2000:
                matched += 1
            else:
                # Current sees wall where zone expects open space — definite mismatch
                return False
        elif c_val > 2000:
            # Current sees open space where zone expects wall — definite mismatch
            return False
        else:
            if abs(z_val - c_val) < tol:
                matched += 1

    # Require ALL checked axes to match
    return checked >= 3 and matched >= checked


def in_zone_lidar_fallback(current_world, zone):
    """Fallback zone detection using stored LiDAR readings (no world coords)."""
    tol    = ZONE_REACHED_MM
    z_lidar = zone.get("lidar", {})

    zone_vals    = [v for v in [z_lidar.get(k, 0) for k in
                    ["front_mm", "back_mm", "left_mm", "right_mm"]] if v and v > 0]
    current_vals = [v for v in current_world.values() if v and v > 0]

    if len(zone_vals) < 3 or len(current_vals) < 3:
        return False

    matched  = 0
    used_idx = set()

    for z_val in zone_vals:
        for i, c_val in enumerate(current_vals):
            if i in used_idx:
                continue
            if z_val > 3000:
                if c_val > 2000:
                    matched += 1
                    used_idx.add(i)
                    break
            elif c_val <= 3000 and abs(z_val - c_val) < tol:
                matched += 1
                used_idx.add(i)
                break

    return matched >= 3


# ── Tape detection ─────────────────────────────────────────────────────────────
def check_tape(sensors):
    gs = sensors.get("grayscale", [0, 0, 0])
    def valid(v): return v if v and v > GS_ZERO_IGNORE else None
    left   = valid(gs[0] if len(gs) > 0 else None)
    center = valid(gs[1] if len(gs) > 1 else None)
    right  = valid(gs[2] if len(gs) > 2 else None)
    left_t   = left   is not None and left   > GS_THRESHOLD_LEFT
    center_t = center is not None and center > GS_THRESHOLD_CENTER
    right_t  = right  is not None and right  > GS_THRESHOLD_RIGHT
    return (left_t and center_t) or (right_t and center_t)


# ── Main navigation loop ───────────────────────────────────────────────────────
def navigate_to_zone(zone_name, zone, compass_available):
    """Navigate to a zone. Returns (reached, entries)."""
    print(f"\nNavigating to Zone {zone_name}...")

    zone_world = zone.get("world_distances", {})
    zone_heading = zone.get("heading_deg")

    if zone_world:
        print(f"  Target world: N={zone_world.get('N',0):.0f}  S={zone_world.get('S',0):.0f}  "
              f"E={zone_world.get('E',0):.0f}  W={zone_world.get('W',0):.0f}mm")
    if zone_heading:
        print(f"  Target heading: {zone_heading}°")

    start_time  = time.time()
    leg_entries = []
    orient_done = False

    while True:
        elapsed = time.time() - start_time

        if elapsed > MAX_MISSION_TIME:
            print(f"\n  Timeout reaching Zone {zone_name}")
            return False, leg_entries

        sensors = api("/api/sensors")
        if not sensors:
            time.sleep(1 / POLL_HZ)
            continue

        lidar   = sensors.get("lidar", {})
        heading = sensors.get("compass_heading")

        current_lidar = {
            "front_mm": lidar.get("front", 0) or 0,
            "back_mm":  lidar.get("back",  0) or 0,
            "left_mm":  lidar.get("left",  0) or 0,
            "right_mm": lidar.get("right", 0) or 0,
        }

        # Convert to world coordinates
        if heading is not None:
            current_world = lidar_to_world(current_lidar, heading)
        else:
            # No compass — use car-relative as world proxy
            current_world = {
                "N": current_lidar["front_mm"],
                "S": current_lidar["back_mm"],
                "E": current_lidar["right_mm"],
                "W": current_lidar["left_mm"],
            }

        # ── Check arrival ──────────────────────────────────────────────────────
        if in_zone(current_world, zone):
            stop()
            print(f"\n  ✓ Zone {zone_name} reached in {elapsed:.1f}s")
            if heading is not None:
                print(f"    Final heading: {heading:.1f}°  "
                      f"(calibrated: {zone_heading}°)")
            leg_entries.append({
                "ts":      datetime.now().isoformat(timespec="milliseconds"),
                "elapsed": round(elapsed, 2),
                "action":  "ZONE_REACHED",
                "zone":    zone_name,
                "world":   current_world,
                "heading": heading,
            })
            return True, leg_entries

        # ── Tape emergency stop ────────────────────────────────────────────────
        if check_tape(sensors):
            stop()
            print(f"\n  TAPE STOP at t={elapsed:.1f}s — backing up...")
            time.sleep(0.5)
            drive(-DRIVE_SPEED, 0)
            time.sleep(1.0)

            # Turn toward most open world direction
            max_dir = max(current_world, key=lambda k: current_world.get(k, 0))
            if max_dir in ("N", "E"):
                turn = TURN_ANGLE
            else:
                turn = -TURN_ANGLE
            drive(TURN_SPEED, turn)
            time.sleep(0.8)

            leg_entries.append({
                "ts":      datetime.now().isoformat(timespec="milliseconds"),
                "elapsed": round(elapsed, 2),
                "action":  "TAPE_STOP",
                "world":   current_world,
                "heading": heading,
            })
            orient_done = False  # re-orient after tape stop
            continue

        front = current_lidar["front_mm"]
        left  = current_lidar["left_mm"]
        right = current_lidar["right_mm"]

        # ── Emergency wall stop ────────────────────────────────────────────────
        ultrasonic_cm = sensors.get("ultrasonic_cm", 0) or 0
        obstacle_close = sensors.get("obstacle_close", False)

        if (0 < front < WALL_STOP_MM) or (obstacle_close and 0 < ultrasonic_cm < 15):
            turn_dir = "left" if left > right else "right"
            angle    = -TURN_ANGLE if turn_dir == "left" else TURN_ANGLE
            drive(TURN_SPEED, angle)
            action = f"WALL_AVOID_{turn_dir.upper()}"
            reason = f"Front wall {front:.0f}mm ultrasonic={ultrasonic_cm:.0f}cm"
            # Don't reset orient_done — keep heading reference after obstacle avoidance

        # ── Phase 1: Orient toward zone heading ────────────────────────────────
        elif not orient_done and heading is not None and zone_heading is not None:
            diff = heading_diff(heading, zone_heading)

            if abs(diff) <= HEADING_TOLERANCE:
                orient_done = True
                drive(DRIVE_SPEED, 0)
                action = "ORIENTED"
                reason = f"Heading {heading:.1f}° ≈ target {zone_heading:.1f}°"
            else:
                angle  = TURN_ANGLE if diff > 0 else -TURN_ANGLE
                drive(TURN_SPEED, angle)
                action = "ORIENT_TO_ZONE"
                reason = f"Rotating {'right' if diff > 0 else 'left'} — current={heading:.1f}° target={zone_heading:.1f}° diff={diff:.1f}°"

        # ── Phase 2: Drive toward zone ─────────────────────────────────────────
        else:
            orient_done = True  # skip orientation if no compass

            # Side wall avoidance
            if 0 < left < WALL_STEER_MM:
                angle  = CORRECTION_ANGLE
                action = "WALL_STEER_RIGHT"
                reason = f"Left wall {left:.0f}mm"
            elif 0 < right < WALL_STEER_MM:
                angle  = -CORRECTION_ANGLE
                action = "WALL_STEER_LEFT"
                reason = f"Right wall {right:.0f}mm"
            else:
                # Steer based on world coordinate deltas
                # Compare current readings to zone targets on each valid axis
                # to determine left/right correction needed
                steer = 0
                steer_reason = []

                if zone_world:
                    # Check each axis — if current reading differs from target,
                    # steer to correct. Skip open-space readings (>2000mm).
                    for direction, car_axis, steer_sign in [
                        ("W", "right_mm", +1),   # Too little W → steer right (positive)
                        ("E", "left_mm",  -1),   # Too little E → steer left (negative)
                    ]:
                        z_val = zone_world.get(direction, 0) or 0
                        c_val = current_world.get(direction, 0) or 0

                        # Skip open-space readings
                        if z_val > 2000 or c_val > 2000:
                            continue
                        if z_val <= 0 or c_val <= 0:
                            continue

                        delta = z_val - c_val  # positive = need more distance on this side
                        if abs(delta) > 200:
                            steer += steer_sign * CORRECTION_ANGLE
                            steer_reason.append(f"{direction}:{c_val:.0f}→{z_val:.0f}")

                if steer != 0:
                    angle  = max(-CORRECTION_ANGLE, min(CORRECTION_ANGLE, steer))
                    action = "STEER_TO_ZONE"
                    reason = f"Correcting: {', '.join(steer_reason)}"
                else:
                    angle  = 0
                    action = "DRIVE_TO_ZONE"
                    reason = "Driving straight to zone"

            spd = DRIVE_SPEED if front == 0 or front > WALL_SLOW_MM else max(TURN_SPEED, int(DRIVE_SPEED * (front / WALL_SLOW_MM)))
            drive(spd, angle)

        print(f"  [{elapsed:5.1f}s] {action:<22} "
              f"{'hdg='+str(round(heading,1))+'°' if heading else 'no compass':<12} "
              f"N={current_world.get('N',0):.0f} S={current_world.get('S',0):.0f} "
              f"E={current_world.get('E',0):.0f} W={current_world.get('W',0):.0f}",
              end="\r")

        leg_entries.append({
            "ts":      datetime.now().isoformat(timespec="milliseconds"),
            "elapsed": round(elapsed, 2),
            "action":  action,
            "reason":  reason if 'reason' in dir() else "",
            "world":   current_world,
            "heading": heading,
            "lidar":   current_lidar,
        })

        time.sleep(1 / POLL_HZ)


def main(targets, env=None):
    global ENVIRONMENT, ZONES_FILE
    if env:
        ENVIRONMENT = env
        ZONES_FILE  = os.path.join(DATA_DIR, f"zones_{ENVIRONMENT}.json")

    log_path = os.path.join(
        LOG_DIR,
        f"mission_{ENVIRONMENT}_{'_'.join(targets)}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )
    os.makedirs(LOG_DIR, exist_ok=True)

    print("=" * 60)
    print("  PiCar-X Mission Navigator v2")
    print(f"  Environment: {ENVIRONMENT}")
    print(f"  Mission:     {'→'.join(targets)}")
    print(f"  Log:         {log_path}")
    print("=" * 60)

    # Load zones
    if not os.path.exists(ZONES_FILE):
        print(f"ERROR: No zones file at {ZONES_FILE}. Run zone_calibrate.py first.")
        sys.exit(1)

    with open(ZONES_FILE) as f:
        zone_data = json.load(f)
    zones = zone_data.get("zones", {})

    for target in targets:
        if target.upper() not in zones:
            print(f"ERROR: Zone {target} not defined.")
            sys.exit(1)
        z = zones[target.upper()]
        if not z.get("world_distances"):
            print(f"WARNING: Zone {target} has no world coordinates.")
            print(f"  Re-calibrate with compass installed for best results.")

    status = api("/api/status")
    if not status:
        print("ERROR: Cannot reach Pi agent.")
        sys.exit(1)

    compass_ok = status.get("compass_ok", False)
    print(f"\nBattery: {status.get('battery_v','?')}V")
    print(f"Compass: {'OK ✓' if compass_ok else 'NOT AVAILABLE ✗'}")

    if not compass_ok:
        print("\nWARNING: Compass not available.")
        print("Navigation will use LiDAR-only fallback (less reliable).")
        print("Install and calibrate GY-271 for best results.\n")

    print("\nStarting mission in 3 seconds...")
    time.sleep(3)
    set_mode("autonomous")

    mission_start = time.time()
    mission_log   = {
        "type":          "mission",
        "environment":   ENVIRONMENT,
        "timestamp":     datetime.now().isoformat(),
        "targets":       targets,
        "compass_used":  compass_ok,
        "legs":          [],
    }

    all_reached = True
    try:
        for target in targets:
            zone    = zones[target.upper()]
            reached, entries = navigate_to_zone(target.upper(), zone, compass_ok)

            mission_log["legs"].append({
                "target":   target.upper(),
                "reached":  reached,
                "entries":  entries,
                "duration": entries[-1]["elapsed"] if entries else 0,
            })

            if not reached:
                all_reached = False
                print(f"\nMission aborted — could not reach Zone {target}.")
                break

            if target != targets[-1]:
                print(f"\nPausing 2 seconds before next leg...")
                time.sleep(2)

    except KeyboardInterrupt:
        print("\nMission interrupted by user.")
        all_reached = False
    finally:
        stop()
        set_mode("manual")

    total = round(time.time() - mission_start, 1)
    reached_zones = [l["target"] for l in mission_log["legs"] if l["reached"]]

    mission_log["duration_s"] = total
    mission_log["completed"]  = all_reached
    mission_log["summary"]    = (
        f"Mission {'completed' if all_reached else 'aborted'} in {ENVIRONMENT}. "
        f"Targets: {' → '.join(targets)}. "
        f"Reached: {', '.join(reached_zones) if reached_zones else 'none'}. "
        f"Duration: {total}s. Compass: {'used' if compass_ok else 'not available'}."
    )

    with open(log_path, "w") as f:
        json.dump(mission_log, f, indent=2)

    print("\n" + "=" * 60)
    print("  MISSION SUMMARY")
    print("=" * 60)
    print(f"  Status:    {'COMPLETED' if all_reached else 'ABORTED'}")
    print(f"  Duration:  {total}s")
    print(f"  Reached:   {', '.join(reached_zones) if reached_zones else 'none'}")
    print(f"  Compass:   {'used' if compass_ok else 'not available'}")
    print(f"  Log:       {log_path}")
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PiCar-X Mission Navigator v2")
    parser.add_argument("--target", required=True,   help="First target zone")
    parser.add_argument("--then",   action="append", help="Additional zones", default=[])
    parser.add_argument("--env",    default=None,    help="Environment name")
    args = parser.parse_args()

    if PI_IP == "YOUR_PI_IP":
        print("ERROR: config.py not found.")
        sys.exit(1)

    main([args.target] + args.then, env=args.env)
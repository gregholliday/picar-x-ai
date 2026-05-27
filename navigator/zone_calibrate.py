#!/usr/bin/env python3
"""
zone_calibrate.py — PiCar-X Zone Calibration Tool v2

Calibrates named zones using LiDAR + compass heading.
Stores world coordinates (N/S/E/W wall distances) so zone matching
is heading-independent during navigation.

Usage:
  python3 zone_calibrate.py --zone A --description "Back left corner"
  python3 zone_calibrate.py --zone B --description "Front left near door"
  python3 zone_calibrate.py --list
  python3 zone_calibrate.py --delete A

Requires compass to be wired and calibrated before use.
Falls back to car-relative coordinates if compass unavailable.
"""

import requests
import time
import json
import sys
import os
import math
import argparse
from datetime import datetime
from statistics import mean

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from config import PI_IP, AGENT_PORT
except ImportError:
    PI_IP      = "YOUR_PI_IP"
    AGENT_PORT = 8000

AGENT_URL    = f"http://{PI_IP}:{AGENT_PORT}"
ENVIRONMENT  = os.environ.get("PICAR_ENV", "garage")
DATA_DIR     = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
LOG_DIR      = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
ZONES_FILE   = os.path.join(DATA_DIR, f"zones_{ENVIRONMENT}.json")
SAMPLES      = 20
SAMPLE_HZ    = 5
TOLERANCE_MM = 300


def api(endpoint, method="get", params=None):
    try:
        url = f"{AGENT_URL}{endpoint}"
        r   = requests.get(url, timeout=2.0) if method == "get" else requests.post(url, params=params, timeout=2.0)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        print(f"  [API ERROR] {endpoint}: {e}")
        return None


def load_zones():
    if os.path.exists(ZONES_FILE):
        with open(ZONES_FILE) as f:
            return json.load(f)
    return {"environment": ENVIRONMENT, "zones": {}, "created": datetime.now().isoformat()}


def save_zones(data):
    os.makedirs(DATA_DIR, exist_ok=True)
    data["updated"] = datetime.now().isoformat()
    with open(ZONES_FILE, "w") as f:
        json.dump(data, f, indent=2)


def lidar_to_world(lidar, heading_deg):
    """
    Convert car-relative LiDAR readings to world coordinates.
    Uses compass heading to determine which physical wall each axis sees.

    heading_deg: compass heading of car's FRONT (0=N, 90=E, 180=S, 270=W)

    Returns dict: {"N": mm, "S": mm, "E": mm, "W": mm}
    """
    # Snap heading to nearest 45 degrees for robust mapping
    # Use 90-degree snapping for clean axis alignment
    h = round(heading_deg / 90) * 90 % 360

    front = lidar.get("front_mm", 0) or 0
    back  = lidar.get("back_mm",  0) or 0
    left  = lidar.get("left_mm",  0) or 0
    right = lidar.get("right_mm", 0) or 0

    if h == 0:    # facing North: F=N, R=E, B=S, L=W
        return {"N": front, "E": right, "S": back,  "W": left}
    elif h == 90:  # facing East:  F=E, R=S, B=W, L=N
        return {"N": left,  "E": front, "S": right, "W": back}
    elif h == 180: # facing South: F=S, R=W, B=N, L=E
        return {"N": back,  "E": left,  "S": front, "W": right}
    else:          # facing West:  F=W, R=N, B=E, L=S
        return {"N": right, "E": back,  "S": left,  "W": front}


def world_to_car(world, heading_deg):
    """
    Convert world coordinates back to car-relative for drive commands.
    Inverse of lidar_to_world.
    """
    h = round(heading_deg / 90) * 90 % 360
    N = world.get("N", 0)
    S = world.get("S", 0)
    E = world.get("E", 0)
    W = world.get("W", 0)

    if h == 0:    # facing North
        return {"front_mm": N, "right_mm": E, "back_mm": S, "left_mm": W}
    elif h == 90:  # facing East
        return {"front_mm": E, "right_mm": S, "back_mm": W, "left_mm": N}
    elif h == 180: # facing South
        return {"front_mm": S, "right_mm": W, "back_mm": N, "left_mm": E}
    else:          # facing West
        return {"front_mm": W, "right_mm": N, "back_mm": E, "left_mm": S}


def take_readings():
    """Take averaged LiDAR + compass readings."""
    print(f"  Taking {SAMPLES} readings at {SAMPLE_HZ}Hz...")
    lidar_readings   = {"front": [], "back": [], "left": [], "right": []}
    heading_readings = []

    for i in range(SAMPLES):
        sensors = api("/api/sensors")
        if sensors:
            lidar = sensors.get("lidar", {})
            for k in ["front", "back", "left", "right"]:
                v = lidar.get(k, 0)
                if v and v > 0:
                    lidar_readings[k].append(v)

            heading = sensors.get("compass_heading")
            if heading is not None:
                heading_readings.append(heading)

        print(f"    [{i+1:2d}/{SAMPLES}]", end="\r")
        time.sleep(1 / SAMPLE_HZ)

    print()

    # Average LiDAR
    avg_lidar = {}
    for k in ["front", "back", "left", "right"]:
        vals = lidar_readings[k]
        avg_lidar[f"{k}_mm"] = round(mean(vals), 1) if vals else 0

    # Average heading (circular mean)
    avg_heading = None
    if heading_readings:
        sin_sum = sum(math.sin(math.radians(h)) for h in heading_readings)
        cos_sum = sum(math.cos(math.radians(h)) for h in heading_readings)
        avg_heading = round(math.degrees(math.atan2(sin_sum, cos_sum)) % 360, 1)

    return avg_lidar, avg_heading


def calibrate_zone(zone_name, description=""):
    print(f"\nCalibrating Zone {zone_name.upper()} — environment: {ENVIRONMENT}")
    print("Place car at the zone position facing the desired heading.")
    print("Starting in 3 seconds...\n")
    time.sleep(3)

    status = api("/api/status")
    if not status:
        print("ERROR: Cannot reach Pi agent.")
        return False

    compass_ok = status.get("compass_ok", False)
    if not compass_ok:
        print("WARNING: Compass not available or not calibrated.")
        print("Zone will be stored with car-relative coordinates only.")
        print("Re-calibrate after compass is installed for best results.\n")

    battery = status.get("battery_v", 0)
    print(f"Battery: {battery}V  Compass: {'OK' if compass_ok else 'NOT AVAILABLE'}\n")

    avg_lidar, avg_heading = take_readings()

    # Convert to world coordinates if compass available
    world_distances = None
    if avg_heading is not None:
        world_distances = lidar_to_world(avg_lidar, avg_heading)
        print(f"  Heading:  {avg_heading}°")
        print(f"  World:    N={world_distances['N']:.0f}mm  S={world_distances['S']:.0f}mm  "
              f"E={world_distances['E']:.0f}mm  W={world_distances['W']:.0f}mm")
    else:
        print(f"  Heading:  unknown (no compass)")

    print(f"  LiDAR:    front={avg_lidar['front_mm']:.0f}mm  back={avg_lidar['back_mm']:.0f}mm  "
          f"left={avg_lidar['left_mm']:.0f}mm  right={avg_lidar['right_mm']:.0f}mm")

    zone_data = {
        "name":             zone_name.upper(),
        "description":      description or f"Zone {zone_name.upper()}",
        "environment":      ENVIRONMENT,
        "calibrated":       datetime.now().isoformat(),
        "tolerance_mm":     TOLERANCE_MM,
        "heading_deg":      avg_heading,
        "lidar":            avg_lidar,
        "world_distances":  world_distances,
        "compass_used":     compass_ok,
    }

    data = load_zones()
    data["zones"][zone_name.upper()] = zone_data
    save_zones(data)

    # KB log
    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(
        LOG_DIR,
        f"calibration_{ENVIRONMENT}_{zone_name.upper()}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )
    log = {
        "type":        "zone_calibration",
        "environment": ENVIRONMENT,
        "zone":        zone_name.upper(),
        "timestamp":   datetime.now().isoformat(),
        "summary":     f"Zone {zone_name.upper()} calibrated in {ENVIRONMENT}. "
                       f"Heading: {avg_heading}°. "
                       f"World: N={world_distances['N'] if world_distances else '?'}mm "
                       f"S={world_distances['S'] if world_distances else '?'}mm "
                       f"E={world_distances['E'] if world_distances else '?'}mm "
                       f"W={world_distances['W'] if world_distances else '?'}mm. "
                       f"Compass used: {compass_ok}.",
        "data": zone_data,
    }
    with open(log_path, "w") as f:
        json.dump(log, f, indent=2)

    print(f"\nZone {zone_name.upper()} saved to: {ZONES_FILE}")
    print(f"Log: {log_path}")
    return True


def list_zones():
    data  = load_zones()
    zones = data.get("zones", {})
    if not zones:
        print(f"No zones defined for environment: {ENVIRONMENT}")
        return
    print(f"\nZones for environment: {ENVIRONMENT}")
    print("=" * 65)
    for name, zone in zones.items():
        world = zone.get("world_distances", {})
        lidar = zone.get("lidar", {})
        heading = zone.get("heading_deg", "unknown")
        compass = "✓" if zone.get("compass_used") else "✗"
        print(f"\n  Zone {name} — {zone.get('description', '')}  [compass: {compass}]")
        print(f"    Calibrated: {zone.get('calibrated', 'unknown')}")
        print(f"    Heading:    {heading}°")
        if world:
            print(f"    World:      N={world.get('N',0):.0f}  S={world.get('S',0):.0f}  "
                  f"E={world.get('E',0):.0f}  W={world.get('W',0):.0f}  (mm)")
        print(f"    LiDAR:      F={lidar.get('front_mm',0):.0f}  B={lidar.get('back_mm',0):.0f}  "
              f"L={lidar.get('left_mm',0):.0f}  R={lidar.get('right_mm',0):.0f}  (mm)")
        print(f"    Tolerance:  ±{zone.get('tolerance_mm', TOLERANCE_MM)}mm")


def delete_zone(zone_name):
    data = load_zones()
    if zone_name.upper() in data["zones"]:
        del data["zones"][zone_name.upper()]
        save_zones(data)
        print(f"Zone {zone_name.upper()} deleted.")
    else:
        print(f"Zone {zone_name.upper()} not found.")


def main():
    parser = argparse.ArgumentParser(description="PiCar-X Zone Calibration Tool v2")
    parser.add_argument("--zone",        help="Zone name to calibrate (A, B, C)")
    parser.add_argument("--description", help="Zone description", default="")
    parser.add_argument("--list",        action="store_true")
    parser.add_argument("--delete",      help="Delete a zone")
    parser.add_argument("--env",         help="Environment name", default=None)
    args = parser.parse_args()

    global ENVIRONMENT, ZONES_FILE
    if args.env:
        ENVIRONMENT = args.env
        ZONES_FILE  = os.path.join(DATA_DIR, f"zones_{ENVIRONMENT}.json")

    if args.list:
        list_zones()
    elif args.delete:
        delete_zone(args.delete)
    elif args.zone:
        calibrate_zone(args.zone, args.description)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()

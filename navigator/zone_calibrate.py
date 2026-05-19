#!/usr/bin/env python3
"""
zone_calibrate.py — PiCar-X Zone Calibration Tool

Run on Fedora to define named zones in the current environment.
Places the car at a zone, takes averaged LiDAR readings, stores to zones.json.

Usage:
  python3 zone_calibrate.py --zone A
  python3 zone_calibrate.py --zone B
  python3 zone_calibrate.py --list
  python3 zone_calibrate.py --delete A

Zones are stored in:
  /mnt/ai-lab/picar-x-ai/data/zones_<environment>.json

Log (KB-ingestible) stored in:
  /mnt/ai-lab/picar-x-ai/logs/calibration_<environment>_<timestamp>.json
"""

import requests
import time
import json
import sys
import os
import argparse
from datetime import datetime
from statistics import mean, stdev

# ── Load config ────────────────────────────────────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from config import PI_IP, AGENT_PORT
except ImportError:
    PI_IP      = "YOUR_PI_IP"
    AGENT_PORT = 8000

AGENT_URL = f"http://{PI_IP}:{AGENT_PORT}"

# ── Config ─────────────────────────────────────────────────────────────────────
ENVIRONMENT   = os.environ.get("PICAR_ENV", "garage")
DATA_DIR      = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
LOG_DIR       = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
ZONES_FILE    = os.path.join(DATA_DIR, f"zones_{ENVIRONMENT}.json")
SAMPLES       = 20          # readings to average
SAMPLE_HZ     = 5           # readings per second
TOLERANCE_MM  = 450         # zone match tolerance in mm


def api(endpoint, method="get", params=None):
    try:
        url = f"{AGENT_URL}{endpoint}"
        r = requests.get(url, timeout=2.0) if method == "get" else requests.post(url, params=params, timeout=2.0)
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


def take_readings():
    """Take SAMPLES LiDAR readings and return averaged values."""
    print(f"  Taking {SAMPLES} readings at {SAMPLE_HZ}Hz...")
    readings = {"front": [], "back": [], "left": [], "right": [], "grayscale": []}

    for i in range(SAMPLES):
        sensors = api("/api/sensors")
        if sensors:
            lidar = sensors.get("lidar", {})
            for k in ["front", "back", "left", "right"]:
                v = lidar.get(k, 0)
                if v > 0:
                    readings[k].append(v)
            gs = sensors.get("grayscale", [0,0,0])
            readings["grayscale"].append(gs)

        print(f"    [{i+1:2d}/{SAMPLES}]", end="\r")
        time.sleep(1 / SAMPLE_HZ)

    print()
    averaged = {}
    for k in ["front", "back", "left", "right"]:
        vals = readings[k]
        if vals:
            averaged[k] = {
                "mean_mm":   round(mean(vals), 1),
                "stdev_mm":  round(stdev(vals), 1) if len(vals) > 1 else 0,
                "samples":   len(vals),
            }
        else:
            averaged[k] = {"mean_mm": 0, "stdev_mm": 0, "samples": 0}

    # Average grayscale
    gs_readings = readings["grayscale"]
    if gs_readings:
        averaged["grayscale"] = {
            "left":   round(mean(r[0] for r in gs_readings if len(r) > 0), 1),
            "center": round(mean(r[1] for r in gs_readings if len(r) > 1), 1),
            "right":  round(mean(r[2] for r in gs_readings if len(r) > 2), 1),
        }

    return averaged


def calibrate_zone(zone_name, description=""):
    """Calibrate a zone at the car's current position."""
    print(f"\nCalibrating Zone {zone_name.upper()} — environment: {ENVIRONMENT}")
    print("Place car at the zone position and hold still.")
    print("Starting in 3 seconds...\n")
    time.sleep(3)

    # Verify agent
    status = api("/api/status")
    if not status:
        print("ERROR: Cannot reach Pi agent.")
        return False

    battery = status.get("battery_v", 0)
    print(f"Battery: {battery}V ({status.get('battery_pct', 0)}%)")

    readings = take_readings()

    zone_data = {
        "name":        zone_name.upper(),
        "description": description or f"Zone {zone_name.upper()}",
        "environment": ENVIRONMENT,
        "calibrated":  datetime.now().isoformat(),
        "tolerance_mm": TOLERANCE_MM,
        "lidar": {
            "front_mm":  readings["front"]["mean_mm"],
            "back_mm":   readings["back"]["mean_mm"],
            "left_mm":   readings["left"]["mean_mm"],
            "right_mm":  readings["right"]["mean_mm"],
        },
        "lidar_stdev": {
            "front_mm":  readings["front"]["stdev_mm"],
            "back_mm":   readings["back"]["stdev_mm"],
            "left_mm":   readings["left"]["stdev_mm"],
            "right_mm":  readings["right"]["stdev_mm"],
        },
        "grayscale":   readings.get("grayscale", {}),
        "battery_v":   battery,
    }

    # Save zone
    data = load_zones()
    data["zones"][zone_name.upper()] = zone_data
    save_zones(data)

    # Write KB-ingestible log
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
        "summary":     f"Zone {zone_name.upper()} calibrated in {ENVIRONMENT} environment. "
                       f"LiDAR: front={readings['front']['mean_mm']:.0f}mm, "
                       f"back={readings['back']['mean_mm']:.0f}mm, "
                       f"left={readings['left']['mean_mm']:.0f}mm, "
                       f"right={readings['right']['mean_mm']:.0f}mm. "
                       f"Tolerance: ±{TOLERANCE_MM}mm.",
        "data":        zone_data,
    }
    with open(log_path, "w") as f:
        json.dump(log, f, indent=2)

    print(f"\nZone {zone_name.upper()} calibrated:")
    print(f"  Front:  {readings['front']['mean_mm']:.0f}mm  (±{readings['front']['stdev_mm']:.0f}mm)")
    print(f"  Back:   {readings['back']['mean_mm']:.0f}mm  (±{readings['back']['stdev_mm']:.0f}mm)")
    print(f"  Left:   {readings['left']['mean_mm']:.0f}mm  (±{readings['left']['stdev_mm']:.0f}mm)")
    print(f"  Right:  {readings['right']['mean_mm']:.0f}mm  (±{readings['right']['stdev_mm']:.0f}mm)")
    print(f"\nSaved to: {ZONES_FILE}")
    print(f"Log:      {log_path}")
    return True


def list_zones():
    data = load_zones()
    zones = data.get("zones", {})
    if not zones:
        print(f"No zones defined for environment: {ENVIRONMENT}")
        return
    print(f"\nZones for environment: {ENVIRONMENT}")
    print("=" * 60)
    for name, zone in zones.items():
        lidar = zone.get("lidar", {})
        print(f"\n  Zone {name} — {zone.get('description', '')}")
        print(f"    Calibrated: {zone.get('calibrated', 'unknown')}")
        print(f"    Front:  {lidar.get('front_mm', 0):.0f}mm")
        print(f"    Back:   {lidar.get('back_mm', 0):.0f}mm")
        print(f"    Left:   {lidar.get('left_mm', 0):.0f}mm")
        print(f"    Right:  {lidar.get('right_mm', 0):.0f}mm")
        print(f"    Tolerance: ±{zone.get('tolerance_mm', TOLERANCE_MM)}mm")


def delete_zone(zone_name):
    data = load_zones()
    if zone_name.upper() in data["zones"]:
        del data["zones"][zone_name.upper()]
        save_zones(data)
        print(f"Zone {zone_name.upper()} deleted.")
    else:
        print(f"Zone {zone_name.upper()} not found.")


def main():
    parser = argparse.ArgumentParser(description="PiCar-X Zone Calibration Tool")
    parser.add_argument("--zone",        help="Zone name to calibrate (A, B, C, etc.)")
    parser.add_argument("--description", help="Zone description", default="")
    parser.add_argument("--list",        action="store_true", help="List all zones")
    parser.add_argument("--delete",      help="Delete a zone")
    parser.add_argument("--env",         help="Environment name (default: garage)", default=None)
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

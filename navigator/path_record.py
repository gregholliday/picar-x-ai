#!/usr/bin/env python3
"""
path_record.py — PiCar-X Path Teaching Recorder

Run on Fedora while manually driving the car via the keyboard dashboard.
Records LiDAR snapshots at regular intervals to build a path map.

Usage:
  python3 path_record.py --path A_to_B
  python3 path_record.py --path A_to_B --env garage
  python3 path_record.py --list

Paths stored in:
  /mnt/ai-lab/picar-x-ai/data/paths_<environment>.json

Log (KB-ingestible) stored in:
  /mnt/ai-lab/picar-x-ai/logs/path_record_<path>_<timestamp>.json
"""

import requests
import time
import json
import sys
import os
import argparse
import signal
from datetime import datetime
from statistics import mean

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from config import PI_IP, AGENT_PORT
except ImportError:
    PI_IP      = "YOUR_PI_IP"
    AGENT_PORT = 8000

AGENT_URL = f"http://{PI_IP}:{AGENT_PORT}"

ENVIRONMENT  = os.environ.get("PICAR_ENV", "garage")
DATA_DIR     = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
LOG_DIR      = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
PATHS_FILE   = os.path.join(DATA_DIR, f"paths_{ENVIRONMENT}.json")
RECORD_HZ    = 2    # snapshots per second while driving
recording    = True


def api(endpoint):
    try:
        r = requests.get(f"{AGENT_URL}{endpoint}", timeout=2.0)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        print(f"  [API ERROR] {endpoint}: {e}")
        return None


def load_paths():
    if os.path.exists(PATHS_FILE):
        with open(PATHS_FILE) as f:
            return json.load(f)
    return {"environment": ENVIRONMENT, "paths": {}, "created": datetime.now().isoformat()}


def save_paths(data):
    os.makedirs(DATA_DIR, exist_ok=True)
    data["updated"] = datetime.now().isoformat()
    with open(PATHS_FILE, "w") as f:
        json.dump(data, f, indent=2)


def record_path(path_name):
    global recording

    print(f"\nPath Recorder — {path_name} — environment: {ENVIRONMENT}")
    print("Drive the car manually using your keyboard dashboard.")
    print("Recording starts in 3 seconds. Press Ctrl+C to stop.\n")
    time.sleep(3)

    status = api("/api/status")
    if not status:
        print("ERROR: Cannot reach Pi agent.")
        return False

    print("Recording... drive the car now.\n")

    snapshots    = []
    elapsed      = 0.0
    interval     = 1.0 / RECORD_HZ
    start_time   = time.time()

    def handle_stop(sig, frame):
        global recording
        recording = False
        print("\n\nStopping recording...")

    signal.signal(signal.SIGINT, handle_stop)

    while recording:
        loop_start = time.time()
        sensors    = api("/api/sensors")

        if sensors:
            lidar  = sensors.get("lidar", {})
            gs     = sensors.get("grayscale", [0,0,0])
            elapsed = round(time.time() - start_time, 2)

            snapshot = {
                "t":          elapsed,
                "lidar": {
                    "front_mm": lidar.get("front", 0),
                    "back_mm":  lidar.get("back",  0),
                    "left_mm":  lidar.get("left",  0),
                    "right_mm": lidar.get("right", 0),
                    "points":   lidar.get("points", 0),
                },
                "grayscale": {
                    "left":   gs[0] if len(gs) > 0 else 0,
                    "center": gs[1] if len(gs) > 1 else 0,
                    "right":  gs[2] if len(gs) > 2 else 0,
                },
                "speed":  sensors.get("speed", 0),
                "angle":  sensors.get("angle", 0),
            }
            snapshots.append(snapshot)
            print(f"  t={elapsed:6.1f}s  front={lidar.get('front',0):.0f}mm  "
                  f"left={lidar.get('left',0):.0f}mm  right={lidar.get('right',0):.0f}mm  "
                  f"[{len(snapshots)} pts]", end="\r")

        elapsed_loop = time.time() - loop_start
        time.sleep(max(0, interval - elapsed_loop))

    # Build path record
    path_data = {
        "name":        path_name,
        "environment": ENVIRONMENT,
        "recorded":    datetime.now().isoformat(),
        "duration_s":  elapsed,
        "points":      len(snapshots),
        "record_hz":   RECORD_HZ,
        "snapshots":   snapshots,
    }

    # Compute bounding box from LiDAR readings for summary
    fronts  = [s["lidar"]["front_mm"] for s in snapshots if s["lidar"]["front_mm"] > 0]
    lefts   = [s["lidar"]["left_mm"]  for s in snapshots if s["lidar"]["left_mm"]  > 0]
    rights  = [s["lidar"]["right_mm"] for s in snapshots if s["lidar"]["right_mm"] > 0]
    backs   = [s["lidar"]["back_mm"]  for s in snapshots if s["lidar"]["back_mm"]  > 0]

    summary_text = (
        f"Path '{path_name}' recorded in {ENVIRONMENT} environment. "
        f"Duration: {elapsed:.1f}s, {len(snapshots)} data points at {RECORD_HZ}Hz. "
        f"LiDAR ranges — front: {min(fronts):.0f}-{max(fronts):.0f}mm, "
        f"left: {min(lefts):.0f}-{max(lefts):.0f}mm, "
        f"right: {min(rights):.0f}-{max(rights):.0f}mm."
        if fronts and lefts and rights else
        f"Path '{path_name}' recorded. Duration: {elapsed:.1f}s, {len(snapshots)} points."
    )

    # Save path
    data = load_paths()
    data["paths"][path_name] = path_data
    save_paths(data)

    # KB-ingestible log (summary only, not full snapshots)
    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(
        LOG_DIR,
        f"path_record_{path_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )
    log = {
        "type":        "path_recording",
        "environment": ENVIRONMENT,
        "path":        path_name,
        "timestamp":   datetime.now().isoformat(),
        "summary":     summary_text,
        "metadata": {
            "duration_s":   elapsed,
            "point_count":  len(snapshots),
            "record_hz":    RECORD_HZ,
            "lidar_ranges": {
                "front_min_mm": min(fronts) if fronts else 0,
                "front_max_mm": max(fronts) if fronts else 0,
                "left_min_mm":  min(lefts)  if lefts  else 0,
                "left_max_mm":  max(lefts)  if lefts  else 0,
                "right_min_mm": min(rights) if rights else 0,
                "right_max_mm": max(rights) if rights else 0,
            }
        }
    }
    with open(log_path, "w") as f:
        json.dump(log, f, indent=2)

    print(f"\n\nPath '{path_name}' recorded:")
    print(f"  Duration:    {elapsed:.1f}s")
    print(f"  Data points: {len(snapshots)}")
    print(f"\nSaved to: {PATHS_FILE}")
    print(f"Log:      {log_path}")
    return True


def list_paths():
    data = load_paths()
    paths = data.get("paths", {})
    if not paths:
        print(f"No paths recorded for environment: {ENVIRONMENT}")
        return
    print(f"\nPaths for environment: {ENVIRONMENT}")
    print("=" * 60)
    for name, path in paths.items():
        print(f"\n  {name}")
        print(f"    Recorded:    {path.get('recorded', 'unknown')}")
        print(f"    Duration:    {path.get('duration_s', 0):.1f}s")
        print(f"    Data points: {path.get('points', 0)}")


def main():
    parser = argparse.ArgumentParser(description="PiCar-X Path Teaching Recorder")
    parser.add_argument("--path",   help="Path name (e.g. A_to_B)")
    parser.add_argument("--list",   action="store_true", help="List recorded paths")
    parser.add_argument("--delete", help="Delete a recorded path")
    parser.add_argument("--env",    help="Environment name", default=None)
    args = parser.parse_args()

    global ENVIRONMENT, PATHS_FILE
    if args.env:
        ENVIRONMENT = args.env
        PATHS_FILE  = os.path.join(DATA_DIR, f"paths_{ENVIRONMENT}.json")

    if args.list:
        list_paths()
    elif args.delete:
        data = load_paths()
        if args.delete in data["paths"]:
            del data["paths"][args.delete]
            save_paths(data)
            print(f"Path '{args.delete}' deleted.")
        else:
            print(f"Path '{args.delete}' not found.")
    elif args.path:
        record_path(args.path)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()

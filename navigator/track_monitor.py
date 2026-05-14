#!/usr/bin/env python3
"""
track_monitor.py — PiCar-X Track Session Monitor v1

The heavy lifting (boundary detection, steering corrections, corner turns)
now runs on the Pi in picar_agent_v8.py as a local reflex loop.

This script on Fedora just:
  1. Starts the track session via /api/track/start
  2. Polls /api/sensors at 5Hz for logging
  3. Watches for off-surface events and resumes after manual replacement
  4. Stops the session after SESSION_DURATION
  5. Writes a detailed JSONL log for post-session analysis

Usage:
  python3 track_monitor.py
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
SESSION_DURATION = 45      # seconds
POLL_RATE_HZ     = 5        # logging rate — lower since Pi handles reflexes now

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
            r = requests.get(url, params=params, timeout=2.0)
        else:
            r = requests.post(url, params=params, timeout=2.0)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        print(f"  [API ERROR] {endpoint}: {e}")
        return None


# ── Logging ────────────────────────────────────────────────────────────────────
def build_log_entry(sensors):
    gs    = sensors.get("grayscale", [0, 0, 0])
    lidar = sensors.get("lidar", {})
    return {
        "ts":                  datetime.now().isoformat(timespec="milliseconds"),
        "track_event":         sensors.get("track_event", "CLEAR"),
        "track_reflex_active": sensors.get("track_reflex_active", False),
        "track_off_surface":   sensors.get("track_off_surface", False),
        "grayscale": {
            "right":  gs[0] if len(gs) > 0 else 0,
            "center": gs[1] if len(gs) > 1 else 0,
            "left":   gs[2] if len(gs) > 2 else 0,
        },
        "lidar": {
            "points":   lidar.get("points", 0),
            "front_mm": lidar.get("front", 0),
            "left_mm":  lidar.get("left", 0),
            "right_mm": lidar.get("right", 0),
            "back_mm":  lidar.get("back", 0),
        },
        "ultrasonic_cm":  sensors.get("ultrasonic_cm", 0),
        "reflex_active":  sensors.get("reflex_active", False),
    }

def write_log(f, entry):
    f.write(json.dumps(entry) + "\n")
    f.flush()

def write_summary(entries, duration):
    from collections import Counter
    events = Counter(e.get("track_event") for e in entries if "track_event" in e)
    off_surface = sum(1 for e in entries if e.get("track_off_surface"))
    corners     = events.get("CORNER", 0)

    return {
        "type":              "session_summary",
        "session_duration":  duration,
        "total_entries":     len(entries),
        "event_counts":      dict(events),
        "off_surface_count": off_surface,
        "corner_detections": corners,
        "estimated_laps":    round(corners / 4, 1) if corners >= 4 else 0,
    }


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    print("=" * 60)
    print("  PiCar-X Track Session Monitor v1")
    print(f"  Agent:    {AGENT_URL}")
    print(f"  Duration: {SESSION_DURATION}s")
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
    print(f"  Agent version: picar_agent_v8")

    # Check estop
    estop = api("/api/estop")
    if estop and estop.get("estop_active"):
        print("ERROR: ESTOP is active. Clear it before running.")
        sys.exit(1)

    os.makedirs(LOG_DIR, exist_ok=True)

    print("\nStarting track session in 3 seconds...")
    print("Place car on track facing clockwise.")
    print("Press Ctrl+C at any time to stop safely.\n")
    time.sleep(3)

    # Start track mode on Pi — car starts moving immediately
    result = api("/api/track/start", method="post")
    if result is None:
        print("ERROR: Failed to start track mode.")
        sys.exit(1)
    print(f"Track started at speed {result.get('speed')}.\n")

    log_entries   = []
    session_start = time.time()
    loop_interval = 1.0 / POLL_RATE_HZ
    off_surface_warned = False

    try:
        with open(LOG_FILE, "w") as log_f:

            write_log(log_f, {
                "type":       "session_header",
                "start_time": datetime.now().isoformat(),
                "config": {
                    "duration":    SESSION_DURATION,
                    "poll_hz":     POLL_RATE_HZ,
                }
            })

            while True:
                loop_start = time.time()
                elapsed    = loop_start - session_start

                if elapsed >= SESSION_DURATION:
                    print(f"\nSession complete ({SESSION_DURATION}s).")
                    break

                remaining = int(SESSION_DURATION - elapsed)

                # Poll sensors
                sensors = api("/api/sensors")
                if sensors is None:
                    print(f"  [{remaining:3d}s] sensor error")
                    time.sleep(loop_interval)
                    continue

                entry = build_log_entry(sensors)
                log_entries.append(entry)
                write_log(log_f, entry)

                event       = sensors.get("track_event", "CLEAR")
                off_surface = sensors.get("track_off_surface", False)
                gs          = sensors.get("grayscale", [0,0,0])

                # Off-surface handling
                if off_surface and not off_surface_warned:
                    print(f"\n  [{remaining:3d}s] OFF SURFACE — place car back on track")
                    print("  Car is stopped. Watching for replacement...")
                    off_surface_warned = True

                elif off_surface_warned and not off_surface:
                    # Car back on surface — Pi will auto-resume but
                    # we can also trigger resume explicitly
                    print(f"  [{remaining:3d}s] Back on surface — resuming...")
                    api("/api/track/resume", method="post")
                    off_surface_warned = False

                else:
                    print(
                        f"  [{remaining:3d}s] "
                        f"event={event:<12} "
                        f"gs=[R:{gs[0]:4d} C:{gs[1]:4d} L:{gs[2]:4d}]",
                        end="\r"
                    )

                elapsed_loop = time.time() - loop_start
                time.sleep(max(0, loop_interval - elapsed_loop))

            # Session end
            api("/api/track/stop", method="post")

            summary = write_summary(log_entries, SESSION_DURATION)
            write_log(log_f, summary)

            print("\n" + "=" * 60)
            print("  SESSION SUMMARY")
            print("=" * 60)
            print(f"  Duration:          {SESSION_DURATION}s")
            print(f"  Total entries:     {summary['total_entries']}")
            print(f"  Corner detections: {summary['corner_detections']}")
            print(f"  Estimated laps:    {summary['estimated_laps']}")
            print(f"  Off surface:       {summary['off_surface_count']}")
            print(f"  Event counts:      {summary['event_counts']}")
            print(f"  Log saved:         {LOG_FILE}")
            print("=" * 60)

    except KeyboardInterrupt:
        print("\n\nInterrupted — stopping safely...")
        api("/api/track/stop", method="post")
        print("Car stopped. Log saved to:", LOG_FILE)


if __name__ == "__main__":
    if PI_IP == "YOUR_PI_IP":
        print("ERROR: config.py not found at repo root.")
        sys.exit(1)
    main()

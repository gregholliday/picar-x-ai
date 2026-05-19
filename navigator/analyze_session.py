#!/usr/bin/env python3
"""
analyze_session.py — PiCar-X Session Log Analyzer (Part B) v2

Takes a session log file, sends structured analysis to Ollama,
produces two outputs:
  1. diagnostic_<session>.md  — technical analysis for the engineer
  2. narrative_<session>.md   — Medium-ready experiment summary

Improvements over v1:
  - Rich statistical summary instead of raw entry dump
  - Samples key moments: start, tape stops, wall turns, end
  - LiDAR distance trends and ranges
  - Grayscale analysis during events
  - Better prompts with full system context

Usage:
  python3 analyze_session.py --latest --model gemma3-homelab:12b
  python3 analyze_session.py --log logs/roam_garage_20260519_092237.json
  python3 analyze_session.py --latest --diag-only
  python3 analyze_session.py --latest --narr-only
"""

import json
import os
import sys
import argparse
import requests
from datetime import datetime
from pathlib import Path
from collections import Counter
from statistics import mean, stdev

# ── Config ─────────────────────────────────────────────────────────────────────
BASE_DIR      = Path(__file__).parent.parent
LOG_DIR       = BASE_DIR / "logs"
REPORTS_DIR   = BASE_DIR / "reports"
OLLAMA_URL    = "http://localhost:11434/api/generate"
DEFAULT_MODEL = "gemma3-homelab:12b"

# System context — injected into every prompt
SYSTEM_CONTEXT = """
You are analyzing experiment logs from a PiCar-X autonomous robot car project.

HARDWARE:
- PiCar-X robot car running on Raspberry Pi 4B
- RPLiDAR C1 — 360 degree laser scanner, mounted on top of car, scans horizontally
  at car height (does NOT see floor tape or obstacles below car height)
- Grayscale sensor array — 3 sensors (left, center, right) mounted at front bottom,
  reads floor reflectance. Higher values = more reflective surface.
- Ultrasonic sensor — forward-facing, short range obstacle detection
- Camera — not used in navigation sessions

ARCHITECTURE:
- Pi agent (picar_agent.py) runs on Pi, exposes REST API
- Navigator scripts run on Fedora, poll Pi at 10Hz, send drive commands
- LiDAR readings: front/back/left/right in mm (0 = no reading)
- Grayscale: gs[0]=left sensor, gs[1]=center, gs[2]=right sensor
- Grayscale thresholds: masking tape reads ~950-1450 (HIGH), garage floor reads ~300-800 (LOW)

CURRENT NAVIGATION APPROACH (roam_navigator.py):
- LiDAR handles wall avoidance on 3 closed sides of garage
- Grayscale tape barriers at open garage doors = emergency stop
- WALL_STOP_MM=400, WALL_SLOW_MM=800, WALL_STEER_MM=600
- TAPE_STOP: car stops, backs up ~0.8s, turns toward open space, resumes
- FORWARD = clear driving, WALL_TURN = avoiding wall, WALL_STEER = gentle correction
- False tape stops can occur from reflective floor spots — car recovers automatically

ENVIRONMENT:
- Two-car garage, ~24ft wide x 22ft deep
- Three solid walls (LiDAR sees them clearly)
- One open end (both garage doors open) — tape barriers here
- Zones A/B/C defined by LiDAR distance signatures from walls
"""


def call_ollama(prompt, model):
    print(f"  Calling {model}...", end=" ", flush=True)
    try:
        r = requests.post(
            OLLAMA_URL,
            json={"model": model, "prompt": prompt, "stream": False},
            timeout=360,
        )
        r.raise_for_status()
        result = r.json()["response"]
        print("done.")
        return result
    except Exception as e:
        print(f"ERROR: {e}")
        return None


def load_log(log_path):
    with open(log_path) as f:
        return json.load(f)


def get_latest_log():
    logs = sorted(LOG_DIR.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not logs:
        print(f"No log files found in {LOG_DIR}")
        sys.exit(1)
    return logs[0]


def extract_rich_summary(log):
    """
    Build a rich statistical summary of the session instead of dumping raw entries.
    Includes: action distribution, LiDAR trends, key events with context.
    """
    log_type = log.get("type", "unknown")
    entries  = log.get("entries", [])

    # For mission logs, flatten entries
    if log_type == "mission":
        all_entries = []
        for leg in log.get("legs", []):
            all_entries.extend(leg.get("entries", []))
        entries = all_entries

    if not entries:
        return {}

    # ── Action distribution ────────────────────────────────────────────────────
    action_counts = Counter(e.get("action", "UNKNOWN") for e in entries)
    total         = len(entries)

    # ── LiDAR statistics ───────────────────────────────────────────────────────
    def lidar_stats(axis):
        vals = [e["lidar"][axis] for e in entries
                if "lidar" in e and e["lidar"].get(axis, 0) > 0]
        if not vals:
            return {"min": 0, "max": 0, "mean": 0, "samples": 0}
        return {
            "min":     round(min(vals)),
            "max":     round(max(vals)),
            "mean":    round(mean(vals)),
            "samples": len(vals),
        }

    lidar_summary = {
        "front": lidar_stats("front_mm"),
        "back":  lidar_stats("back_mm"),
        "left":  lidar_stats("left_mm"),
        "right": lidar_stats("right_mm"),
    }

    # ── Grayscale statistics ───────────────────────────────────────────────────
    def gs_stats(sensor):
        vals = [e["grayscale"][sensor] for e in entries
                if "grayscale" in e and e["grayscale"].get(sensor, 0) > 50]
        if not vals:
            return {"min": 0, "max": 0, "mean": 0}
        return {"min": round(min(vals)), "max": round(max(vals)), "mean": round(mean(vals))}

    gs_summary = {
        "left":   gs_stats("left"),
        "center": gs_stats("center"),
        "right":  gs_stats("right"),
    }

    # ── Key events with context ────────────────────────────────────────────────
    def get_context(entries, idx, window=3):
        """Get entries surrounding an event."""
        start = max(0, idx - window)
        end   = min(len(entries), idx + window + 1)
        return entries[start:end]

    def format_entry(e):
        lidar = e.get("lidar", {})
        gs    = e.get("grayscale", {})
        return {
            "t":      round(e.get("elapsed", 0), 1),
            "action": e.get("action", ""),
            "reason": e.get("reason", ""),
            "front":  lidar.get("front_mm", 0),
            "left":   lidar.get("left_mm", 0),
            "right":  lidar.get("right_mm", 0),
            "gs_l":   gs.get("left", 0),
            "gs_c":   gs.get("center", 0),
            "gs_r":   gs.get("right", 0),
            "us_cm":  e.get("ultrasonic_cm", 0),
        }

    # Tape stop events
    tape_events = []
    for i, e in enumerate(entries):
        if e.get("action") == "TAPE_STOP":
            context = [format_entry(x) for x in get_context(entries, i)]
            tape_events.append({"index": i, "context": context})

    # Wall turn events (sample up to 5)
    wall_events = []
    for i, e in enumerate(entries):
        if e.get("action") in ("WALL_TURN", "WALL_AVOID_LEFT", "WALL_AVOID_RIGHT"):
            context = [format_entry(x) for x in get_context(entries, i, window=2)]
            wall_events.append({"index": i, "context": context})
            if len(wall_events) >= 5:
                break

    # Start and end samples
    start_sample = [format_entry(e) for e in entries[:5]]
    end_sample   = [format_entry(e) for e in entries[-5:]]

    # ── Time analysis ──────────────────────────────────────────────────────────
    total_time = log.get("duration_s", 0)
    if total_time and total and "FORWARD" in action_counts:
        forward_pct = round(100 * action_counts["FORWARD"] / total)
    else:
        forward_pct = 0

    # Gap between tape stops
    tape_times = [entries[e["index"]].get("elapsed", 0) for e in tape_events]
    tape_gaps  = [round(tape_times[i+1] - tape_times[i], 1)
                  for i in range(len(tape_times)-1)] if len(tape_times) > 1 else []

    return {
        "log_type":      log_type,
        "environment":   log.get("environment", "unknown"),
        "timestamp":     log.get("timestamp", "unknown"),
        "duration_s":    total_time,
        "total_entries": total,
        "forward_pct":   forward_pct,
        "action_counts": dict(action_counts.most_common()),
        "lidar_ranges":  lidar_summary,
        "grayscale_ranges": gs_summary,
        "tape_stop_count":  action_counts.get("TAPE_STOP", 0),
        "tape_stop_times":  tape_times,
        "tape_stop_gaps":   tape_gaps,
        "tape_events":      tape_events,
        "wall_events":      wall_events,
        "start_sample":     start_sample,
        "end_sample":       end_sample,
        "session_summary":  log.get("summary", ""),
        "config":           log.get("metadata", {}).get("config", {}),
    }


def generate_diagnostic(rich, model):
    """Generate technical diagnostic with rich context."""

    prompt = f"""{SYSTEM_CONTEXT}

---

ANALYZE THIS SESSION:

Session type: {rich['log_type']}
Environment: {rich['environment']}
Timestamp: {rich['timestamp']}
Duration: {rich['duration_s']}s
Total decisions: {rich['total_entries']} at 10Hz

Session summary (auto-generated):
{rich['session_summary']}

Navigator config used:
{json.dumps(rich['config'], indent=2)}

ACTION DISTRIBUTION:
{json.dumps(rich['action_counts'], indent=2)}

Forward driving: {rich['forward_pct']}% of session

LIDAR DISTANCE RANGES (mm) across full session:
{json.dumps(rich['lidar_ranges'], indent=2)}

GRAYSCALE RANGES across full session:
{json.dumps(rich['grayscale_ranges'], indent=2)}
Note: Masking tape reads ~950-1450 (high). Garage floor reads ~300-800 (low).
Values above threshold = tape detected.

TAPE STOP EVENTS ({rich['tape_stop_count']} total):
Times elapsed: {rich['tape_stop_times']}
Gaps between stops (s): {rich['tape_stop_gaps']}
Event details with surrounding context:
{json.dumps(rich['tape_events'], indent=2)}

WALL TURN EVENTS (first 5):
{json.dumps(rich['wall_events'], indent=2)}

SESSION START (first 5 decisions):
{json.dumps(rich['start_sample'], indent=2)}

SESSION END (last 5 decisions):
{json.dumps(rich['end_sample'], indent=2)}

---

Write a technical diagnostic report in markdown. Be specific — use actual numbers from the data above.
Do NOT invent sensor readings or events not present in the data.
Do NOT explain what sensors do — the engineer already knows.

## Session Overview
What happened during this session in 2-3 sentences.

## What Worked Well
Specific behaviors with evidence. Reference actual values.

## Issues Found
Real problems only. For each tape stop, was it a real barrier or likely a false trigger?
Base this on the grayscale values at the time — if gs values are floor-level (~300-800), it was likely a false trigger.
If gs values are tape-level (~950+), it was a real barrier detection.

## Root Cause Analysis
For each real issue, the likely cause based on the data.

## Recommended Config Changes
Specific parameter changes with suggested values. Reference the config above.

## Sensor Health
One paragraph based on actual ranges seen."""

    return call_ollama(prompt, model)


def generate_narrative(rich, model):
    """Generate Medium narrative with rich context."""

    # Count real vs false tape stops based on grayscale values
    real_stops  = 0
    false_stops = 0
    for event in rich.get("tape_events", []):
        for entry in event.get("context", []):
            if entry.get("action") == "TAPE_STOP":
                gs_max = max(entry.get("gs_l", 0), entry.get("gs_c", 0), entry.get("gs_r", 0))
                if gs_max > 900:
                    real_stops += 1
                else:
                    false_stops += 1
                break

    prompt = f"""{SYSTEM_CONTEXT}

---

Write a Medium article entry about this experiment session.

SESSION FACTS (use these, do not invent):
- Session type: {rich['log_type']} in {rich['environment']}
- Duration: {rich['duration_s']} seconds
- The car drove forward {rich['forward_pct']}% of the time
- Wall avoidance triggered {rich['action_counts'].get('WALL_TURN', 0) + rich['action_counts'].get('WALL_STEER_LEFT', 0) + rich['action_counts'].get('WALL_STEER_RIGHT', 0)} times
- Tape stops: {rich['tape_stop_count']} total
  - Estimated real barrier detections: {real_stops}
  - Estimated false triggers (reflective floor spots): {false_stops}
- LiDAR front distance ranged from {rich['lidar_ranges']['front']['min']}mm to {rich['lidar_ranges']['front']['max']}mm
- Session summary: {rich['session_summary']}

CONTEXT FOR THE ARTICLE:
This is part of an ongoing series about building a home lab autonomous robot.
The author is a software developer/hobbyist, not a professional roboticist.
Previous sessions struggled with tape-based lane following on a test track.
This session is the FIRST successful free-roam test using LiDAR-based navigation.
The garage has 3 solid walls (LiDAR reference) and one open end (tape barriers).

Write in first person. Tone: honest, curious, like a lab notebook for a technically curious friend.
Length: 250-350 words. Flowing prose, no bullet points.
End with what you plan to try next (zone-to-zone navigation).
Do not use jargon without a brief explanation.
Do not invent any sensor readings or events."""

    return call_ollama(prompt, model)


def main():
    parser = argparse.ArgumentParser(description="PiCar-X Session Log Analyzer v2")
    parser.add_argument("--log",       help="Path to log file")
    parser.add_argument("--latest",    action="store_true", help="Analyze most recent log")
    parser.add_argument("--model",     default=DEFAULT_MODEL, help="Ollama model")
    parser.add_argument("--diag-only", action="store_true",  help="Diagnostic only")
    parser.add_argument("--narr-only", action="store_true",  help="Narrative only")
    args = parser.parse_args()

    if args.latest:
        log_path = get_latest_log()
    elif args.log:
        log_path = Path(args.log)
        if not log_path.exists():
            log_path = LOG_DIR / args.log
    else:
        parser.print_help()
        sys.exit(1)

    print(f"Analyzing: {log_path.name}")
    log  = load_log(log_path)
    rich = extract_rich_summary(log)

    print(f"  {rich['total_entries']} entries, {rich['duration_s']}s, "
          f"{rich['tape_stop_count']} tape stops, "
          f"{rich['forward_pct']}% forward")

    REPORTS_DIR.mkdir(parents=True, exist_ok=True)
    stem = log_path.stem

    if not args.narr_only:
        print("\nGenerating diagnostic...")
        diag = generate_diagnostic(rich, args.model)
        if diag:
            path = REPORTS_DIR / f"diagnostic_{stem}.md"
            path.write_text(
                f"# Diagnostic: {stem}\n\n"
                f"**Generated:** {datetime.now().isoformat()}\n"
                f"**Model:** {args.model}\n"
                f"**Session:** {rich['session_summary']}\n\n---\n\n" + diag
            )
            print(f"  Saved: {path}")

    if not args.diag_only:
        print("\nGenerating narrative...")
        narr = generate_narrative(rich, args.model)
        if narr:
            path = REPORTS_DIR / f"narrative_{stem}.md"
            path.write_text(
                f"# Narrative: {stem}\n\n"
                f"**Generated:** {datetime.now().isoformat()}\n"
                f"**Model:** {args.model}\n\n---\n\n" + narr
            )
            print(f"  Saved: {path}")

    print("\nDone.")


if __name__ == "__main__":
    main()

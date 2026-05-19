#!/usr/bin/env python3
"""
analyze_session.py — PiCar-X Session Log Analyzer (Part B)

Takes a session log file, sends it to Ollama for analysis,
produces two outputs:
  1. diagnostic_<session>.md  — technical analysis for the engineer
  2. narrative_<session>.md   — Medium-ready experiment summary

Usage:
  python3 analyze_session.py --log logs/roam_garage_20260519_120000.json
  python3 analyze_session.py --log logs/mission_garage_20260519_120000.json
  python3 analyze_session.py --latest
  python3 analyze_session.py --latest --model gemma3:12b
"""

import json
import os
import sys
import argparse
import requests
from datetime import datetime
from pathlib import Path

# ── Config ─────────────────────────────────────────────────────────────────────
BASE_DIR     = Path(__file__).parent.parent
LOG_DIR      = BASE_DIR / "logs"
REPORTS_DIR  = BASE_DIR / "reports"
OLLAMA_URL   = "http://localhost:11434/api/generate"
DEFAULT_MODEL = "gemma3:12b"
MAX_CHARS    = 16000   # max log content to send to Ollama


def call_ollama(prompt, model):
    """Call Ollama and return the response text."""
    print(f"  Calling {model}...", end=" ", flush=True)
    try:
        r = requests.post(
            OLLAMA_URL,
            json={"model": model, "prompt": prompt, "stream": False},
            timeout=300,
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
    """Find the most recent log file."""
    logs = sorted(LOG_DIR.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not logs:
        print(f"No log files found in {LOG_DIR}")
        sys.exit(1)
    return logs[0]


def summarize_log(log):
    """Extract key stats from log for prompt context."""
    log_type = log.get("type", "unknown")
    summary  = log.get("summary", "")
    meta     = log.get("metadata", {})
    entries  = log.get("entries", [])

    # For mission logs, flatten leg entries
    if log_type == "mission":
        for leg in log.get("legs", []):
            entries.extend(leg.get("entries", []))

    # Trim entries to fit context window
    entry_text = json.dumps(entries[:200], indent=2)
    if len(entry_text) > MAX_CHARS:
        entry_text = entry_text[:MAX_CHARS] + "\n... (truncated)"

    return {
        "type":        log_type,
        "summary":     summary,
        "metadata":    meta,
        "entry_sample": entry_text,
        "environment": log.get("environment", "unknown"),
        "timestamp":   log.get("timestamp", "unknown"),
        "duration_s":  log.get("duration_s", meta.get("duration_s", 0)),
    }


def generate_diagnostic(log_summary, model):
    """Generate technical diagnostic report."""
    prompt = f"""You are analyzing a PiCar-X autonomous robot experiment log.

Session type: {log_summary['type']}
Environment: {log_summary['environment']}
Timestamp: {log_summary['timestamp']}
Duration: {log_summary['duration_s']}s

Session summary:
{log_summary['summary']}

Metadata:
{json.dumps(log_summary['metadata'], indent=2)}

Sample of decision log entries:
{log_summary['entry_sample']}

Write a technical diagnostic report in markdown. Include:

## Session Overview
Brief description of what happened.

## What Worked Well
Specific behaviors that performed correctly with evidence from the log.

## Issues Identified
Specific problems observed, with timestamps and sensor values as evidence.

## Root Cause Analysis
For each issue, what likely caused it.

## Recommended Fixes
Concrete, actionable changes to config or code. Be specific.

## Sensor Health
Assessment of LiDAR, grayscale, and ultrasonic sensor reliability during this session.

Be technical and specific. Reference actual values from the log. Do not invent data."""

    return call_ollama(prompt, model)


def generate_narrative(log_summary, model):
    """Generate Medium-ready experiment narrative."""
    prompt = f"""You are helping write a Medium article series about building an autonomous robot car (PiCar-X).

Session type: {log_summary['type']}
Environment: {log_summary['environment']}
Timestamp: {log_summary['timestamp']}
Duration: {log_summary['duration_s']}s

Session summary:
{log_summary['summary']}

Key metadata:
{json.dumps(log_summary['metadata'], indent=2)}

Write an engaging experiment summary suitable for a Medium article. Write in first person as the experimenter.

The audience is technically curious but not necessarily robotics experts.

Include:
- What you were trying to accomplish in this session
- What actually happened (be honest about failures — they make better stories)
- What you learned
- What you'll try next

Tone: conversational, honest, curious. Like a lab notebook entry written for a technically curious friend.
Length: 200-300 words.
Format: flowing prose, no bullet points.

Do not invent technical details not supported by the summary. Do not use jargon without explanation."""

    return call_ollama(prompt, model)


def main():
    parser = argparse.ArgumentParser(description="PiCar-X Session Log Analyzer")
    parser.add_argument("--log",    help="Path to log file")
    parser.add_argument("--latest", action="store_true", help="Analyze most recent log")
    parser.add_argument("--model",  default=DEFAULT_MODEL, help="Ollama model to use")
    parser.add_argument("--diag-only", action="store_true", help="Only generate diagnostic")
    parser.add_argument("--narr-only", action="store_true", help="Only generate narrative")
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
    log         = load_log(log_path)
    log_summary = summarize_log(log)

    REPORTS_DIR.mkdir(parents=True, exist_ok=True)
    stem = log_path.stem

    if not args.narr_only:
        print("\nGenerating diagnostic report...")
        diagnostic = generate_diagnostic(log_summary, args.model)
        if diagnostic:
            diag_path = REPORTS_DIR / f"diagnostic_{stem}.md"
            header    = f"# Diagnostic Report: {stem}\n\n"
            header   += f"**Generated:** {datetime.now().isoformat()}\n"
            header   += f"**Model:** {args.model}\n"
            header   += f"**Session:** {log_summary['summary']}\n\n---\n\n"
            diag_path.write_text(header + diagnostic)
            print(f"Diagnostic saved: {diag_path}")

    if not args.diag_only:
        print("\nGenerating narrative...")
        narrative = generate_narrative(log_summary, args.model)
        if narrative:
            narr_path = REPORTS_DIR / f"narrative_{stem}.md"
            header    = f"# Experiment Narrative: {stem}\n\n"
            header   += f"**Generated:** {datetime.now().isoformat()}\n"
            header   += f"**Model:** {args.model}\n\n---\n\n"
            narr_path.write_text(header + narrative)
            print(f"Narrative saved: {narr_path}")

    print("\nDone.")


if __name__ == "__main__":
    main()

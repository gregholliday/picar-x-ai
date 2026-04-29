#!/usr/bin/env python3

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import requests


DEFAULT_PI = "http://192.168.1.225:8000"
DEFAULT_RUN_ROOT = "/mnt/ai-lab/picar-x-ai/experiments/runs"
DEFAULT_KB_ROOT = "/mnt/ai-lab/knowledge-base/wiki/picar/experiments"


def api_post(base_url, path, params=None, timeout=2):
    url = f"{base_url}{path}"
    r = requests.post(url, params=params or {}, timeout=timeout)
    r.raise_for_status()
    return r.json() if r.text else {}


def api_get(base_url, path, timeout=2):
    url = f"{base_url}{path}"
    r = requests.get(url, timeout=timeout)
    r.raise_for_status()
    return r.json()


def safe_estop(base_url):
    try:
        api_post(base_url, "/api/estop", timeout=2)
        print("E-stop activated.")
    except Exception as e:
        print(f"WARNING: failed to activate E-stop: {e}")


def set_manual(base_url):
    try:
        api_post(base_url, "/api/mode", params={"mode": "manual"}, timeout=2)
    except Exception as e:
        print(f"WARNING: failed to set manual mode: {e}")


def write_json(path, data):
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


def write_summary_md(path, data):
    md = f"""# PiCar Experiment {data["run_id"]}

## Summary

- Task: `{data["task"]}`
- Started: {data["started_at"]}
- Ended: {data["ended_at"]}
- Duration seconds: {data["duration_sec"]}
- Result: **{data["result"]}**
- Goal reached: {data["goal_reached"]}
- Timed out: {data["timed_out"]}
- E-stop used: {data["estop_used"]}

## Final Status

```json
{json.dumps(data.get("final_status", {}), indent=2)}
#!/usr/bin/env python3

import requests
import time
import subprocess
import os
import signal
import json
import select
from datetime import datetime

PI = "http://192.168.1.225:8000"
TASK = "Find the tea pot"
TIME_LIMIT = 180  # 5 minutes

run_id = datetime.now().strftime("%Y-%m-%d_%H%M%S")
run_dir = f"/mnt/ai-lab/picar-x-ai/experiments/{run_id}"
os.makedirs(run_dir, exist_ok=True)

log_path = f"{run_dir}/navigator.log"
result_path = f"{run_dir}/result.json"

goal_reached = False
timed_out = False
estop_used = False

def estop():
    global estop_used
    try:
        requests.post(f"{PI}/api/estop", timeout=2)
        estop_used = True
        print("🚨 E-STOP SENT")
    except Exception as e:
        print(f"Failed to send E-stop: {e}")

def cleanup():
    estop()
    print("Cleanup complete")

def signal_handler(sig, frame):
    print("\nInterrupted!")
    cleanup()
    exit(1)

signal.signal(signal.SIGINT, signal_handler)

print(f"Starting run {run_id}")
print(f"Task: {TASK}")

# Reset estop if needed
try:
    requests.post(f"{PI}/api/estop/reset", timeout=2)
except:
    pass

requests.post(f"{PI}/api/navigator/log/clear", timeout=2)

# Set task
requests.post(f"{PI}/api/task", params={"task": TASK})

# Set mode autonomous
requests.post(f"{PI}/api/mode/autonomous")

time.sleep(1)

status = requests.get(f"{PI}/api/status", timeout=2).json()
print(f"Mode after start: {status.get('mode')}")
print(f"E-stop active: {status.get('estop_active')}")

if status.get("estop_active"):
    raise RuntimeError("E-stop is still active after reset")

if status.get("mode") != "autonomous":
    raise RuntimeError(f"Failed to enter autonomous mode: {status.get('mode')}")

log_file = open(log_path, "w")

proc = subprocess.Popen(
    ["journalctl", "-u", "picar-navigator", "-f", "-n", "0", "-o", "cat"],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True
)

start_time = time.time()

try:
    while True:
        elapsed = time.time() - start_time

        if elapsed > TIME_LIMIT:
            timed_out = True
            print("⏱ Timeout reached")
            break

        ready, _, _ = select.select([proc.stdout], [], [], 0.5)

        if ready:
            line = proc.stdout.readline()

            if line:
                print(line, end="")
                log_file.write(line)
                log_file.flush()

                if "GOAL REACHED" in line:
                    goal_reached = True
                    print("🎯 Goal reached!")
                    break

        try:
            status = requests.get(f"{PI}/api/status", timeout=1).json()

            if status.get("estop_active"):
                print("🚨 E-stop is active")
                break

            if status.get("task_found") or status.get("task_status") == "GOAL_REACHED":
                goal_reached = True
                print("🎯 Goal reached by status")
                break

        except Exception:
            pass

except Exception as e:
    print(f"Error: {e}")

finally:
    proc.terminate()
    log_file.close()

    if not goal_reached:
        estop()

    try:
        status = requests.get(f"{PI}/api/status", timeout=2).json()
    except:
        status = {}

    result = {
        "run_id": run_id,
        "task": TASK,
        "goal_reached": goal_reached,
        "timed_out": timed_out,
        "estop_used": estop_used,
        "duration_sec": round(time.time() - start_time, 2),
        "final_status": status
    }

    with open(result_path, "w") as f:
        json.dump(result, f, indent=2)

    print("\nRun complete")
    print(json.dumps(result, indent=2))
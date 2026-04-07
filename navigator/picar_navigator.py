#!/usr/bin/env python3
"""
PiCar-X Navigator with Gemma 3 Vision and Goal-Directed Navigation
Runs on Fedora. Polls the Pi agent for sensor data and makes
autonomous driving decisions. Supports task-based navigation:
  "find the purple flag", "find the kitchen", etc.

Usage:
    python3 picar_navigator.py

    The PiCar must be in AUTONOMOUS mode via the dashboard
    before the navigator will send any drive commands.
    Set a task via the dashboard task input field.
"""

import requests
import time
import math
import json
import sys
import os
import base64
import threading
import urllib.request
from datetime import datetime
from collections import deque

# ── Load config ────────────────────────────────────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from config import (
        PI_IP, AGENT_PORT, CAMERA_PORT,
        BASE_SPEED, SLOW_SPEED, TURN_ANGLE,
        FRONT_CLEAR, FRONT_CAUTION, SIDE_CLEAR,
        OLLAMA_IP, OLLAMA_PORT, OLLAMA_MODEL,
    )
    AGENT_URL  = f"http://{PI_IP}:{AGENT_PORT}"
    CAMERA_URL = f"http://{PI_IP}:{CAMERA_PORT}/mjpg"
    OLLAMA_URL = f"http://{OLLAMA_IP}:{OLLAMA_PORT}/api/generate"
    print(f"Config loaded from config.py. Agent URL: {AGENT_URL}")
except ImportError:
    print("config.py not found — using defaults.")
    AGENT_URL     = "http://YOUR_PI_IP:8000"
    CAMERA_URL    = "http://YOUR_PI_IP:9000/mjpg"
    OLLAMA_URL    = "http://YOUR_OLLAMA_IP:11434/api/generate"
    OLLAMA_MODEL  = "gemma3"
    BASE_SPEED    = 30
    SLOW_SPEED    = 20
    TURN_ANGLE    = 35
    FRONT_CLEAR   = 800
    FRONT_CAUTION = 500
    SIDE_CLEAR    = 300

POLL_INTERVAL = 0.2    # seconds between navigator decisions

# ── Vision config ──────────────────────────────────────────────────────────────
VISION_ENABLED       = True
VISION_INTERVAL      = 3      # seconds between Gemma 3 queries during search
APPROACH_STOP_CM     = 30     # ultrasonic distance to stop when approaching target
SEARCH_ROTATE_STEPS  = 15     # number of loop iterations to rotate during search
SEARCH_FORWARD_STEPS = 25     # number of loop iterations to drive forward during search

# ── Vision state ───────────────────────────────────────────────────────────────
vision_state = {
    "description": "Waiting for first vision query...",
    "hint":        "none",
    "target_seen": False,
    "target_where": "",       # left, center, right — where in frame
    "timestamp":   0,
    "processing":  False,
    "query_count": 0,
}

# ── Room map ───────────────────────────────────────────────────────────────────
GRID_SIZE   = 100
GRID_RES    = 100
grid        = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]
robot_x     = GRID_SIZE // 2
robot_y     = GRID_SIZE // 2
robot_angle = 0.0

# ── Navigator state ────────────────────────────────────────────────────────────
running       = True
last_decision = "IDLE"
decision_log  = deque(maxlen=20)

# Search pattern state
search_phase        = "forward"   # "forward" or "rotate"
search_step_counter = 0

# ── Logging ────────────────────────────────────────────────────────────────────
def log(msg):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    line = f"[{ts}] {msg}"
    print(line)
    decision_log.append(line)

# ── Agent API helpers ──────────────────────────────────────────────────────────
def get_sensors():
    try:
        r = requests.get(f"{AGENT_URL}/api/sensors", timeout=1.0)
        return r.json()
    except Exception as e:
        log(f"Sensor fetch failed: {e}")
        return None

def get_status():
    try:
        r = requests.get(f"{AGENT_URL}/api/status", timeout=1.0)
        return r.json()
    except Exception as e:
        log(f"Status fetch failed: {e}")
        return None

def get_task():
    try:
        r = requests.get(f"{AGENT_URL}/api/task", timeout=1.0)
        return r.json()
    except Exception as e:
        log(f"Task fetch failed: {e}")
        return None

def get_lidar():
    try:
        r = requests.get(f"{AGENT_URL}/api/lidar", timeout=1.0)
        return r.json().get("scan", [])
    except Exception as e:
        log(f"LiDAR fetch failed: {e}")
        return []

def send_drive(speed: int, angle: int):
    try:
        requests.post(
            f"{AGENT_URL}/api/drive",
            params={"speed": speed, "angle": angle},
            timeout=0.5
        )
    except Exception as e:
        log(f"Drive command failed: {e}")

def send_stop():
    try:
        requests.post(f"{AGENT_URL}/api/stop", timeout=0.5)
    except Exception as e:
        log(f"Stop command failed: {e}")

def send_buzzer(sound: str = "horn"):
    try:
        requests.post(
            f"{AGENT_URL}/api/buzzer",
            params={"sound": sound},
            timeout=1.0
        )
    except Exception as e:
        log(f"Buzzer command failed: {e}")

def post_decision(decision: str):
    try:
        requests.post(
            f"{AGENT_URL}/api/navigator/decision",
            params={"decision": decision},
            timeout=0.5
        )
    except Exception:
        pass

def post_vision(description: str, hint: str):
    try:
        requests.post(
            f"{AGENT_URL}/api/vision/update",
            params={"description": description, "hint": hint},
            timeout=0.5
        )
    except Exception:
        pass

def post_task_status(status: str):
    try:
        requests.post(
            f"{AGENT_URL}/api/task/status",
            params={"status": status},
            timeout=0.5
        )
    except Exception:
        pass

def post_task_found():
    try:
        requests.post(f"{AGENT_URL}/api/task/found", timeout=0.5)
    except Exception:
        pass

# ── Vision — capture single JPEG frame ────────────────────────────────────────
def capture_frame():
    try:
        req  = urllib.request.urlopen(CAMERA_URL, timeout=5)
        data = b''
        while True:
            chunk = req.read(1024)
            data += chunk
            start = data.find(b'\xff\xd8')
            end   = data.find(b'\xff\xd9')
            if start != -1 and end != -1:
                req.close()
                return data[start:end+2]
    except Exception as e:
        log(f"Frame capture failed: {e}")
        return None

# ── Vision — navigation query (no task) ───────────────────────────────────────
def query_vision_navigation():
    """Standard navigation hint when no task is set."""
    if vision_state["processing"]:
        return

    vision_state["processing"] = True

    def run():
        try:
            frame = capture_frame()
            if frame is None:
                return

            image_b64 = base64.b64encode(frame).decode()

            prompt = """You are a navigation assistant for a small robot car.
Look at this camera image and respond in this EXACT format with no extra text:
DESCRIPTION: [one sentence describing what you see, focusing on obstacles and open paths]
HINT: [exactly one word: forward, left, right, or stop]

Choose HINT based on:
- forward: clear path ahead
- left: obstacle ahead, more space on left
- right: obstacle ahead, more space on right
- stop: blocked in all directions"""

            response = requests.post(OLLAMA_URL, json={
                "model":  OLLAMA_MODEL,
                "prompt": prompt,
                "images": [image_b64],
                "stream": False
            }, timeout=15)

            text = response.json().get('response', '')

            desc = ""
            hint = "none"
            for line in text.strip().split('\n'):
                if line.startswith('DESCRIPTION:'):
                    desc = line.replace('DESCRIPTION:', '').strip()
                elif line.startswith('HINT:'):
                    raw = line.replace('HINT:', '').strip().lower()
                    if raw in ['forward', 'left', 'right', 'stop']:
                        hint = raw

            vision_state["description"] = desc
            vision_state["hint"]        = hint
            vision_state["timestamp"]   = time.time()
            vision_state["query_count"] += 1

            log(f"Vision [{vision_state['query_count']}]: {desc} → hint={hint}")
            post_vision(desc, hint)

        except Exception as e:
            log(f"Vision navigation query failed: {e}")
        finally:
            vision_state["processing"] = False

    threading.Thread(target=run, daemon=True).start()

# ── Vision — goal detection query ─────────────────────────────────────────────
def query_vision_goal(task: str):
    """
    Check if the target described in task is visible in the current frame.
    Updates vision_state["target_seen"] and vision_state["target_where"].
    """
    if vision_state["processing"]:
        return

    vision_state["processing"] = True

    def run():
        try:
            frame = capture_frame()
            if frame is None:
                return

            image_b64 = base64.b64encode(frame).decode()

            prompt = f"""You are a navigation assistant for a small robot car.
The robot is looking for: {task}

Look at this camera image and respond in this EXACT format with no extra text:
DESCRIPTION: [one sentence describing what you see]
FOUND: [YES or NO — is the target visible?]
WHERE: [if found: LEFT, CENTER, or RIGHT side of the image. If not found: UNKNOWN]
HINT: [one word for navigation: forward, left, right, or stop]

Be accurate. Only say YES if you are confident the target is visible."""

            response = requests.post(OLLAMA_URL, json={
                "model":  OLLAMA_MODEL,
                "prompt": prompt,
                "images": [image_b64],
                "stream": False
            }, timeout=15)

            text = response.json().get('response', '')

            desc        = ""
            found       = False
            where       = "unknown"
            hint        = "none"

            for line in text.strip().split('\n'):
                if line.startswith('DESCRIPTION:'):
                    desc = line.replace('DESCRIPTION:', '').strip()
                elif line.startswith('FOUND:'):
                    found = 'YES' in line.upper()
                elif line.startswith('WHERE:'):
                    raw_where = line.replace('WHERE:', '').strip().upper()
                    if raw_where in ['LEFT', 'CENTER', 'RIGHT']:
                        where = raw_where.lower()
                elif line.startswith('HINT:'):
                    raw = line.replace('HINT:', '').strip().lower()
                    if raw in ['forward', 'left', 'right', 'stop']:
                        hint = raw

            vision_state["description"]  = desc
            vision_state["target_seen"]  = found
            vision_state["target_where"] = where
            vision_state["hint"]         = hint
            vision_state["timestamp"]    = time.time()
            vision_state["query_count"] += 1

            status = f"FOUND at {where}" if found else "NOT FOUND"
            log(f"Goal [{vision_state['query_count']}]: {desc} → {status}")
            post_vision(desc, hint)

        except Exception as e:
            log(f"Vision goal query failed: {e}")
        finally:
            vision_state["processing"] = False

    threading.Thread(target=run, daemon=True).start()

# ── Room mapping ───────────────────────────────────────────────────────────────
def update_map(scan, rx, ry, heading_deg):
    for point in scan:
        angle_deg = point["angle"]
        dist_mm   = point["distance"]
        if dist_mm <= 0 or dist_mm > 5000:
            continue
        world_angle = math.radians(heading_deg + angle_deg)
        dx = dist_mm * math.sin(world_angle) / GRID_RES
        dy = dist_mm * math.cos(world_angle) / GRID_RES
        gx = int(rx + dx)
        gy = int(ry - dy)
        if 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE:
            grid[gy][gx] = min(grid[gy][gx] + 1, 10)

def save_map():
    try:
        map_data = {
            "grid":        grid,
            "robot_x":     robot_x,
            "robot_y":     robot_y,
            "robot_angle": robot_angle,
            "grid_size":   GRID_SIZE,
            "grid_res_mm": GRID_RES,
            "timestamp":   datetime.now().isoformat()
        }
        with open("/tmp/picar_map.json", "w") as f:
            json.dump(map_data, f)
    except Exception as e:
        log(f"Map save failed: {e}")

# ── Navigation — standard obstacle avoidance ───────────────────────────────────
def decide_navigate(sensors):
    """Standard obstacle avoidance with vision hint as tiebreaker."""
    global robot_angle

    if sensors.get("reflex_active"):
        return 0, 0, "REFLEX_OVERRIDE"

    if sensors.get("cliff_detected"):
        return -20, 0, "CLIFF_BACKUP"

    lidar  = sensors.get("lidar", {})
    front  = lidar.get("front", 9999)
    left   = lidar.get("left",  9999)
    right  = lidar.get("right", 9999)
    back   = lidar.get("back",  9999)
    us_cm  = sensors.get("ultrasonic_cm", 999)

    us_mm = us_cm * 10
    effective_front = min(front, us_mm) if us_mm > 0 else front
    hint = vision_state["hint"] if VISION_ENABLED else "none"

    if effective_front > FRONT_CLEAR:
        return BASE_SPEED, 0, "FORWARD"

    if effective_front > FRONT_CAUTION:
        return SLOW_SPEED, 0, "FORWARD_SLOW"

    if left > right and left > SIDE_CLEAR:
        robot_angle = (robot_angle - 35) % 360
        suffix = " (vision)" if hint == 'left' else ""
        return SLOW_SPEED, -TURN_ANGLE, f"TURN_LEFT{suffix}"

    elif right > left and right > SIDE_CLEAR:
        robot_angle = (robot_angle + 35) % 360
        suffix = " (vision)" if hint == 'right' else ""
        return SLOW_SPEED, TURN_ANGLE, f"TURN_RIGHT{suffix}"

    elif left > SIDE_CLEAR and right > SIDE_CLEAR:
        if hint == 'right':
            robot_angle = (robot_angle + 35) % 360
            return SLOW_SPEED, TURN_ANGLE, "TURN_RIGHT (vision)"
        else:
            robot_angle = (robot_angle - 35) % 360
            return SLOW_SPEED, -TURN_ANGLE, "TURN_LEFT (vision)"

    elif left > SIDE_CLEAR:
        robot_angle = (robot_angle - 35) % 360
        return SLOW_SPEED, -TURN_ANGLE, "TURN_LEFT"

    elif right > SIDE_CLEAR:
        robot_angle = (robot_angle + 35) % 360
        return SLOW_SPEED, TURN_ANGLE, "TURN_RIGHT"

    elif back > 300:
        return -SLOW_SPEED, 0, "REVERSE"

    else:
        return 0, 0, "STUCK"

# ── Navigation — search pattern ────────────────────────────────────────────────
def decide_search(sensors):
    """
    Search pattern: drive forward avoiding obstacles, then rotate to scan.
    Alternates between FORWARD and ROTATE phases.
    """
    global search_phase, search_step_counter, robot_angle

    if sensors.get("reflex_active"):
        return 0, 0, "REFLEX_OVERRIDE"

    if sensors.get("cliff_detected"):
        return -20, 0, "CLIFF_BACKUP"

    lidar  = sensors.get("lidar", {})
    front  = lidar.get("front", 9999)
    left   = lidar.get("left",  9999)
    right  = lidar.get("right", 9999)
    us_cm  = sensors.get("ultrasonic_cm", 999)
    us_mm  = us_cm * 10
    effective_front = min(front, us_mm) if us_mm > 0 else front

    search_step_counter += 1

    if search_phase == "forward":
        # Drive forward, avoiding obstacles
        if search_step_counter >= SEARCH_FORWARD_STEPS:
            search_phase        = "rotate"
            search_step_counter = 0
            log("Search: switching to ROTATE phase")

        # Still need to avoid obstacles while searching
        if effective_front < FRONT_CAUTION:
            if left > right and left > SIDE_CLEAR:
                return SLOW_SPEED, -TURN_ANGLE, "SEARCH_TURN_LEFT"
            elif right > SIDE_CLEAR:
                return SLOW_SPEED, TURN_ANGLE, "SEARCH_TURN_RIGHT"
            else:
                search_phase        = "rotate"
                search_step_counter = 0
                return SLOW_SPEED, TURN_ANGLE, "SEARCH_TURN_RIGHT"

        return SLOW_SPEED, 0, "SEARCHING_FORWARD"

    else:  # rotate phase
        if search_step_counter >= SEARCH_ROTATE_STEPS:
            search_phase        = "forward"
            search_step_counter = 0
            log("Search: switching to FORWARD phase")

        # Rotate in place to scan environment
        robot_angle = (robot_angle + 35) % 360
        return 0, TURN_ANGLE, "SEARCHING_ROTATE"

# ── Navigation — approach target ───────────────────────────────────────────────
def decide_approach(sensors):
    """
    Move toward the target using vision hint for steering.
    Stop when ultrasonic detects we're close enough.
    """
    if sensors.get("reflex_active"):
        return 0, 0, "REFLEX_OVERRIDE"

    if sensors.get("cliff_detected"):
        return -20, 0, "CLIFF_BACKUP"

    us_cm  = sensors.get("ultrasonic_cm", 999)
    lidar  = sensors.get("lidar", {})
    front  = lidar.get("front", 9999)
    us_mm  = us_cm * 10
    effective_front = min(front, us_mm) if us_mm > 0 else front

    # Close enough — goal reached!
    if 0 < us_cm <= APPROACH_STOP_CM:
        return 0, 0, "GOAL_REACHED"

    # Steer toward target based on where it is in the frame
    where = vision_state.get("target_where", "center")
    if where == "left":
        angle = -int(TURN_ANGLE * 0.5)   # gentle left
    elif where == "right":
        angle = int(TURN_ANGLE * 0.5)    # gentle right
    else:
        angle = 0                          # straight ahead

    # Slow down if getting close
    if effective_front < FRONT_CAUTION:
        return SLOW_SPEED, angle, "APPROACHING_SLOW"

    return BASE_SPEED, angle, "APPROACHING"

# ── Update robot position estimate ─────────────────────────────────────────────
def update_position(speed, angle, dt):
    global robot_x, robot_y, robot_angle
    if speed == 0:
        return
    dist_mm = (speed / 100.0) * 500 * dt
    if abs(angle) > 5:
        robot_angle = (robot_angle + angle * 0.1) % 360
    rad = math.radians(robot_angle)
    robot_x += int(dist_mm * math.sin(rad) / GRID_RES)
    robot_y -= int(dist_mm * math.cos(rad) / GRID_RES)
    robot_x = max(1, min(GRID_SIZE - 2, robot_x))
    robot_y = max(1, min(GRID_SIZE - 2, robot_y))

# ── Main loop ──────────────────────────────────────────────────────────────────
def main():
    global last_decision, running, search_phase, search_step_counter

    log("PiCar Navigator starting...")
    log(f"Agent URL:    {AGENT_URL}")
    log(f"Camera URL:   {CAMERA_URL}")
    log(f"Ollama URL:   {OLLAMA_URL}")
    log(f"Vision:       {'ENABLED' if VISION_ENABLED else 'DISABLED'} (every {VISION_INTERVAL}s)")
    log(f"Thresholds:   front_clear={FRONT_CLEAR}mm, front_caution={FRONT_CAUTION}mm")
    log(f"Speed:        base={BASE_SPEED}, slow={SLOW_SPEED}")
    log(f"Approach stop: {APPROACH_STOP_CM}cm")
    log("Waiting for agent to be in AUTONOMOUS mode...")

    map_save_counter  = 0
    last_vision_time  = 0
    goal_reached      = False

    while running:
        loop_start = time.time()

        try:
            # Check agent is reachable and in autonomous mode
            status = get_status()
            if status is None:
                log("Agent unreachable — waiting...")
                time.sleep(2)
                continue

            if status.get("mode") != "autonomous":
                if last_decision != "IDLE":
                    log("Not in autonomous mode — navigator idling.")
                    last_decision = "IDLE"
                    goal_reached  = False
                    search_phase  = "forward"
                    search_step_counter = 0
                    vision_state["target_seen"] = False
                time.sleep(0.5)
                continue

            # Get current task
            task_data = get_task()
            task      = task_data.get("task", "") if task_data else ""
            task_status = task_data.get("status", "IDLE") if task_data else "IDLE"

            # Reset if task was cleared
            if not task and goal_reached:
                goal_reached = False
                vision_state["target_seen"] = False
                search_phase = "forward"
                search_step_counter = 0

            # Don't keep driving after goal reached
            if goal_reached or task_status == "GOAL_REACHED":
                if last_decision != "GOAL_REACHED":
                    log(f"Task complete! '{task}' — waiting for new task.")
                    last_decision = "GOAL_REACHED"
                time.sleep(0.5)
                continue

            # Get sensor data
            sensors = get_sensors()
            if sensors is None:
                time.sleep(POLL_INTERVAL)
                continue

            # Get full LiDAR scan for mapping
            scan = get_lidar()
            if scan:
                update_map(scan, robot_x, robot_y, robot_angle)

            # ── Vision queries ─────────────────────────────────────────────────
            now = time.time()
            if VISION_ENABLED and (now - last_vision_time >= VISION_INTERVAL):
                if task:
                    query_vision_goal(task)
                else:
                    query_vision_navigation()
                last_vision_time = now

            # ── State machine ──────────────────────────────────────────────────
            if not task:
                # No task — standard obstacle avoidance
                speed, angle, decision = decide_navigate(sensors)

            elif vision_state["target_seen"]:
                # Target visible — approach it
                speed, angle, decision = decide_approach(sensors)
                post_task_status("APPROACHING")

                # Check if we reached it
                if decision == "GOAL_REACHED":
                    log(f"🎯 GOAL REACHED: '{task}'")
                    send_stop()
                    goal_reached = True
                    post_task_found()

                    # Celebrate — honk twice and flash dashboard
                    def celebrate():
                        time.sleep(0.3)
                        send_buzzer("horn")
                        time.sleep(1.0)
                        send_buzzer("horn")
                    threading.Thread(target=celebrate, daemon=True).start()

            else:
                # Task set but target not seen — search
                speed, angle, decision = decide_search(sensors)
                post_task_status("SEARCHING")

            # Log and post decision if changed
            if decision != last_decision:
                lidar_data = sensors.get('lidar', {})
                log(f"Decision: {last_decision} → {decision} "
                    f"(F:{lidar_data.get('front', 0):.0f}mm "
                    f"L:{lidar_data.get('left', 0):.0f}mm "
                    f"R:{lidar_data.get('right', 0):.0f}mm "
                    f"US:{sensors.get('ultrasonic_cm', 0):.1f}cm "
                    f"vision={vision_state['hint']} "
                    f"target={'YES' if vision_state['target_seen'] else 'NO'})")
                last_decision = decision
                post_decision(decision)

            # Send drive command (unless goal reached)
            if not goal_reached:
                send_drive(speed, angle)

            update_position(speed, angle, POLL_INTERVAL)

            # Save map every 10 loops
            map_save_counter += 1
            if map_save_counter >= 10:
                save_map()
                map_save_counter = 0

        except KeyboardInterrupt:
            running = False
            break
        except Exception as e:
            log(f"Navigator error: {e}")

        elapsed    = time.time() - loop_start
        sleep_time = max(0, POLL_INTERVAL - elapsed)
        time.sleep(sleep_time)

    log("Navigator stopping — sending stop command...")
    send_stop()
    save_map()
    log("Navigator stopped. Final map saved to /tmp/picar_map.json")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        send_stop()
        print("\nNavigator stopped.")
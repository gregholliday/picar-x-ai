#!/usr/bin/env python3
"""
PiCar-X Navigator with Goal-Directed Navigation and Vision
Runs on Fedora. Polls the Pi agent for sensor data and makes
autonomous driving decisions. Supports task-based navigation:
  "find the red Coca-Cola aluminum can"
  "find the copper tea kettle"
  etc.

State machine:
  IDLE         → no task, standard obstacle avoidance
  SEARCHING    → task set, target not seen, drive+rotate pattern
  LOCKING      → first detection, car stops, waits for confirmation
  APPROACHING  → target confirmed, drive toward locked heading
  GOAL_REACHED → close enough, stop and honk

Key design decisions:
  - When LOCKING, the car stops completely facing the target
  - When confirmed, the heading at that moment is locked
  - APPROACHING drives toward the locked heading
  - Vision WHERE used only for micro-corrections (±7°)
  - Confirmations only count when WHERE is known (not unknown)
  - Multi-word tasks require ALL keywords to match

Usage:
    python3 picar_navigator.py
    The PiCar must be in AUTONOMOUS mode via the dashboard.
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
    try:
        from config import (VISION_ENABLED, VISION_INTERVAL, VISION_TIMEOUT,
                            TARGET_CONFIRM_NEEDED, APPROACH_STOP_CM,
                            SEARCH_FORWARD_STEPS, SEARCH_ROTATE_STEPS)
    except ImportError:
        VISION_ENABLED        = True
        VISION_INTERVAL       = 3
        VISION_TIMEOUT        = 30
        TARGET_CONFIRM_NEEDED = 2
        APPROACH_STOP_CM      = 30
        SEARCH_FORWARD_STEPS  = 25
        SEARCH_ROTATE_STEPS   = 15

    AGENT_URL  = f"http://{PI_IP}:{AGENT_PORT}"
    CAMERA_URL = f"http://{PI_IP}:{CAMERA_PORT}/mjpg"
    OLLAMA_URL = f"http://{OLLAMA_IP}:{OLLAMA_PORT}/api/generate"
    print(f"Config loaded. Agent: {AGENT_URL} | Ollama: {OLLAMA_URL} | Model: {OLLAMA_MODEL}")

except ImportError as e:
    print(f"config.py not found ({e}) — using defaults.")
    AGENT_URL             = "http://YOUR_PI_IP:8000"
    CAMERA_URL            = "http://YOUR_PI_IP:9000/mjpg"
    OLLAMA_URL            = "http://YOUR_OLLAMA_IP:11434/api/generate"
    OLLAMA_MODEL          = "qwen2.5vl:latest"
    BASE_SPEED            = 30
    SLOW_SPEED            = 20
    TURN_ANGLE            = 35
    FRONT_CLEAR           = 800
    FRONT_CAUTION         = 500
    SIDE_CLEAR            = 300
    VISION_ENABLED        = True
    VISION_INTERVAL       = 3
    VISION_TIMEOUT        = 30
    TARGET_CONFIRM_NEEDED = 3
    APPROACH_STOP_CM      = 30
    SEARCH_FORWARD_STEPS  = 25
    SEARCH_ROTATE_STEPS   = 15

POLL_INTERVAL            = 0.2
LOCKING_VISION_INTERVAL  = 2.0
APPROACH_VISION_INTERVAL = 1.5
OPTION_A_TURN_SPEED      = 20   # outside wheel speed for Option A turns

# Lost tolerance — how many consecutive NOT FOUND before giving up
# Split by state: be patient when locked/approaching, fast when searching
TARGET_LOST_TOLERANCE          = 2   # searching  — give up fast, wrong object
TARGET_LOST_TOLERANCE_LOCKING  = 4   # locking    — be patient, car is stopped
TARGET_LOST_TOLERANCE_APPROACH = 5   # approaching — object may briefly leave frame

# ── Vision state ───────────────────────────────────────────────────────────────
vision_state = {
    "description":          "Waiting for first vision query...",
    "hint":                 "none",
    "target_seen":          False,
    "target_where":         "unknown",
    "target_confirm_count": 0,
    "target_lost_count":    0,
    "lock_heading":         None,
    "timestamp":            0,
    "processing":           False,
    "query_count":          0,
    "target_center_count":  0,
    "last_where":          "unknown",
}

# ── Room map ───────────────────────────────────────────────────────────────────
GRID_SIZE   = 100
GRID_RES    = 100
grid        = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]
robot_x     = GRID_SIZE // 2
robot_y     = GRID_SIZE // 2
robot_angle = 0.0

# ── Navigator state ────────────────────────────────────────────────────────────
running             = True
last_decision       = "IDLE"
decision_log        = deque(maxlen=20)
search_phase        = "forward"
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
        requests.post(f"{AGENT_URL}/api/drive",
                      params={"speed": speed, "angle": angle}, timeout=0.5)
    except Exception as e:
        log(f"Drive command failed: {e}")

def send_stop():
    try:
        requests.post(f"{AGENT_URL}/api/stop", timeout=0.5)
    except Exception as e:
        log(f"Stop command failed: {e}")

def send_turn(direction: str, speed: int = 20, angle: int = 35):
    """
    Option A turn via agent /api/turn endpoint.
    direction: 'left' or 'right'
    Front wheels steer + outside wheel drives + inside wheel stops.
    """
    try:
        requests.post(f"{AGENT_URL}/api/turn",
                      params={"direction": direction,
                              "speed":     speed,
                              "angle":     angle}, timeout=0.5)
    except Exception as e:
        log(f"Turn command failed: {e}")

def send_buzzer(sound: str = "horn"):
    try:
        requests.post(f"{AGENT_URL}/api/buzzer",
                      params={"sound": sound}, timeout=1.0)
    except Exception as e:
        log(f"Buzzer command failed: {e}")

def post_decision(decision: str):
    try:
        requests.post(f"{AGENT_URL}/api/navigator/decision",
                      params={"decision": decision}, timeout=0.5)
    except Exception:
        pass

def post_vision(description: str, hint: str):
    try:
        requests.post(f"{AGENT_URL}/api/vision/update",
                      params={"description": description, "hint": hint}, timeout=0.5)
    except Exception:
        pass

def post_task_status(status: str):
    try:
        requests.post(f"{AGENT_URL}/api/task/status",
                      params={"status": status}, timeout=0.5)
    except Exception:
        pass

def post_task_found():
    try:
        requests.post(f"{AGENT_URL}/api/task/found", timeout=0.5)
    except Exception:
        pass

# ── Vision — capture frames ────────────────────────────────────────────────────
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

def capture_best_frame(n=3):
    frames = []
    for i in range(n):
        frame = capture_frame()
        if frame:
            frames.append(frame)
        if i < n - 1:
            time.sleep(0.3)
    return max(frames, key=len) if frames else None

# ── Vision — response parser ───────────────────────────────────────────────────
def parse_vision_response(text, task):
    """
    Parse free-form model response using keyword matching.
    Multi-word tasks require ALL keywords to match — reduces false positives.
    """
    text_lower = text.lower()

    stop_words = {'find', 'the', 'a', 'an', 'some', 'look', 'for',
                  'get', 'locate', 'search', 'seek', 'identify', 'spot'}
    task_words = [w for w in task.lower().split() if w not in stop_words]

    # Multi-word tasks require ALL keywords — prevents Yeti/kettle confusion
    if len(task_words) >= 2:
        found = all(word in text_lower for word in task_words)
    else:
        found = any(word in text_lower for word in task_words)

    negative = any(phrase in text_lower for phrase in [
        'no ', 'not ', "don't", "doesn't", "can't", 'cannot',
        'no sign', 'not visible', 'not present', 'not found',
        'unable to', 'do not see', 'i cannot'
    ])
    positive = any(phrase in text_lower for phrase in [
        'yes', 'i see', 'i can see', 'there is', 'there are',
        'visible', 'present', 'i found', 'i notice'
    ])
    if negative and not positive:
        found = False

    where = "unknown"
    if   'left'   in text_lower: where = 'left'
    elif 'right'  in text_lower: where = 'right'
    elif any(w in text_lower for w in ['center', 'middle', 'front', 'straight', 'ahead']):
        where = 'center'

    if found:
        hint = where if where != 'unknown' else 'forward'
    else:
        if   'left'  in text_lower: hint = 'left'
        elif 'right' in text_lower: hint = 'right'
        else:                        hint = 'forward'

    high_conf = found and any(p in text_lower for p in [
        'yes', 'i can see', 'i see', 'there is', 'clearly',
        'definitely', 'confident', 'visible', 'present', 'i found'
    ])
    confidence = 'HIGH' if high_conf else ('MEDIUM' if found else 'LOW')

    return found, where, hint, confidence

# ── Vision — navigation/search query ──────────────────────────────────────────
def query_vision_navigation(task: str = ""):
    """
    Vision query used during SEARCHING (no confirmed target yet).
    If a task is set, uses a goal-aware prompt to help steer toward the target.
    If no task, uses a plain navigation prompt for obstacle avoidance hints.
    """
    if vision_state["processing"]:
        return
    vision_state["processing"] = True

    def run():
        try:
            frame = capture_best_frame()
            if frame is None:
                return

            image_b64 = base64.b64encode(frame).decode()

            if task:
                prompt = f"""You are a small robot car searching for: {task}

Look carefully at this image and answer:
1. What objects do you see on the floor or in the scene?
2. Do you see: {task}?
3. If yes, describe SPECIFICALLY why you are sure it matches.
4. If yes, is it on the LEFT, CENTER, or RIGHT side of the image?
5. What direction should the robot move to find it?

Important: Only say YES if the object EXACTLY matches the description.
Be conservative — say NO if you are not certain.
Be specific and concise."""
            else:
                prompt = """Look at this camera image and answer briefly:
1. What obstacles or objects are ahead?
2. Is the path clear to drive forward?
3. Which direction (left, right, or forward) is most open?"""

            response = requests.post(OLLAMA_URL, json={
                "model": OLLAMA_MODEL, "prompt": prompt,
                "images": [image_b64], "stream": False
            }, timeout=VISION_TIMEOUT)

            text       = response.json().get('response', '')
            text_lower = text.lower()

            hint = "forward"
            if   'left'    in text_lower: hint = 'left'
            elif 'right'   in text_lower: hint = 'right'
            elif 'blocked' in text_lower or 'stop' in text_lower: hint = 'stop'

            desc = text[:150].replace('\n', ' ')
            vision_state["description"]  = desc
            vision_state["hint"]         = hint
            vision_state["timestamp"]    = time.time()
            vision_state["query_count"] += 1

            log(f"Vision nav [{vision_state['query_count']}]: hint={hint} | {desc[:80]}")
            post_vision(desc, hint)

        except Exception as e:
            log(f"Vision navigation query failed: {e}")
        finally:
            vision_state["processing"] = False

    threading.Thread(target=run, daemon=True).start()

# ── Vision — goal detection query ─────────────────────────────────────────────
def query_vision_goal(task: str):
    """
    Ask the vision model if the confirmed target is still visible.
    Used during LOCKING and APPROACHING.

    Key rules:
    - Confirmations only count when WHERE is known (not 'unknown')
    - WHERE=unknown means model sees something but can't locate it —
      unreliable for approach, so we don't count it
    - Requires TARGET_CONFIRM_NEEDED consecutive WHERE-known detections
    """
    if vision_state["processing"]:
        return
    vision_state["processing"] = True

    def run():
        try:
            frame = capture_best_frame(n=3)
            if frame is None:
                return

            image_b64 = base64.b64encode(frame).decode()
            prompt = f"""You are a robot car. You are looking for: {task}

Look carefully at this image and answer these questions:
1. What objects do you see on the floor or in the scene?
2. Do you see: {task}?
3. If yes, describe SPECIFICALLY why you are sure it matches the description.
4. If yes, is it on the LEFT, CENTER, or RIGHT side of the image?
5. What direction should the robot move to get closer to it?

Important: Only say YES if the object EXACTLY matches the description.
Be conservative — say NO if you are not certain.
Be specific and concise."""

            response = requests.post(OLLAMA_URL, json={
                "model": OLLAMA_MODEL, "prompt": prompt,
                "images": [image_b64], "stream": False
            }, timeout=VISION_TIMEOUT)

            text  = response.json().get('response', '')
            found, where, hint, confidence = parse_vision_response(text, task)

            if found and confidence in ("HIGH", "MEDIUM"):
                vision_state["target_confirm_count"] += 1
                vision_state["target_lost_count"]     = 0

                if where == "center":
                    vision_state["target_center_count"] += 1
                else:
                    vision_state["target_center_count"] = 0

                log_suffix = f"WHERE={where} — confirms={vision_state['target_confirm_count']} center={vision_state['target_center_count']}"
            else:
                vision_state["target_lost_count"] += 1
                vision_state["target_center_count"] = 0

                # Pick tolerance based on current state
                if vision_state["target_seen"]:
                    lost_tolerance = TARGET_LOST_TOLERANCE_APPROACH
                elif vision_state["target_confirm_count"] > 0:
                    lost_tolerance = TARGET_LOST_TOLERANCE_LOCKING
                else:
                    lost_tolerance = TARGET_LOST_TOLERANCE

                log_suffix = f"lost={vision_state['target_lost_count']}/{lost_tolerance}"
                if vision_state["target_lost_count"] >= lost_tolerance:
                    if vision_state["target_confirm_count"] > 0:
                        log(f"Target lost after {lost_tolerance} misses — resuming search")
                    vision_state["target_confirm_count"] = 0
                    vision_state["target_seen"]          = False
                    vision_state["target_lost_count"]    = 0
                    vision_state["lock_heading"]         = None

            if vision_state["target_confirm_count"] >= TARGET_CONFIRM_NEEDED:
                if not vision_state["target_seen"]:
                    log(f"🎯 Target confirmed after "
                        f"{vision_state['target_confirm_count']} WHERE-known detections!")
                vision_state["target_seen"] = True

            vision_state["description"]  = text[:150].replace('\n', ' ')
            vision_state["hint"]         = hint
            vision_state["target_where"] = where
            vision_state["timestamp"]    = time.time()
            vision_state["query_count"] += 1

            status = (f"FOUND at {where} (conf={confidence}, "
                      f"confirms={vision_state['target_confirm_count']}/{TARGET_CONFIRM_NEEDED}) "
                      f"— {log_suffix}"
                      if found else
                      f"NOT FOUND ({log_suffix})")
            log(f"Vision goal [{vision_state['query_count']}]: {status}")
            post_vision(vision_state["description"], hint)

        except Exception as e:
            log(f"Vision goal query failed: {e}")
        finally:
            vision_state["processing"] = False

    threading.Thread(target=run, daemon=True).start()

# ── Room mapping ───────────────────────────────────────────────────────────────
def update_map(scan, rx, ry, heading_deg):
    for point in scan:
        dist_mm = point["distance"]
        if dist_mm <= 0 or dist_mm > 5000:
            continue
        world_angle = math.radians(heading_deg + point["angle"])
        gx = int(rx + dist_mm * math.sin(world_angle) / GRID_RES)
        gy = int(ry - dist_mm * math.cos(world_angle) / GRID_RES)
        if 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE:
            grid[gy][gx] = min(grid[gy][gx] + 1, 10)

def save_map():
    try:
        with open("/tmp/picar_map.json", "w") as f:
            json.dump({
                "grid": grid, "robot_x": robot_x, "robot_y": robot_y,
                "robot_angle": robot_angle, "grid_size": GRID_SIZE,
                "grid_res_mm": GRID_RES, "timestamp": datetime.now().isoformat()
            }, f)
    except Exception as e:
        log(f"Map save failed: {e}")

# ── Navigation — standard obstacle avoidance ───────────────────────────────────
def decide_navigate(sensors):
    global robot_angle

    if sensors.get("reflex_active"):  return 0, 0, "REFLEX_OVERRIDE"
    if sensors.get("cliff_detected"): return -SLOW_SPEED, 0, "CLIFF_BACKUP"

    lidar  = sensors.get("lidar", {})
    front  = lidar.get("front", 9999)
    left   = lidar.get("left",  9999)
    right  = lidar.get("right", 9999)
    back   = lidar.get("back",  9999)
    us_cm  = sensors.get("ultrasonic_cm", 999)
    us_mm  = us_cm * 10
    efront = min(front, us_mm) if us_mm > 0 else front
    hint   = vision_state["hint"] if VISION_ENABLED else "none"

    if efront > FRONT_CLEAR:   return BASE_SPEED, 0, "FORWARD"
    if efront > FRONT_CAUTION: return SLOW_SPEED, 0, "FORWARD_SLOW"

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
        robot_angle = (robot_angle - 35) % 360
        return SLOW_SPEED, -TURN_ANGLE, "TURN_LEFT (vision)"
    elif left  > SIDE_CLEAR:
        robot_angle = (robot_angle - 35) % 360
        return SLOW_SPEED, -TURN_ANGLE, "TURN_LEFT"
    elif right > SIDE_CLEAR:
        robot_angle = (robot_angle + 35) % 360
        return SLOW_SPEED, TURN_ANGLE, "TURN_RIGHT"
    elif back > 300: return -SLOW_SPEED, 0, "REVERSE"
    else:            return 0, 0, "STUCK"

# ── Navigation — search pattern ────────────────────────────────────────────────
def decide_search(sensors):
    global search_phase, search_step_counter, robot_angle

    if sensors.get("reflex_active"):  return 0, 0, "REFLEX_OVERRIDE"
    if sensors.get("cliff_detected"): return -SLOW_SPEED, 0, "CLIFF_BACKUP"

    lidar  = sensors.get("lidar", {})
    front  = lidar.get("front", 9999)
    left   = lidar.get("left",  9999)
    right  = lidar.get("right", 9999)
    us_cm  = sensors.get("ultrasonic_cm", 999)
    us_mm  = us_cm * 10
    efront = min(front, us_mm) if us_mm > 0 else front

    search_step_counter += 1

    if search_phase == "forward":
        if search_step_counter >= SEARCH_FORWARD_STEPS:
            search_phase        = "rotate"
            search_step_counter = 0
            log("Search: switching to ROTATE phase")

        if efront < FRONT_CAUTION:
            if left > right and left > SIDE_CLEAR:
                return SLOW_SPEED, -TURN_ANGLE, "SEARCH_TURN_LEFT"
            elif right > SIDE_CLEAR:
                return SLOW_SPEED, TURN_ANGLE, "SEARCH_TURN_RIGHT"
            else:
                search_phase        = "rotate"
                search_step_counter = 0
                return SLOW_SPEED, TURN_ANGLE, "SEARCH_TURN_RIGHT"

        return SLOW_SPEED, 0, "SEARCHING_FORWARD"

    else:
        if search_step_counter >= SEARCH_ROTATE_STEPS:
            search_phase        = "forward"
            search_step_counter = 0
            log("Search: switching to FORWARD phase")

        robot_angle = (robot_angle + 35) % 360
        return 0, TURN_ANGLE, "SEARCHING_ROTATE"
# ── Navigation — locking ───────────────────────────────────────────────────────
def decide_lock(sensors):
    if sensors.get("cliff_detected"):
        return -SLOW_SPEED, 0, "CLIFF_BACKUP"

    where = vision_state.get("target_where", "unknown")

    ALIGN_ANGLE = int(TURN_ANGLE * 0.5)  # ~17°

    if where == "left":
        return 0, -ALIGN_ANGLE, "LOCKING_ALIGN_LEFT"
    elif where == "right":
        return 0, ALIGN_ANGLE, "LOCKING_ALIGN_RIGHT"
    else:
        return 0, 0, "LOCKING_CENTERED"

# ── Navigation — approach target ───────────────────────────────────────────────
def decide_approach(sensors):
    if sensors.get("cliff_detected"):
        return -SLOW_SPEED, 0, "CLIFF_BACKUP"

    now = time.time()
    vision_age = now - vision_state.get("timestamp", 0)

    # 🚫 DO NOT MOVE if vision is stale or processing
    if vision_state["processing"] or vision_age > 1.0:
        return 0, 0, "APPROACH_WAIT_VISION"

    us_cm  = sensors.get("ultrasonic_cm", 999)
    lidar  = sensors.get("lidar", {})
    front  = lidar.get("front", 9999)
    us_mm  = us_cm * 10
    efront = min(front, us_mm) if us_mm > 0 else front

    where = vision_state.get("target_where", "center")

    # 🛑 STOP CONDITIONS
    if efront < 150:
        return 0, 0, "GOAL_REACHED"

    if where == "center" and 0 < us_cm <= APPROACH_STOP_CM:
        return 0, 0, "GOAL_REACHED"

    # 🎯 ALIGN FIRST, THEN MOVE
    ALIGN_ANGLE = int(TURN_ANGLE * 0.5)

    if where == "left":
        return 0, -ALIGN_ANGLE, "APPROACH_ALIGN_LEFT"

    elif where == "right":
        return 0, ALIGN_ANGLE, "APPROACH_ALIGN_RIGHT"

    # 🚗 ONLY MOVE FORWARD WHEN CENTERED
    if efront < FRONT_CAUTION:
        return SLOW_SPEED, 0, "APPROACH_FORWARD_SLOW"

    return SLOW_SPEED, 0, "APPROACH_FORWARD"

# ── Update robot position estimate ─────────────────────────────────────────────
def update_position(speed, angle, dt):
    global robot_x, robot_y, robot_angle
    if speed == 0: return
    dist_mm = (speed / 100.0) * 500 * dt
    if abs(angle) > 5:
        robot_angle = (robot_angle + angle * 0.1) % 360
    rad = math.radians(robot_angle)
    robot_x = max(1, min(GRID_SIZE-2, robot_x + int(dist_mm * math.sin(rad) / GRID_RES)))
    robot_y = max(1, min(GRID_SIZE-2, robot_y - int(dist_mm * math.cos(rad) / GRID_RES)))

def reset_vision():
    vision_state["target_seen"]          = False
    vision_state["target_confirm_count"] = 0
    vision_state["target_lost_count"]    = 0
    vision_state["lock_heading"]         = None
    vision_state["target_where"]         = "unknown"
    vision_state["target_center_count"] = 0

# ── Main loop ──────────────────────────────────────────────────────────────────
def main():
    global last_decision, running, search_phase, search_step_counter

    log("PiCar Navigator starting...")
    log(f"Agent:    {AGENT_URL}")
    log(f"Camera:   {CAMERA_URL}")
    log(f"Ollama:   {OLLAMA_URL}")
    log(f"Model:    {OLLAMA_MODEL}")
    log(f"Vision:   {'ENABLED' if VISION_ENABLED else 'DISABLED'} "
        f"search={VISION_INTERVAL}s lock={LOCKING_VISION_INTERVAL}s "
        f"approach={APPROACH_VISION_INTERVAL}s timeout={VISION_TIMEOUT}s")
    log(f"Confirm:  {TARGET_CONFIRM_NEEDED} WHERE-known detections needed, "
        f"lost=search:{TARGET_LOST_TOLERANCE}/"
        f"lock:{TARGET_LOST_TOLERANCE_LOCKING}/"
        f"approach:{TARGET_LOST_TOLERANCE_APPROACH}")
    log(f"Approach: stop at {APPROACH_STOP_CM}cm, hard stop at 15cm")
    log(f"Steering: micro-correction only during approach (±{int(TURN_ANGLE*0.2)}°)")
    log(f"Speed:    base={BASE_SPEED}, slow={SLOW_SPEED}, turn={TURN_ANGLE}°, option_a={OPTION_A_TURN_SPEED}")
    log("Waiting for AUTONOMOUS mode...")

    map_save_counter = 0
    last_vision_time = 0
    goal_reached     = False

    while running:
        loop_start = time.time()

        try:
            status = get_status()
            if status is None:
                log("Agent unreachable — waiting...")
                time.sleep(2)
                continue

            if status.get("mode") != "autonomous":
                if last_decision != "IDLE":
                    log("Not in autonomous mode — idling.")
                    last_decision = "IDLE"
                    goal_reached  = False
                    search_phase  = "forward"
                    search_step_counter = 0
                    reset_vision()
                time.sleep(0.5)
                continue

            task_data   = get_task()
            task        = task_data.get("task", "")   if task_data else ""
            task_status = task_data.get("status", "") if task_data else ""

            if not task and goal_reached:
                goal_reached = False
                search_phase        = "forward"
                search_step_counter = 0
                reset_vision()

            if goal_reached or task_status == "GOAL_REACHED":
                if last_decision != "GOAL_REACHED":
                    log("Task complete — waiting for new task.")
                    last_decision = "GOAL_REACHED"
                time.sleep(0.5)
                continue

            sensors = get_sensors()
            if sensors is None:
                time.sleep(POLL_INTERVAL)
                continue

            scan = get_lidar()
            if scan:
                update_map(scan, robot_x, robot_y, robot_angle)

            # ── Vision query interval based on state ───────────────────────────
            now      = time.time()
            confirms = vision_state["target_confirm_count"]
            seen     = vision_state["target_seen"]

            if seen:
                interval = APPROACH_VISION_INTERVAL
            elif confirms > 0:
                interval = LOCKING_VISION_INTERVAL
            else:
                interval = VISION_INTERVAL

            if VISION_ENABLED and (now - last_vision_time >= interval):
                if task:
                    # Target detected or being confirmed — use goal query
                    query_vision_goal(task)
                else:
                    query_vision_navigation()
                last_vision_time = now

            # ── State machine ──────────────────────────────────────────────────
            if not task:
                speed, angle, decision = decide_navigate(sensors)

            elif (
                vision_state["target_seen"]
                and vision_state["target_center_count"] >= 2
            ):
                if vision_state["lock_heading"] is None:
                    vision_state["lock_heading"] = robot_angle
                    log(f"Approach heading locked at {robot_angle:.0f}°")

                speed, angle, decision = decide_approach(sensors)
                post_task_status("APPROACHING")

                if decision == "GOAL_REACHED":
                    log(f"🎯 GOAL REACHED: '{task}'")
                    send_stop()
                    goal_reached = True
                    post_task_found()
                    def celebrate():
                        time.sleep(0.3)
                        send_buzzer("horn")
                        time.sleep(1.0)
                        send_buzzer("horn")
                    threading.Thread(target=celebrate, daemon=True).start()

            elif vision_state["target_confirm_count"] > 0:
                speed, angle, decision = decide_lock(sensors)
                post_task_status("LOCKING")

            else:
                speed, angle, decision = decide_search(sensors)
                post_task_status("SEARCHING")

            if decision != last_decision:
                lidar = sensors.get('lidar', {})
                hdg   = vision_state['lock_heading']
                log(f"Decision: {last_decision} → {decision} "
                    f"(F:{lidar.get('front',0):.0f} "
                    f"L:{lidar.get('left',0):.0f} "
                    f"R:{lidar.get('right',0):.0f} "
                    f"US:{sensors.get('ultrasonic_cm',0):.1f}cm "
                    f"where={vision_state['target_where']} "
                    f"seen={vision_state['target_seen']} "
                    f"confirms={vision_state['target_confirm_count']} "
                    f"lost={vision_state['target_lost_count']} "
                    f"lock_hdg={hdg:.0f}° " if hdg is not None else
                    f"Decision: {last_decision} → {decision} "
                    f"(F:{lidar.get('front',0):.0f} "
                    f"L:{lidar.get('left',0):.0f} "
                    f"R:{lidar.get('right',0):.0f} "
                    f"US:{sensors.get('ultrasonic_cm',0):.1f}cm "
                    f"where={vision_state['target_where']} "
                    f"seen={vision_state['target_seen']} "
                    f"confirms={vision_state['target_confirm_count']} "
                    f"lost={vision_state['target_lost_count']} "
                    f"lock_hdg=None)"
                    f"center={vision_state['target_center_count']} "
                    f"vision_age={now - vision_state['timestamp']:.2f}s ")
                last_decision = decision
                post_decision(decision)

            if not goal_reached:
                # Use Option A turns for search rotate/turn decisions
                if decision in ("SEARCHING_ROTATE", "SEARCH_TURN_RIGHT"):
                    send_turn("right", OPTION_A_TURN_SPEED, TURN_ANGLE)
                elif decision in ("SEARCH_TURN_LEFT",):
                    send_turn("left", OPTION_A_TURN_SPEED, TURN_ANGLE)
                else:
                    send_drive(speed, angle)

            update_position(speed, angle, POLL_INTERVAL)

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

    log("Stopping — sending stop command...")
    send_stop()
    save_map()
    log("Navigator stopped.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        send_stop()
        print("\nNavigator stopped.")

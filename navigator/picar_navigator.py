#!/usr/bin/env python3
"""
PiCar-X Navigator with Goal-Directed Vision Navigation

Runs on Fedora. Polls the PiCar FastAPI agent for sensor data and sends
movement decisions back to the Pi. Uses Ollama vision models for target
search, lock, and approach behavior.

Current control model:
  IDLE         -> no task, obstacle avoidance only
  SEARCHING    -> pulse-and-settle search motion while looking for target
  LOCKING      -> target seen, align until centered and confirmed
  APPROACHING  -> target confirmed and centered, approach in short pulses
  GOAL_REACHED -> close enough, stop and honk

Important design choices:
  - Motion is pulse-and-settle, not continuous.
  - The robot does not lock on WHERE=unknown.
  - The target must be centered before approach.
  - Approach safety is governed by ultrasonic + corrected LiDAR distance.
  - LiDAR front readings are corrected for rear-mounted LiDAR position.
"""

import base64
import json
import math
import os
import sys
import threading
import time
import urllib.request
from collections import deque
from datetime import datetime

import requests

# ── Load config ────────────────────────────────────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from config import (  # type: ignore
        PI_IP,
        AGENT_PORT,
        CAMERA_PORT,
        BASE_SPEED,
        SLOW_SPEED,
        TURN_ANGLE,
        FRONT_CLEAR,
        FRONT_CAUTION,
        SIDE_CLEAR,
        OLLAMA_IP,
        OLLAMA_PORT,
        OLLAMA_MODEL,
    )
except ImportError as e:
    print(f"config.py not found ({e}) — using defaults.")
    PI_IP = "YOUR_PI_IP"
    AGENT_PORT = 8000
    CAMERA_PORT = 9000
    BASE_SPEED = 30
    SLOW_SPEED = 15
    TURN_ANGLE = 35
    FRONT_CLEAR = 800
    FRONT_CAUTION = 500
    SIDE_CLEAR = 300
    OLLAMA_IP = "YOUR_OLLAMA_IP"
    OLLAMA_PORT = 11434
    OLLAMA_MODEL = "qwen3-vl:8b"

try:
    from config import (  # type: ignore
        VISION_ENABLED,
        VISION_INTERVAL,
        VISION_TIMEOUT,
        TARGET_CONFIRM_NEEDED,
        APPROACH_STOP_CM,
        SEARCH_FORWARD_STEPS,
        SEARCH_ROTATE_STEPS,
    )
except ImportError:
    VISION_ENABLED = True
    VISION_INTERVAL = 5
    VISION_TIMEOUT = 15
    TARGET_CONFIRM_NEEDED = 3
    APPROACH_STOP_CM = 30
    SEARCH_FORWARD_STEPS = 8
    SEARCH_ROTATE_STEPS = 4

# Optional config values. These can go in config_local.py.
def _config_value(name, default):
    try:
        import config  # type: ignore
        return getattr(config, name, default)
    except Exception:
        return default

POLL_INTERVAL = _config_value("POLL_INTERVAL", 0.2)
LOCKING_VISION_INTERVAL = _config_value("LOCKING_VISION_INTERVAL", 5.0)
APPROACH_VISION_INTERVAL = _config_value("APPROACH_VISION_INTERVAL", 5.0)
OPTION_A_TURN_SPEED = _config_value("OPTION_A_TURN_SPEED", 20)

# Pulse-and-settle motion tuning.
SEARCH_ROTATE_PULSE_SEC = _config_value("SEARCH_ROTATE_PULSE_SEC", 0.45)
SEARCH_TURN_PULSE_SEC = _config_value("SEARCH_TURN_PULSE_SEC", 0.35)
LOCK_ALIGN_PULSE_SEC = _config_value("LOCK_ALIGN_PULSE_SEC", 0.45)
APPROACH_ALIGN_PULSE_SEC = _config_value("APPROACH_ALIGN_PULSE_SEC", 0.20)
SEARCH_FORWARD_PULSE_SEC = _config_value("SEARCH_FORWARD_PULSE_SEC", 0.60)
APPROACH_FORWARD_PULSE_SEC = _config_value("APPROACH_FORWARD_PULSE_SEC", 0.20)
SETTLE_AFTER_MOVE_SEC = _config_value("SETTLE_AFTER_MOVE_SEC", 0.50)
VISION_WAIT_TIMEOUT_SEC = _config_value("VISION_WAIT_TIMEOUT_SEC", 3.50)

# LiDAR is mounted behind the front bumper/camera plane. Correct front clearance.
LIDAR_FRONT_OFFSET_MM = _config_value("LIDAR_FRONT_OFFSET_MM", 150)
APPROACH_HARD_STOP_MM = _config_value("APPROACH_HARD_STOP_MM", 250)
APPROACH_VISION_STALE_SEC = _config_value("APPROACH_VISION_STALE_SEC", 10.0)
APPROACH_MAX_TIMEOUTS = _config_value("APPROACH_MAX_TIMEOUTS", 2)
SEARCH_MAX_TIMEOUTS = _config_value("SEARCH_MAX_TIMEOUTS", 4)
OLLAMA_KEEP_ALIVE = _config_value("OLLAMA_KEEP_ALIVE", "48h")

# Lost tolerance — how many consecutive NOT FOUND responses before giving up.
TARGET_LOST_TOLERANCE = _config_value("TARGET_LOST_TOLERANCE", 2)
TARGET_LOST_TOLERANCE_LOCKING = _config_value("TARGET_LOST_TOLERANCE_LOCKING", 4)
TARGET_LOST_TOLERANCE_APPROACH = _config_value("TARGET_LOST_TOLERANCE_APPROACH", 5)

AGENT_URL = f"http://{PI_IP}:{AGENT_PORT}"
CAMERA_URL = f"http://{PI_IP}:{CAMERA_PORT}/mjpg"
OLLAMA_URL = f"http://{OLLAMA_IP}:{OLLAMA_PORT}/api/generate"

# ── Vision state ───────────────────────────────────────────────────────────────
vision_state = {
    "description": "Waiting for first vision query...",
    "hint": "none",
    "target_seen": False,
    "target_where": "unknown",
    "target_confirm_count": 0,
    "target_center_count": 0,
    "target_lost_count": 0,
    "target_weak_count": 0,
    "vision_timeout_count": 0,
    "last_where": "unknown",
    "lock_heading": None,
    "timestamp": 0.0,
    "processing": False,
    "query_count": 0,
    "last_query_duration": 0.0,
    "last_known_where": "center",
}

# ── Room map ───────────────────────────────────────────────────────────────────
GRID_SIZE = 100
GRID_RES = 100
grid = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]
robot_x = GRID_SIZE // 2
robot_y = GRID_SIZE // 2
robot_angle = 0.0

# ── Navigator state ────────────────────────────────────────────────────────────
running = True
last_decision = "IDLE"
decision_log = deque(maxlen=20)
search_phase = "forward"
search_step_counter = 0
goal_reached = False

# ── Logging ────────────────────────────────────────────────────────────────────
def log(msg: str) -> None:
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    line = f"[{ts}] {msg}"
    print(line, flush=True)
    decision_log.append(line)

# ── Agent API helpers ──────────────────────────────────────────────────────────
def get_json(path: str, timeout: float = 1.0):
    try:
        r = requests.get(f"{AGENT_URL}{path}", timeout=timeout)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        log(f"GET {path} failed: {e}")
        return None


def post(path: str, params=None, timeout: float = 0.5):
    try:
        requests.post(f"{AGENT_URL}{path}", params=params or {}, timeout=timeout)
    except Exception as e:
        log(f"POST {path} failed: {e}")


def get_sensors():
    return get_json("/api/sensors")


def get_status():
    return get_json("/api/status")


def get_task():
    return get_json("/api/task")


def get_lidar():
    data = get_json("/api/lidar")
    return data.get("scan", []) if data else []


def send_drive(speed: int, angle: int):
    post("/api/drive", {"speed": speed, "angle": angle})


def send_stop():
    post("/api/stop")


def send_turn(direction: str, speed: int = OPTION_A_TURN_SPEED, angle: int = TURN_ANGLE):
    post("/api/turn", {"direction": direction, "speed": speed, "angle": angle})


def pulse_turn(direction: str, duration: float, speed: int = OPTION_A_TURN_SPEED, angle: int = TURN_ANGLE):
    log(f"Executing turn pulse: direction={direction} speed={speed} angle={angle} duration={duration:.2f}s")
    send_turn(direction, speed, angle)
    time.sleep(duration)
    send_stop()


def pulse_drive(speed: int, angle: int, duration: float):
    log(f"Executing drive pulse: speed={speed} angle={angle} duration={duration:.2f}s")
    send_drive(speed, angle)
    time.sleep(duration)
    send_stop()


def wait_for_vision_update(previous_ts: float, timeout: float = VISION_WAIT_TIMEOUT_SEC) -> bool:
    start = time.time()
    while time.time() - start < timeout:
        if vision_state["timestamp"] > previous_ts and not vision_state["processing"]:
            return True
        time.sleep(0.05)
    return False


def send_buzzer(sound: str = "horn"):
    post("/api/buzzer", {"sound": sound}, timeout=1.0)


def post_decision(decision: str):
    post("/api/navigator/decision", {"decision": decision})


def post_vision(description: str, hint: str):
    post("/api/vision/update", {"description": description, "hint": hint})


def post_task_status(status: str):
    post("/api/task/status", {"status": status})


def post_task_found():
    post("/api/task/found")

# ── Sensor helpers ─────────────────────────────────────────────────────────────
def corrected_distances(sensors):
    """Return corrected effective front distance and raw values.

    front_raw is measured from the LiDAR, which sits behind the front bumper.
    front_corr estimates clearance from the front of the car.
    efront is the effective front clearance using the most conservative value
    from corrected LiDAR and ultrasonic.
    """
    lidar = sensors.get("lidar", {}) if sensors else {}
    front_raw = float(lidar.get("front", 9999) or 9999)
    left = float(lidar.get("left", 9999) or 9999)
    right = float(lidar.get("right", 9999) or 9999)
    back = float(lidar.get("back", 9999) or 9999)
    us_cm = float(sensors.get("ultrasonic_cm", 999) or 999) if sensors else 999
    us_mm = us_cm * 10 if us_cm > 0 else 9999

    if 0 < front_raw < 9999:
        front_corr = max(0, front_raw - LIDAR_FRONT_OFFSET_MM)
    else:
        front_corr = front_raw

    efront = min(front_corr, us_mm) if us_mm > 0 else front_corr
    return {
        "front_raw": front_raw,
        "front_corr": front_corr,
        "left": left,
        "right": right,
        "back": back,
        "us_cm": us_cm,
        "us_mm": us_mm,
        "efront": efront,
    }

# ── Vision — capture frames ────────────────────────────────────────────────────
def capture_frame():
    try:
        req = urllib.request.urlopen(CAMERA_URL, timeout=5)
        data = b""
        while True:
            data += req.read(1024)
            start = data.find(b"\xff\xd8")
            end = data.find(b"\xff\xd9")
            if start != -1 and end != -1:
                req.close()
                return data[start : end + 2]
    except Exception as e:
        log(f"Frame capture failed: {e}")
        return None


def capture_best_frame(n: int = 2):
    frames = []
    for i in range(n):
        frame = capture_frame()
        if frame:
            frames.append(frame)
        if i < n - 1:
            time.sleep(0.2)
    return max(frames, key=len) if frames else None

# ── Vision — response parser ───────────────────────────────────────────────────
def _task_aliases(task: str):
    t = task.lower()
    aliases = []
    if "kettle" in t or "tea pot" in t or "teapot" in t:
        aliases.extend(["kettle", "tea kettle", "teapot", "tea pot", "copper kettle", "copper tea kettle"])
    return aliases


def parse_vision_response(text: str, task: str):
    text_lower = text.lower()
    stop_words = {
        "find", "the", "a", "an", "some", "look", "for", "get", "locate",
        "search", "seek", "identify", "spot", "please",
    }
    task_words = [w for w in task.lower().replace("-", " ").split() if w not in stop_words]
    aliases = _task_aliases(task)

    alias_found = any(alias in text_lower for alias in aliases)
    if alias_found:
        found = True
    elif len(task_words) >= 2:
        # Less brittle than all words. Require either all words or the noun-like last word.
        found = all(word in text_lower for word in task_words) or task_words[-1] in text_lower
    else:
        found = any(word in text_lower for word in task_words)

    negative = any(phrase in text_lower for phrase in [
        "no ", "not ", "don't", "doesn't", "can't", "cannot", "no sign",
        "not visible", "not present", "not found", "unable to", "do not see", "i cannot",
    ])
    positive = any(phrase in text_lower for phrase in [
        "yes", "i see", "i can see", "there is", "there are", "visible",
        "present", "i found", "i notice", "appears", "looks like",
    ])
    if negative and not positive:
        found = False

    where = "unknown"
    if "left" in text_lower:
        where = "left"
    elif "right" in text_lower:
        where = "right"
    elif any(w in text_lower for w in ["center", "centre", "middle", "front", "straight", "ahead"]):
        where = "center"

    if found:
        hint = where if where != "unknown" else "forward"
    else:
        if "left" in text_lower:
            hint = "left"
        elif "right" in text_lower:
            hint = "right"
        else:
            hint = "forward"

    high_conf = found and any(p in text_lower for p in [
        "yes", "i can see", "i see", "there is", "clearly", "definitely",
        "confident", "visible", "present", "i found",
    ])
    confidence = "HIGH" if high_conf else ("MEDIUM" if found else "LOW")
    return found, where, hint, confidence

# ── Vision queries ─────────────────────────────────────────────────────────────
def _ollama_generate(prompt: str, image_b64: str):
    t0 = time.time()
    response = requests.post(
        OLLAMA_URL,
        json={
            "model": OLLAMA_MODEL,
            "prompt": prompt,
            "images": [image_b64],
            "stream": False,
            "keep_alive": OLLAMA_KEEP_ALIVE,
        },
        timeout=VISION_TIMEOUT,
    )
    elapsed = time.time() - t0
    vision_state["last_query_duration"] = elapsed
    response.raise_for_status()
    return response.json().get("response", ""), elapsed


def query_vision_navigation(task: str = ""):
    if vision_state["processing"]:
        return
    vision_state["processing"] = True

    def run():
        try:
            frame = capture_best_frame(n=1)
            if frame is None:
                return
            image_b64 = base64.b64encode(frame).decode()
            if task:
                prompt = f"""You are helping a small robot car search for: {task}

Return a concise answer.
Do you see the target? If visible, say LEFT, CENTER, or RIGHT.
If not visible, say NOT_VISIBLE.
Also mention the safest direction to move: left, right, forward, or stop."""
            else:
                prompt = """Look at this robot camera image.
Which direction is most open: left, right, forward, or stop?
Be concise."""

            text, elapsed = _ollama_generate(prompt, image_b64)
            text_lower = text.lower()
            hint = "forward"
            if "left" in text_lower:
                hint = "left"
            elif "right" in text_lower:
                hint = "right"
            elif "blocked" in text_lower or "stop" in text_lower:
                hint = "stop"

            desc = text[:150].replace("\n", " ")
            vision_state["description"] = desc
            vision_state["hint"] = hint
            vision_state["timestamp"] = time.time()
            vision_state["query_count"] += 1
            vision_state["vision_timeout_count"] = 0
            log(f"Vision nav [{vision_state['query_count']}]: {elapsed:.2f}s hint={hint} | {desc[:80]}")
            post_vision(desc, hint)
        except Exception as e:
            vision_state["vision_timeout_count"] += 1
            log(f"Vision navigation query failed: {e}")
        finally:
            vision_state["processing"] = False

    threading.Thread(target=run, daemon=True).start()


def query_vision_goal(task: str):
    if vision_state["processing"]:
        return
    vision_state["processing"] = True

    def run():
        try:
            frame = capture_best_frame(n=2)
            if frame is None:
                return
            image_b64 = base64.b64encode(frame).decode()
            prompt = f"""You are localizing a target for a small robot car.
Target: {task}

Answer in this exact compact format:
TARGET=LEFT, TARGET=CENTER, TARGET=RIGHT, or TARGET=NOT_VISIBLE.
Then one short reason.

If the target is visible but partly unclear, choose the closest location.
Accept similar words such as kettle, tea kettle, teapot, or tea pot when the task asks for a tea kettle."""

            text, elapsed = _ollama_generate(prompt, image_b64)
            found, where, hint, confidence = parse_vision_response(text, task)

            # Track last known good direction
            if where in ("left", "center", "right"):
                vision_state["last_known_where"] = where

            if found and confidence in ("HIGH", "MEDIUM"):
                vision_state["target_lost_count"] = 0
                vision_state["vision_timeout_count"] = 0
                if where in ("left", "center", "right"):
                    vision_state["target_confirm_count"] = min(
                        vision_state["target_confirm_count"] + 1,
                        TARGET_CONFIRM_NEEDED + 3,
                    )
                    vision_state["target_weak_count"] = 0
                    if where == "center":
                        vision_state["target_center_count"] = min(
                            vision_state["target_center_count"] + 1,
                            4,
                        )
                    else:
                        vision_state["target_center_count"] = 0
                    log_suffix = (
                        f"WHERE={where} — confirms={vision_state['target_confirm_count']} "
                        f"center={vision_state['target_center_count']}"
                    )
                else:
                    vision_state["target_weak_count"] += 1
                    vision_state["target_center_count"] = 0
                    log_suffix = f"WHERE=unknown — weak={vision_state['target_weak_count']}, not locking yet"
            else:
                vision_state["target_lost_count"] += 1
                vision_state["target_center_count"] = 0

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
                    vision_state["target_center_count"] = 0
                    vision_state["target_seen"] = False
                    vision_state["target_lost_count"] = 0
                    vision_state["target_weak_count"] = 0
                    vision_state["lock_heading"] = None

            if vision_state["target_confirm_count"] >= TARGET_CONFIRM_NEEDED:
                if not vision_state["target_seen"]:
                    log(f"🎯 Target confirmed after {vision_state['target_confirm_count']} WHERE-known detections!")
                vision_state["target_seen"] = True

            vision_state["description"] = text[:150].replace("\n", " ")
            vision_state["hint"] = hint
            vision_state["target_where"] = where
            vision_state["last_where"] = where
            vision_state["timestamp"] = time.time()
            vision_state["query_count"] += 1

            status = (
                f"FOUND at {where} (conf={confidence}, confirms={vision_state['target_confirm_count']}/{TARGET_CONFIRM_NEEDED}) — {log_suffix}"
                if found
                else f"NOT FOUND ({log_suffix})"
            )
            log(f"Vision goal [{vision_state['query_count']}]: {elapsed:.2f}s {status}")
            post_vision(vision_state["description"], hint)
        except Exception as e:
            vision_state["vision_timeout_count"] += 1
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
            json.dump(
                {
                    "grid": grid,
                    "robot_x": robot_x,
                    "robot_y": robot_y,
                    "robot_angle": robot_angle,
                    "grid_size": GRID_SIZE,
                    "grid_res_mm": GRID_RES,
                    "timestamp": datetime.now().isoformat(),
                },
                f,
            )
    except Exception as e:
        log(f"Map save failed: {e}")

# ── Navigation decisions ───────────────────────────────────────────────────────
def decide_navigate(sensors):
    global robot_angle
    if sensors.get("reflex_active"):
        return 0, 0, "REFLEX_OVERRIDE"
    if sensors.get("cliff_detected"):
        return -SLOW_SPEED, 0, "CLIFF_BACKUP"

    d = corrected_distances(sensors)
    front = d["efront"]
    left = d["left"]
    right = d["right"]
    back = d["back"]
    hint = vision_state["hint"] if VISION_ENABLED else "none"

    if front > FRONT_CLEAR:
        return BASE_SPEED, 0, "FORWARD"
    if front > FRONT_CAUTION:
        return SLOW_SPEED, 0, "FORWARD_SLOW"

    if left > right and left > SIDE_CLEAR:
        robot_angle = (robot_angle - 35) % 360
        return SLOW_SPEED, -TURN_ANGLE, "TURN_LEFT (vision)" if hint == "left" else "TURN_LEFT"
    if right > left and right > SIDE_CLEAR:
        robot_angle = (robot_angle + 35) % 360
        return SLOW_SPEED, TURN_ANGLE, "TURN_RIGHT (vision)" if hint == "right" else "TURN_RIGHT"
    if back > 300:
        return -SLOW_SPEED, 0, "REVERSE"
    return 0, 0, "STUCK"


def decide_search(sensors):
    global search_phase, search_step_counter, robot_angle
    if sensors.get("reflex_active"):
        return 0, 0, "REFLEX_OVERRIDE"
    if sensors.get("cliff_detected"):
        return -SLOW_SPEED, 0, "CLIFF_BACKUP"

    d = corrected_distances(sensors)
    efront = d["efront"]
    left = d["left"]
    right = d["right"]

    # If vision is repeatedly failing, stop rather than blindly wandering.
    if vision_state["vision_timeout_count"] >= SEARCH_MAX_TIMEOUTS:
        return 0, 0, "VISION_DEGRADED_HOLD"

    search_step_counter += 1

    if search_phase == "forward":
        if search_step_counter >= SEARCH_FORWARD_STEPS:
            search_phase = "rotate"
            search_step_counter = 0
            log("Search: switching to ROTATE phase")

        if efront < FRONT_CAUTION:
            if left > right and left > SIDE_CLEAR:
                return SLOW_SPEED, -TURN_ANGLE, "SEARCH_TURN_LEFT_STEP"
            if right > SIDE_CLEAR:
                return SLOW_SPEED, TURN_ANGLE, "SEARCH_TURN_RIGHT_STEP"
            search_phase = "rotate"
            search_step_counter = 0
            return SLOW_SPEED, TURN_ANGLE, "SEARCH_TURN_RIGHT_STEP"

        return SLOW_SPEED, 0, "SEARCH_FORWARD_STEP"

    if search_step_counter >= SEARCH_ROTATE_STEPS:
        search_phase = "forward"
        search_step_counter = 0
        log("Search: switching to FORWARD phase")

    robot_angle = (robot_angle + 35) % 360
    return 0, TURN_ANGLE, "SEARCH_ROTATE_STEP"


def decide_lock(sensors):
    if sensors.get("cliff_detected"):
        return -SLOW_SPEED, 0, "CLIFF_BACKUP"

    where = vision_state.get("target_where", "unknown")

    # Fallback if unknown
    if where == "unknown":
        where = vision_state.get("last_known_where", "center")

    align_angle = int(TURN_ANGLE * 0.5)

    if where == "left":
        return 0, -align_angle, "LOCK_ALIGN_LEFT_STEP"
    if where == "right":
        return 0, align_angle, "LOCK_ALIGN_RIGHT_STEP"
    return 0, 0, "LOCK_WAIT_CENTERED"


def decide_approach(sensors):
    if sensors.get("cliff_detected"):
        return -SLOW_SPEED, 0, "CLIFF_BACKUP"

    d = corrected_distances(sensors)
    efront = d["efront"]
    us_cm = d["us_cm"]
    where = vision_state.get("target_where", "unknown")

    # Fallback if unknown
    if where == "unknown":
        where = vision_state.get("last_known_where", "center")

    # Distance safety must win before vision logic.
    if efront <= APPROACH_HARD_STOP_MM:
        return 0, 0, "GOAL_REACHED"
    if 0 < us_cm <= APPROACH_STOP_CM:
        return 0, 0, "GOAL_REACHED"

    # If vision is timing out during approach, hold safely rather than creeping forever.
    if vision_state["vision_timeout_count"] >= APPROACH_MAX_TIMEOUTS:
        return 0, 0, "APPROACH_HOLD_VISION_TIMEOUT"

    now = time.time()
    vision_age = now - vision_state.get("timestamp", 0)

    # If target is strongly centered, allow tiny forward pulse while a query is pending.
    if vision_state["processing"]:
        if vision_state["target_confirm_count"] >= TARGET_CONFIRM_NEEDED and vision_state["target_center_count"] >= 2:
            return SLOW_SPEED, 0, "APPROACH_FORWARD_STEP"
        return 0, 0, "APPROACH_WAIT_VISION"

    if vision_age > APPROACH_VISION_STALE_SEC:
        return 0, 0, "APPROACH_WAIT_VISION"

    align_angle = int(TURN_ANGLE * 0.5)
    if where == "left":
        return 0, -align_angle, "APPROACH_ALIGN_LEFT_STEP"
    if where == "right":
        return 0, align_angle, "APPROACH_ALIGN_RIGHT_STEP"
    return SLOW_SPEED, 0, "APPROACH_FORWARD_STEP"

# ── Position estimate ──────────────────────────────────────────────────────────
def update_position(speed, angle, dt):
    global robot_x, robot_y, robot_angle
    if speed == 0:
        return
    dist_mm = (speed / 100.0) * 500 * dt
    if abs(angle) > 5:
        robot_angle = (robot_angle + angle * 0.1) % 360
    rad = math.radians(robot_angle)
    robot_x = max(1, min(GRID_SIZE - 2, robot_x + int(dist_mm * math.sin(rad) / GRID_RES)))
    robot_y = max(1, min(GRID_SIZE - 2, robot_y - int(dist_mm * math.cos(rad) / GRID_RES)))


def reset_vision():
    vision_state["target_seen"] = False
    vision_state["target_confirm_count"] = 0
    vision_state["target_center_count"] = 0
    vision_state["target_lost_count"] = 0
    vision_state["target_weak_count"] = 0
    vision_state["vision_timeout_count"] = 0
    vision_state["lock_heading"] = None
    vision_state["target_where"] = "unknown"
    vision_state["last_where"] = "unknown"


def log_decision_transition(decision, sensors, now):
    d = corrected_distances(sensors)
    hdg = vision_state["lock_heading"]
    hdg_text = f"{hdg:.0f}°" if hdg is not None else "None"
    log(
        f"Decision: {last_decision} → {decision} "
        f"(Fraw:{d['front_raw']:.0f} Fcorr:{d['front_corr']:.0f} Eff:{d['efront']:.0f} "
        f"L:{d['left']:.0f} R:{d['right']:.0f} US:{d['us_cm']:.1f}cm "
        f"where={vision_state['target_where']} seen={vision_state['target_seen']} "
        f"confirms={vision_state['target_confirm_count']} center={vision_state['target_center_count']} "
        f"lost={vision_state['target_lost_count']} weak={vision_state['target_weak_count']} "
        f"timeouts={vision_state['vision_timeout_count']} lock_hdg={hdg_text} "
        f"vision_age={now - vision_state['timestamp']:.2f}s)"
    )


def execute_decision(decision, speed, angle):
    prev_vision_ts = vision_state["timestamp"]

    if decision == "SEARCH_ROTATE_STEP":
        pulse_turn("right", SEARCH_ROTATE_PULSE_SEC)
        time.sleep(SETTLE_AFTER_MOVE_SEC)
        wait_for_vision_update(prev_vision_ts)
    elif decision == "SEARCH_TURN_RIGHT_STEP":
        pulse_turn("right", SEARCH_TURN_PULSE_SEC)
        time.sleep(SETTLE_AFTER_MOVE_SEC)
        wait_for_vision_update(prev_vision_ts)
    elif decision == "SEARCH_TURN_LEFT_STEP":
        pulse_turn("left", SEARCH_TURN_PULSE_SEC)
        time.sleep(SETTLE_AFTER_MOVE_SEC)
        wait_for_vision_update(prev_vision_ts)
    elif decision == "SEARCH_FORWARD_STEP":
        pulse_drive(SLOW_SPEED, 0, SEARCH_FORWARD_PULSE_SEC)
        time.sleep(SETTLE_AFTER_MOVE_SEC)
        wait_for_vision_update(prev_vision_ts)
    elif decision == "LOCK_ALIGN_RIGHT_STEP":
        pulse_turn("right", LOCK_ALIGN_PULSE_SEC)
        time.sleep(SETTLE_AFTER_MOVE_SEC)
        wait_for_vision_update(prev_vision_ts)
    elif decision == "LOCK_ALIGN_LEFT_STEP":
        pulse_turn("left", LOCK_ALIGN_PULSE_SEC)
        time.sleep(SETTLE_AFTER_MOVE_SEC)
        wait_for_vision_update(prev_vision_ts)
    elif decision == "LOCK_WAIT_CENTERED":
        send_stop()
        time.sleep(SETTLE_AFTER_MOVE_SEC)
        wait_for_vision_update(prev_vision_ts)
    elif decision == "APPROACH_ALIGN_RIGHT_STEP":
        pulse_turn("right", APPROACH_ALIGN_PULSE_SEC)
        time.sleep(SETTLE_AFTER_MOVE_SEC)
        wait_for_vision_update(prev_vision_ts)
    elif decision == "APPROACH_ALIGN_LEFT_STEP":
        pulse_turn("left", APPROACH_ALIGN_PULSE_SEC)
        time.sleep(SETTLE_AFTER_MOVE_SEC)
        wait_for_vision_update(prev_vision_ts)
    elif decision == "APPROACH_FORWARD_STEP":
        pulse_drive(min(SLOW_SPEED, 15), 0, APPROACH_FORWARD_PULSE_SEC)
        time.sleep(SETTLE_AFTER_MOVE_SEC)
        wait_for_vision_update(prev_vision_ts)
    elif decision in ("APPROACH_WAIT_VISION", "APPROACH_HOLD_VISION_TIMEOUT", "VISION_DEGRADED_HOLD"):
        send_stop()
        time.sleep(SETTLE_AFTER_MOVE_SEC)
    elif decision == "GOAL_REACHED":
        send_stop()
    else:
        send_drive(speed, angle)

# ── Main loop ──────────────────────────────────────────────────────────────────
def main():
    global last_decision, running, search_phase, search_step_counter, goal_reached

    log("PiCar Navigator starting...")
    log(f"Agent:    {AGENT_URL}")
    log(f"Camera:   {CAMERA_URL}")
    log(f"Ollama:   {OLLAMA_URL}")
    log(f"Model:    {OLLAMA_MODEL}")
    log(f"Vision:   {'ENABLED' if VISION_ENABLED else 'DISABLED'} search={VISION_INTERVAL}s lock={LOCKING_VISION_INTERVAL}s approach={APPROACH_VISION_INTERVAL}s timeout={VISION_TIMEOUT}s")
    log(f"Confirm:  {TARGET_CONFIRM_NEEDED} WHERE-known detections needed, lost=search:{TARGET_LOST_TOLERANCE}/lock:{TARGET_LOST_TOLERANCE_LOCKING}/approach:{TARGET_LOST_TOLERANCE_APPROACH}")
    log(f"Approach: stop at {APPROACH_STOP_CM}cm, hard stop at {APPROACH_HARD_STOP_MM}mm effective front")
    log(f"LiDAR:    front offset correction={LIDAR_FRONT_OFFSET_MM}mm")
    log("Control: pulse-and-settle mode enabled for search, lock, and approach")
    log(f"Speed:    base={BASE_SPEED}, slow={SLOW_SPEED}, turn={TURN_ANGLE}°, option_a={OPTION_A_TURN_SPEED}")
    log("Waiting for AUTONOMOUS mode...")

    map_save_counter = 0
    last_vision_time = 0.0

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
                    goal_reached = False
                    search_phase = "forward"
                    search_step_counter = 0
                    reset_vision()
                send_stop()
                time.sleep(0.5)
                continue

            task_data = get_task()
            task = task_data.get("task", "") if task_data else ""
            task_status = task_data.get("status", "") if task_data else ""

            if not task and goal_reached:
                goal_reached = False
                search_phase = "forward"
                search_step_counter = 0
                reset_vision()

            if goal_reached or task_status == "GOAL_REACHED":
                if last_decision != "GOAL_REACHED":
                    log("Task complete — waiting for new task.")
                    last_decision = "GOAL_REACHED"
                send_stop()
                time.sleep(0.5)
                continue

            sensors = get_sensors()
            if sensors is None:
                time.sleep(POLL_INTERVAL)
                continue

            scan = get_lidar()
            if scan:
                update_map(scan, robot_x, robot_y, robot_angle)

            now = time.time()
            confirms = vision_state["target_confirm_count"]
            seen = vision_state["target_seen"]
            if seen:
                interval = APPROACH_VISION_INTERVAL
            elif confirms > 0 or vision_state["target_weak_count"] > 0:
                interval = LOCKING_VISION_INTERVAL
            else:
                interval = VISION_INTERVAL

            if VISION_ENABLED and not vision_state["processing"] and (now - last_vision_time >= interval):
                if task:
                    query_vision_goal(task)
                else:
                    query_vision_navigation()
                last_vision_time = now

            if not task:
                speed, angle, decision = decide_navigate(sensors)
            elif vision_state["target_seen"] and vision_state["target_center_count"] >= 2:
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
            elif (
                (vision_state["target_confirm_count"] > 0 or vision_state["target_weak_count"] >= 2)
                and vision_state.get("target_where") in ("left", "center", "right")
            ):
                speed, angle, decision = decide_lock(sensors)
                post_task_status("LOCKING")
            else:
                speed, angle, decision = decide_search(sensors)
                post_task_status("SEARCHING")

            if decision != last_decision:
                log_decision_transition(decision, sensors, now)
                last_decision = decision
                post_decision(decision)

            if not goal_reached:
                execute_decision(decision, speed, angle)

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

        elapsed = time.time() - loop_start
        time.sleep(max(0, POLL_INTERVAL - elapsed))

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

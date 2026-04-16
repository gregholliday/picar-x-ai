#!/usr/bin/env python3

# ── Load config ───────────────────────────────────────────────────────────────
import sys
import os

# Look for config.py one directory up (repo root)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from config import (
        PI_IP, PI_USERNAME, CAMERA_PORT, AGENT_PORT,
        LIDAR_PORT, LIDAR_BAUDRATE,
        ULTRASONIC_STOP, ULTRASONIC_SLOW,
        CLIFF_STOP, CLIFF_WARN,
        STEERING_TRIM,
    )
    print("Config loaded from config.py")
except ImportError:
    print("config.py not found — using defaults. Copy config.py to repo root and edit it.")
    PI_IP            = "YOUR_PI_IP"
    PI_USERNAME      = "pi"
    CAMERA_PORT      = 9000
    AGENT_PORT       = 8000
    LIDAR_PORT       = "/dev/ttyUSB0"
    LIDAR_BAUDRATE   = 460800
    ULTRASONIC_STOP  = 15
    ULTRASONIC_SLOW  = 30
    CLIFF_STOP       = 100
    CLIFF_WARN       = 500
    STEERING_TRIM    = 0

# Patch os.getlogin for systemd service compatibility
os.getlogin = lambda: PI_USERNAME

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from picarx import Picarx
from picarx.music import Music
from vilib import Vilib
from rplidarc1 import RPLidar as RPLidarC1
from robot_hat import ADC
import asyncio
import subprocess
import threading
import signal
import time

# ── Reset MCU (prevents servo bouncing on startup) ────────────────────────────
try:
    from robot_hat import reset_mcu
    reset_mcu()
    time.sleep(0.2)
    print("MCU reset complete.")
except Exception as e:
    print(f"MCU reset skipped: {e}")

# ── App setup ─────────────────────────────────────────────────────────────────
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

px = Picarx()

# ── Sound ─────────────────────────────────────────────────────────────────────
try:
    music = Music()
    music.music_set_volume(80)
    SOUND_HORN    = '/mnt/ai/picar-x/sounds/car-double-horn.wav'
    SOUND_ENGINE  = '/mnt/ai/picar-x/sounds/car-start-engine.wav'
    print("Sound system initialized.")
except Exception as e:
    music = None
    print(f"Sound system not available: {e}")

# ── Battery monitor ────────────────────────────────────────────────────────────
try:
    battery_adc = ADC('A4')
    print("Battery monitor initialized.")
except Exception as e:
    battery_adc = None
    print(f"Battery monitor not available: {e}")

# ── LiDAR (optional — starts gracefully if not connected) ─────────────────────
lidar = None
try:
    lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUDRATE)
    print(f"LiDAR C1 connected on {LIDAR_PORT}.")
except Exception as e:
    print(f"LiDAR not available: {e}")
    print("Starting without LiDAR — connect sensor to enable.")

# ── Constants ──────────────────────────────────────────────────────────────────
MAX_DIST_MM = 6000  # mm — LiDAR clip distance

# ── Shared state ───────────────────────────────────────────────────────────────
state = {
    "mode":           "manual",
    "speed":          0,
    "angle":          0,
    "lidar_scan":     [],
    "lidar_ok":       lidar is not None,
    "ultrasonic_cm":  0.0,
    "grayscale":      [0, 0, 0],
    "cliff_detected": False,
    "obstacle_close": False,
    "reflex_active":  False,
    "battery_v": 0.0,
    "battery_pct": 0,
    "task":               "",
    "task_status":        "IDLE",
    "task_found":         False,
    "vision_description": "",
    "vision_hint":        "none",       
    "navigator_decision": "IDLE",
    "navigator_log":      []   
}

# ── Cleanup handler ────────────────────────────────────────────────────────────
def cleanup(sig=None, frame=None):
    print("\nCleaning up...")
    state["mode"] = "manual"
    try:
        px.stop()
        px.set_dir_servo_angle(0)
        print("PiCar stopped.")
    except Exception as e:
        print(f"PiCar cleanup error: {e}")
    try:
        if lidar is not None:
            lidar.reset()
            print("LiDAR disconnected.")
    except Exception as e:
        print(f"LiDAR cleanup error: {e}")
    try:
        Vilib.camera_close()
        print("Camera closed.")
    except Exception as e:
        print(f"Camera cleanup error: {e}")
    print("Cleanup complete.")
    exit(0)

signal.signal(signal.SIGINT,  cleanup)
signal.signal(signal.SIGTERM, cleanup)

# ── Start Vilib camera ─────────────────────────────────────────────────────────
print("Starting camera...")
Vilib.camera_start(vflip=False, hflip=False)
Vilib.display(local=False, web=True)
time.sleep(5)
# Fix OV5647 pink/red colour cast
# Read what AWB settled on
#controls = Vilib.get_controls()
#awb_gains = controls.get('ColourGains', (1.4, 1.6))
#print(f"AWB settled on: {awb_gains}")

# Lock those gains so they don't drift
#Vilib.set_controls({
#    "ColourGains": awb_gains,
#    "AnalogueGain": 2.0
#})
#print("Camera colour locked.")
print(f"Camera ready. Stream at http://{PI_IP}:{CAMERA_PORT}/mjpg")

# ── Sensor polling thread (10Hz) ───────────────────────────────────────────────
def sensor_worker():
    print("Sensor worker started.")
    while True:
        try:
            dist = px.ultrasonic.read()
            if dist is not None and dist > 2:
                state["ultrasonic_cm"] = round(float(dist), 1)

            gs = px.grayscale.read()
            if gs:
                state["grayscale"] = gs

            valid_gs = [v for v in state["grayscale"] if v > 0]  # filter zero readings
            state["cliff_detected"] = sum(v < CLIFF_STOP for v in valid_gs) >= 2
            state["obstacle_close"] = 0 < state["ultrasonic_cm"] < ULTRASONIC_STOP

        except Exception as e:
            print(f"Sensor read error: {e}")

        if battery_adc is not None:
            try:
                raw = battery_adc.read_voltage()
                volts = round(raw * 3, 2)
                # 2S LiPo: 8.4V=100%, 6.8V=0%
                pct = int(max(0, min(100, (volts - 6.8) / (8.4 - 6.8) * 100)))
                state["battery_v"]   = volts
                state["battery_pct"] = pct
            except Exception:
                pass
        time.sleep(0.1)

threading.Thread(target=sensor_worker, daemon=True).start()

# ── Reflex safety loop (10Hz) ──────────────────────────────────────────────────
def reflex_worker():
    print("Reflex safety loop started.")
    while True:
        try:
            if state["mode"] == "autonomous":
                cliff  = state["cliff_detected"]
                us_cm  = state["ultrasonic_cm"]
                lidar_front = min(
                    (m["distance"] for m in state["lidar_scan"] 
                    if 315 <= m["angle"] <= 360 or 0 <= m["angle"] <= 45),
                    default=9999
                )

                if cliff:
                    px.stop()
                    state["speed"] = 0
                    state["reflex_active"] = True
                    print(f"REFLEX: Cliff detected! GS={state['grayscale']}")
                    time.sleep(0.3)
                    px.backward(30)
                    time.sleep(0.5)
                    px.stop()
                elif lidar_front < 300 and state["speed"] > 0:
                    px.forward(20)
                    #state["reflex_active"] = True
                    state["speed"] = 20
                    print(f"REFLEX: LiDAR front {lidar_front}mm — slowing to 20.")
                elif 0 < us_cm < ULTRASONIC_STOP:
                    px.stop()
                    state["speed"] = 0
                    state["reflex_active"] = True
                    print(f"REFLEX: Obstacle at {us_cm}cm — stopping.")

                elif 0 < us_cm < ULTRASONIC_SLOW:
                    if state["speed"] > 0:
                        reduced = min(state["speed"], 30)
                        px.forward(reduced)
                        state["reflex_active"] = True
                        print(f"REFLEX: Near obstacle at {us_cm}cm — slowing to {reduced}.")

                else:
                    state["reflex_active"] = False

        except Exception as e:
            print(f"Reflex error: {e}")

        time.sleep(0.1)

threading.Thread(target=reflex_worker, daemon=True).start()

# ── LiDAR background thread (rplidarc1 async API) ─────────────────────────────
def lidar_worker():
    if lidar is None:
        print("LiDAR worker skipped — no sensor connected.")
        return

    async def scan_loop():
        print("LiDAR C1 scan loop starting...")

        async def process_queue(q, stop):
            current_scan = []
            last_angle   = -1

            while not stop.is_set():
                try:
                    item = q.get_nowait()
                    angle = item.get('a_deg')
                    dist  = item.get('d_mm')

                    if angle is None or dist is None:
                        continue
                    if dist <= 0 or dist > MAX_DIST_MM:
                        continue

                    # Detect scan boundary by angle wraparound
                    if last_angle > 300 and angle < 60 and current_scan:
                        state["lidar_scan"] = current_scan
                        state["lidar_ok"]   = True
                        current_scan = []

                    current_scan.append({
                        "angle":    round(angle, 1),
                        "distance": round(dist, 1)
                    })
                    last_angle = angle

                except Exception:
                    await asyncio.sleep(0.001)

        async with asyncio.TaskGroup() as tg:
            tg.create_task(lidar.simple_scan())
            tg.create_task(process_queue(lidar.output_queue, lidar.stop_event))

    while True:
        try:
            print("Initializing LiDAR C1...")
            try:
                lidar.reset()
            except Exception:
                pass
            time.sleep(3)
            asyncio.run(scan_loop())
        except Exception as e:
            print(f"LiDAR error: {e}, retrying in 3s...")
            state["lidar_ok"] = False
            try:
                lidar.reset()
            except Exception:
                pass
            time.sleep(3)

threading.Thread(target=lidar_worker, daemon=True).start()

# ── Helper: safe drive (respects reflex overrides) ────────────────────────────
def safe_drive(speed: int, angle: int):
    """Apply drive command only if reflex loop is not actively blocking."""
    if state["reflex_active"]:
        return
    speed = max(-100, min(100, speed))
    angle = max(-40,  min(40,  angle + STEERING_TRIM))
    state["speed"] = speed
    state["angle"] = angle
    px.set_dir_servo_angle(angle)
    if speed > 0:
        px.forward(speed)
    elif speed < 0:
        px.backward(abs(speed))
    else:
        px.stop()

# ── Motor helpers (corrected for Motor 2 inverted wiring) ────────────────────
def left_motor(speed: int):
    """Left motor: positive=forward, negative=backward."""
    px.set_motor_speed(1, speed)

def right_motor(speed: int):
    """Right motor: inverted wiring — negate speed for correct direction."""
    px.set_motor_speed(2, -speed)

def option_a_turn(direction: str, speed: int, angle: int):
    """
    Option A turn: front wheels steer + outside wheel drives + inside wheel stops.
    direction: 'left' or 'right'
    speed: outside wheel speed
    angle: front wheel steering angle (positive value, sign applied internally)
    """
    if direction == "right":
        px.set_dir_servo_angle(abs(angle))
        left_motor(speed)    # outside wheel drives
        right_motor(0)       # inside wheel stops
    elif direction == "left":
        px.set_dir_servo_angle(-abs(angle))
        left_motor(0)        # inside wheel stops
        right_motor(speed)   # outside wheel drives

# ── Routes ─────────────────────────────────────────────────────────────────────
@app.get("/api/status")
def get_status():
    return {
        "mode":           state["mode"],
        "speed":          state["speed"],
        "angle":          state["angle"],
        "lidar_ok":       state["lidar_ok"],
        "lidar_points":   len(state["lidar_scan"]),
        "stream_url":     f"http://{PI_IP}:{CAMERA_PORT}/mjpg",
        "cliff_detected": state["cliff_detected"],
        "obstacle_close": state["obstacle_close"],
        "reflex_active":  state["reflex_active"],
        "battery_v":          state["battery_v"],
        "battery_pct":        state["battery_pct"],
        "task":               state["task"],
        "task_status":        state["task_status"],
        "task_found":         state["task_found"],
        "vision_description": state["vision_description"],
        "vision_hint":        state["vision_hint"],
        "navigator_decision": state["navigator_decision"],
        "navigator_log":      state["navigator_log"],
    }

@app.get("/api/sensors")
def get_sensors():
    """Raw sensor readings for the navigator on Fedora."""
    scan = state["lidar_scan"]

    front = [m for m in scan if 315 <= m["angle"] <= 360 or 0  <= m["angle"] <= 45]
    left  = [m for m in scan if 225 <  m["angle"] <  315]
    back  = [m for m in scan if 135 <  m["angle"] <= 225]
    right = [m for m in scan if 45  <  m["angle"] <= 135]

    def closest(measures):
        return round(min((m["distance"] for m in measures), default=0), 1)

    return {
        "ultrasonic_cm":  state["ultrasonic_cm"],
        "grayscale":      state["grayscale"],
        "cliff_detected": state["cliff_detected"],
        "obstacle_close": state["obstacle_close"],
        "reflex_active":  state["reflex_active"],
        "lidar": {
            "points": len(scan),
            "front":  closest(front),
            "left":   closest(left),
            "back":   closest(back),
            "right":  closest(right),
        }
    }

@app.get("/api/battery")
def get_battery():
    v   = state["battery_v"]
    pct = state["battery_pct"]
    if v < 6.8:
        status = "critical"
    elif v < 7.0:
        status = "low"
    elif v < 7.4:
        status = "medium"
    else:
        status = "good"
    return {
        "voltage":    v,
        "percent":    pct,
        "status":     status
    }

@app.get("/api/lidar")
def get_lidar():
    return {"scan": state["lidar_scan"]}

@app.get("/api/lidar/summary")
def get_lidar_summary():
    scan = state["lidar_scan"]
    if not scan:
        return {"status": "no_data", "points": 0}

    front = [m for m in scan if 315 <= m["angle"] <= 360 or 0  <= m["angle"] <= 45]
    left  = [m for m in scan if 225 <  m["angle"] <  315]
    back  = [m for m in scan if 135 <  m["angle"] <= 225]
    right = [m for m in scan if 45  <  m["angle"] <= 135]

    def closest(measures):
        return round(min((m["distance"] for m in measures), default=0), 1)

    return {
        "status": "ok",
        "points": len(scan),
        "closest": {
            "front": closest(front),
            "left":  closest(left),
            "back":  closest(back),
            "right": closest(right)
        }
    }

@app.post("/api/mode/{mode}")
def set_mode(mode: str):
    if mode in ["manual", "autonomous"]:
        state["mode"] = mode
        if mode == "manual":
            px.stop()
            px.set_dir_servo_angle(0)
            state["speed"] = 0
            state["angle"] = 0
            state["reflex_active"] = False
            print("Mode set to MANUAL.")
        else:
            print("Mode set to AUTONOMOUS.")
    return {"mode": state["mode"]}

@app.post("/api/drive")
def drive(speed: int = 0, angle: int = 0):
    if state["mode"] == "manual":
        speed = max(-100, min(100, speed))
        angle = max(-40,  min(40,  angle + STEERING_TRIM))
        state["speed"] = speed
        state["angle"] = angle
        px.set_dir_servo_angle(angle)
        if speed > 0:
            px.forward(speed)
        elif speed < 0:
            px.backward(abs(speed))
        else:
            px.stop()
    elif state["mode"] == "autonomous":
        safe_drive(speed, angle)
    return {"speed": state["speed"], "angle": state["angle"]}

@app.post("/api/turn")
def turn(direction: str = "right", speed: int = 20, angle: int = 35):
    """
    Option A turn: front wheels steer + outside wheel drives + inside wheel stops.
    direction: 'left' or 'right'
    speed: outside wheel speed (default 20)
    angle: front wheel steering angle (default 35)
    Only works in autonomous mode.
    """
    if state["mode"] == "autonomous" and not state["reflex_active"]:
        option_a_turn(direction, speed, angle)
        state["speed"] = speed
        state["angle"] = angle if direction == "right" else -angle
    return {"direction": direction, "speed": speed, "angle": angle}

@app.post("/api/stop")
def stop():
    px.stop()
    px.set_dir_servo_angle(0)
    state["speed"] = 0
    state["angle"] = 0
    return {"status": "stopped"}

@app.post("/api/shutdown")
def shutdown():
    px.stop()
    Vilib.camera_close()
    subprocess.Popen(['sudo', 'shutdown', 'now'])
    return {"status": "shutting_down"}

# ── Vision endpoints ───────────────────────────────────────────────────────────
@app.post("/api/vision/update")
def update_vision(description: str = "", hint: str = "none"):
    state["vision_description"] = description
    state["vision_hint"]        = hint
    return {"status": "ok"}

@app.get("/api/vision")
def get_vision():
    return {
        "description": state["vision_description"],
        "hint":        state["vision_hint"],
    }

# Add new endpoint
@app.post("/api/navigator/decision")
def post_navigator_decision(decision: str = "", sensors: dict = None):
    state["navigator_decision"] = decision
    state["navigator_log"].append({
        "time": time.strftime("%H:%M:%S"),
        "decision": decision
    })
    # Keep last 20 decisions
    if len(state["navigator_log"]) > 20:
        state["navigator_log"] = state["navigator_log"][-20:]
    return {"status": "ok"}

@app.get("/api/navigator/status")
def get_navigator_status():
    return {
        "decision": state["navigator_decision"],
        "log": state["navigator_log"]
    }

@app.post("/api/buzzer")
def buzzer(sound: str = "horn"):
    def play():
        try:
            if music is None:
                return
            if sound == "horn":
                music.sound_play_threading(SOUND_HORN)
            elif sound == "engine":
                music.sound_play_threading(SOUND_ENGINE)
        except Exception as e:
            print(f"Sound error: {e}")
    threading.Thread(target=play, daemon=True).start()
    return {"status": "ok"}

# ── Task endpoints ─────────────────────────────────────────────────────────────
@app.post("/api/task")
def set_task(task: str = ""):
    state["task"]        = task
    state["task_status"] = "SEARCHING" if task else "IDLE"
    state["task_found"]  = False
    print(f"Task set: '{task}'")
    return {"task": state["task"], "status": state["task_status"]}

@app.get("/api/task")
def get_task():
    return {
        "task":   state["task"],
        "status": state["task_status"],
        "found":  state["task_found"],
    }

@app.post("/api/task/found")
def task_found():
    state["task_status"] = "GOAL_REACHED"
    state["task_found"]  = True
    print(f"Task complete: '{state['task']}'")
    return {"status": "GOAL_REACHED"}

@app.post("/api/task/status")
def set_task_status(status: str = "SEARCHING"):
    state["task_status"] = status
    return {"status": state["task_status"]}

# ── Main ───────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import uvicorn
    print("Starting PiCar agent v7 on port 8000...")
    uvicorn.run(app, host="0.0.0.0", port=8000)

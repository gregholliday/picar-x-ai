#!/usr/bin/env python3

# Patch os.getlogin for systemd service compatibility
import os
os.getlogin = lambda: "YOUR_USERNAME"

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from picarx import Picarx
from vilib import Vilib
from rplidar import RPLidar
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

# ── LiDAR (optional — starts gracefully if not connected) ─────────────────────
lidar = None
try:
    lidar = RPLidar('/dev/ttyUSB0', baudrate=460800, timeout=3)
    print("LiDAR connected on /dev/ttyUSB0.")
except Exception as e:
    print(f"LiDAR not available: {e}")
    print("Starting without LiDAR — connect sensor to enable.")

# ── Thresholds ─────────────────────────────────────────────────────────────────
ULTRASONIC_STOP    = 15    # cm  — emergency stop
ULTRASONIC_SLOW    = 30    # cm  — slow down
CLIFF_STOP         = 100   # ADC — cliff detected, stop immediately
CLIFF_WARN         = 500   # ADC — near edge, slow down
MAX_DIST_MM        = 6000  # mm  — LiDAR clip distance
STEERING_TRIM      = -2    # degrees — negative = left correction. Adjust until car goes straght

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
    "reflex_active":  False,   # True when reflex loop overrode a command
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
            lidar.stop()
            lidar.stop_motor()
            lidar.clean_input()
            lidar.disconnect()
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
time.sleep(2)
print("Camera ready. Stream at http://YOUR_PI_IP:9000/mjpg")

# ── Sensor polling thread (10Hz) ───────────────────────────────────────────────
def sensor_worker():
    print("Sensor worker started.")
    while True:
        try:
            # Read ultrasonic
            dist = px.ultrasonic.read()
            if dist is not None and dist > 0:
                state["ultrasonic_cm"] = round(float(dist), 1)

            # Read grayscale
            gs = px.grayscale.read()
            if gs:
                state["grayscale"] = gs

            # Derive cliff and obstacle flags
            state["cliff_detected"] = any(v < CLIFF_STOP for v in state["grayscale"])
            state["obstacle_close"] = state["ultrasonic_cm"] < ULTRASONIC_STOP

        except Exception as e:
            print(f"Sensor read error: {e}")

        time.sleep(0.1)  # 10Hz

threading.Thread(target=sensor_worker, daemon=True).start()

# ── Reflex safety loop (10Hz) ──────────────────────────────────────────────────
# Runs independently of the navigator — overrides dangerous commands locally
def reflex_worker():
    print("Reflex safety loop started.")
    while True:
        try:
            if state["mode"] == "autonomous":
                cliff    = state["cliff_detected"]
                obstacle = state["obstacle_close"]
                us_cm    = state["ultrasonic_cm"]

                if cliff:
                    # Cliff detected — stop and back up slightly
                    px.stop()
                    state["speed"] = 0
                    state["reflex_active"] = True
                    print(f"REFLEX: Cliff detected! GS={state['grayscale']}")
                    time.sleep(0.3)
                    px.backward(30)
                    time.sleep(0.5)
                    px.stop()

                elif us_cm < ULTRASONIC_STOP and us_cm > 0:
                    # Obstacle too close — stop immediately
                    px.stop()
                    state["speed"] = 0
                    state["reflex_active"] = True
                    print(f"REFLEX: Obstacle at {us_cm}cm — stopping.")

                elif us_cm < ULTRASONIC_SLOW and us_cm > 0:
                    # Getting close — reduce speed if going forward
                    if state["speed"] > 0:
                        reduced = min(state["speed"], 30)
                        px.forward(reduced)
                        state["reflex_active"] = True
                        print(f"REFLEX: Near obstacle at {us_cm}cm — slowing to {reduced}.")

                else:
                    state["reflex_active"] = False

        except Exception as e:
            print(f"Reflex error: {e}")

        time.sleep(0.1)  # 10Hz

threading.Thread(target=reflex_worker, daemon=True).start()

# ── LiDAR background thread ────────────────────────────────────────────────────
def lidar_worker():
    if lidar is None:
        print("LiDAR worker skipped — no sensor connected.")
        return

    while True:
        try:
            print("Initializing LiDAR...")
            lidar.stop()
            lidar.stop_motor()
            time.sleep(2)
            lidar.clean_input()
            time.sleep(1)
            lidar.start_motor()
            time.sleep(3)
            print("LiDAR ready, starting scan...")
            state["lidar_ok"] = True

            current_scan = []
            for measure in lidar.iter_measures(scan_type='normal', max_buf_meas=5000):
                new_scan, quality, angle, distance = measure

                if new_scan and current_scan:
                    state["lidar_scan"] = current_scan
                    current_scan = []

                if distance > 0:
                    current_scan.append({
                        "angle":    round(angle, 1),
                        "distance": round(distance, 1)
                    })

        except Exception as e:
            print(f"LiDAR error: {e}, retrying in 3s...")
            state["lidar_ok"] = False
            try:
                lidar.stop()
                lidar.stop_motor()
            except Exception:
                pass
            time.sleep(3)

threading.Thread(target=lidar_worker, daemon=True).start()

# ── Helper: safe drive (respects reflex overrides) ────────────────────────────
def safe_drive(speed: int, angle: int):
    """Apply drive command only if reflex loop isn't actively blocking."""
    if state["reflex_active"]:
        return  # Reflex has control, ignore navigator command
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

# ── Routes ─────────────────────────────────────────────────────────────────────
@app.get("/api/status")
def get_status():
    return {
        "mode":           state["mode"],
        "speed":          state["speed"],
        "angle":          state["angle"],
        "lidar_ok":       state["lidar_ok"],
        "lidar_points":   len(state["lidar_scan"]),
        "stream_url":     "http://YOUR_PI_IP:9000/mjpg",
        "cliff_detected": state["cliff_detected"],
        "obstacle_close": state["obstacle_close"],
        "reflex_active":  state["reflex_active"],
    }

@app.get("/api/sensors")
def get_sensors():
    """Raw sensor readings for the navigator on Fedora."""
    scan = state["lidar_scan"]

    # Pre-compute quadrant minimums for navigator convenience
    front = [m for m in scan if 315 <= m["angle"] <= 360 or 0  <= m["angle"] <= 45]
    left  = [m for m in scan if 45  <  m["angle"] <= 135]
    back  = [m for m in scan if 135 <  m["angle"] <= 225]
    right = [m for m in scan if 225 <  m["angle"] <  315]

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

@app.get("/api/lidar")
def get_lidar():
    return {"scan": state["lidar_scan"]}

@app.get("/api/lidar/summary")
def get_lidar_summary():
    scan = state["lidar_scan"]
    if not scan:
        return {"status": "no_data", "points": 0}

    front = [m for m in scan if 315 <= m["angle"] <= 360 or 0  <= m["angle"] <= 45]
    left  = [m for m in scan if 45  <  m["angle"] <= 135]
    back  = [m for m in scan if 135 <  m["angle"] <= 225]
    right = [m for m in scan if 225 <  m["angle"] <  315]

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
            print(f"Mode set to MANUAL.")
        else:
            print(f"Mode set to AUTONOMOUS.")
    return {"mode": state["mode"]}

@app.post("/api/drive")
def drive(speed: int = 0, angle: int = 0):
    if state["mode"] == "manual":
        # Manual mode — direct control, bypass safe_drive
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
        # Autonomous mode — go through safe_drive (respects reflex)
        safe_drive(speed, angle)
    return {"speed": state["speed"], "angle": state["angle"]}

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

# ── Main ───────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import uvicorn
    print("Starting PiCar agent v7 on port 8000...")
    uvicorn.run(app, host="0.0.0.0", port=8000)

#!/usr/bin/env python3
"""
PiCar-X Navigator
Runs on Fedora. Polls the Pi agent for sensor data and makes
autonomous driving decisions. The Pi's reflex loop handles
emergency stops independently of this navigator.

Usage:
    python3 picar_navigator.py

    The PiCar must be in AUTONOMOUS mode via the dashboard
    before the navigator will send any drive commands.
"""

import requests
import time
import math
import json
from datetime import datetime
from collections import deque

# ── Config ─────────────────────────────────────────────────────────────────────
AGENT_URL     = "http://YOUR_PI_IP:8000"
POLL_INTERVAL = 0.4    # seconds between navigator decisions
BASE_SPEED    = 40     # normal cruising speed
SLOW_SPEED    = 25     # speed when navigating around obstacles
TURN_ANGLE    = 35     # degrees for turns

# Distance thresholds (mm) for navigation decisions
# Note: Pi reflex loop handles emergency stops below these values
FRONT_CLEAR   = 600    # mm — path is clear, drive forward
FRONT_CAUTION = 350    # mm — slow down, start planning turn
SIDE_CLEAR    = 300    # mm — side is clear enough to turn into

# ── Room map ───────────────────────────────────────────────────────────────────
# Simple occupancy grid — tracks where obstacles have been seen
# Grid is centered on starting position, each cell = 100mm
GRID_SIZE   = 100   # cells (100x100 = 10m x 10m)
GRID_RES    = 100   # mm per cell
grid        = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]
robot_x     = GRID_SIZE // 2   # robot starts in center
robot_y     = GRID_SIZE // 2
robot_angle = 0.0              # degrees, 0 = facing up/north

# ── State ──────────────────────────────────────────────────────────────────────
running       = True
last_decision = "IDLE"
decision_log  = deque(maxlen=20)  # rolling log of last 20 decisions

# ── Helpers ────────────────────────────────────────────────────────────────────
def log(msg):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    line = f"[{ts}] {msg}"
    print(line)
    decision_log.append(line)

def get_sensors():
    """Fetch sensor data from the Pi agent."""
    try:
        r = requests.get(f"{AGENT_URL}/api/sensors", timeout=1.0)
        return r.json()
    except Exception as e:
        log(f"Sensor fetch failed: {e}")
        return None

def get_status():
    """Fetch agent status."""
    try:
        r = requests.get(f"{AGENT_URL}/api/status", timeout=1.0)
        return r.json()
    except Exception as e:
        log(f"Status fetch failed: {e}")
        return None

def get_lidar():
    """Fetch full LiDAR scan for mapping."""
    try:
        r = requests.get(f"{AGENT_URL}/api/lidar", timeout=1.0)
        return r.json().get("scan", [])
    except Exception as e:
        log(f"LiDAR fetch failed: {e}")
        return []

def send_drive(speed: int, angle: int):
    """Send drive command to Pi agent."""
    try:
        requests.post(
            f"{AGENT_URL}/api/drive",
            params={"speed": speed, "angle": angle},
            timeout=0.5
        )
    except Exception as e:
        log(f"Drive command failed: {e}")

def send_stop():
    """Send stop command."""
    try:
        requests.post(f"{AGENT_URL}/api/stop", timeout=0.5)
    except Exception as e:
        log(f"Stop command failed: {e}")

# ── Room mapping ───────────────────────────────────────────────────────────────
def update_map(scan, rx, ry, heading_deg):
    """
    Update occupancy grid from LiDAR scan.
    rx, ry: robot position in grid cells
    heading_deg: robot heading in degrees (0=up)
    """
    for point in scan:
        angle_deg = point["angle"]
        dist_mm   = point["distance"]

        if dist_mm <= 0 or dist_mm > 5000:
            continue

        # Convert to grid coordinates
        # LiDAR 0° = forward, angles increase clockwise
        world_angle = math.radians(heading_deg + angle_deg)
        dx = dist_mm * math.sin(world_angle) / GRID_RES
        dy = dist_mm * math.cos(world_angle) / GRID_RES

        gx = int(rx + dx)
        gy = int(ry - dy)  # y axis inverted in grid

        if 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE:
            grid[gy][gx] = min(grid[gy][gx] + 1, 10)  # increment, cap at 10

def save_map():
    """Save the current occupancy grid to a JSON file."""
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

def print_map_ascii():
    """Print a small ASCII representation of the map around the robot."""
    size = 20  # cells to show around robot
    print("\n── ROOM MAP ──────────────────────────")
    for dy in range(-size//2, size//2):
        row = ""
        for dx in range(-size//2, size//2):
            gx = robot_x + dx
            gy = robot_y + dy
            if gx == robot_x and gy == robot_y:
                row += "R"  # robot position
            elif 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE:
                v = grid[gy][gx]
                if v == 0:
                    row += "·"
                elif v < 3:
                    row += "░"
                elif v < 7:
                    row += "▒"
                else:
                    row += "█"
            else:
                row += " "
        print(f"  {row}")
    print("──────────────────────────────────────\n")

# ── Navigation decisions ───────────────────────────────────────────────────────
def decide(sensors):
    """
    Make a driving decision based on sensor data.
    Returns (speed, angle, decision_name)
    """
    global robot_angle

    # If reflex loop is active, don't fight it
    if sensors.get("reflex_active"):
        return 0, 0, "REFLEX_OVERRIDE"

    # If cliff detected, back up (belt and suspenders with reflex loop)
    if sensors.get("cliff_detected"):
        return -20, 0, "CLIFF_BACKUP"

    lidar   = sensors.get("lidar", {})
    front   = lidar.get("front", 9999)
    left    = lidar.get("left",  9999)
    right   = lidar.get("right", 9999)
    back    = lidar.get("back",  9999)
    us_cm   = sensors.get("ultrasonic_cm", 999)

    # Use ultrasonic as authoritative forward distance if closer than LiDAR front
    us_mm = us_cm * 10
    effective_front = min(front, us_mm) if us_mm > 0 else front

    # ── Decision tree ──────────────────────────────────────────────────────────

    # Path is clear ahead — drive forward
    if effective_front > FRONT_CLEAR:
        robot_angle = (robot_angle + 0) % 360
        return BASE_SPEED, 0, "FORWARD"

    # Getting close — slow down but keep going if still ok
    if effective_front > FRONT_CAUTION:
        return SLOW_SPEED, 0, "FORWARD_SLOW"

    # Obstacle ahead — need to turn
    # Choose direction based on which side has more space
    if left > right and left > SIDE_CLEAR:
        robot_angle = (robot_angle - 35) % 360
        return SLOW_SPEED, -TURN_ANGLE, "TURN_LEFT"

    elif right > left and right > SIDE_CLEAR:
        robot_angle = (robot_angle + 35) % 360
        return SLOW_SPEED, TURN_ANGLE, "TURN_RIGHT"

    elif left > SIDE_CLEAR:
        robot_angle = (robot_angle - 35) % 360
        return SLOW_SPEED, -TURN_ANGLE, "TURN_LEFT"

    elif right > SIDE_CLEAR:
        robot_angle = (robot_angle + 35) % 360
        return SLOW_SPEED, TURN_ANGLE, "TURN_RIGHT"

    # Blocked on front and both sides — back up
    elif back > 300:
        return -SLOW_SPEED, 0, "REVERSE"

    # Completely stuck — stop and wait
    else:
        return 0, 0, "STUCK"

# ── Update robot position estimate ─────────────────────────────────────────────
def update_position(speed, angle, dt):
    """Dead reckoning — rough position estimate based on commands sent."""
    global robot_x, robot_y, robot_angle

    if speed == 0:
        return

    # Estimate distance traveled this tick
    # PiCar max speed ~0.5 m/s at speed=100
    dist_mm = (speed / 100.0) * 500 * dt

    # Update heading estimate based on steering angle
    if abs(angle) > 5:
        robot_angle = (robot_angle + angle * 0.1) % 360

    # Update position
    rad = math.radians(robot_angle)
    robot_x += int(dist_mm * math.sin(rad) / GRID_RES)
    robot_y -= int(dist_mm * math.cos(rad) / GRID_RES)

    # Clamp to grid bounds
    robot_x = max(1, min(GRID_SIZE - 2, robot_x))
    robot_y = max(1, min(GRID_SIZE - 2, robot_y))

# ── Main loop ──────────────────────────────────────────────────────────────────
def main():
    global last_decision, running

    log("PiCar Navigator starting...")
    log(f"Agent URL: {AGENT_URL}")
    log(f"Thresholds: front_clear={FRONT_CLEAR}mm, front_caution={FRONT_CAUTION}mm")
    log("Waiting for agent to be in AUTONOMOUS mode...")

    map_save_counter  = 0
    map_print_counter = 0

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
                # Not in autonomous mode — idle
                if last_decision != "IDLE":
                    log("Not in autonomous mode — navigator idling.")
                    last_decision = "IDLE"
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

            # Make driving decision
            speed, angle, decision = decide(sensors)

            # Log if decision changed
            if decision != last_decision:
                log(f"Decision: {last_decision} → {decision} "
                    f"(F:{sensors['lidar']['front']:.0f}mm "
                    f"L:{sensors['lidar']['left']:.0f}mm "
                    f"R:{sensors['lidar']['right']:.0f}mm "
                    f"US:{sensors['ultrasonic_cm']:.1f}cm)")
                last_decision = decision

            # Send drive command
            send_drive(speed, angle)

            # Update dead reckoning position
            update_position(speed, angle, POLL_INTERVAL)

            # Save map every 10 loops (~4 seconds)
            map_save_counter += 1
            if map_save_counter >= 10:
                save_map()
                map_save_counter = 0

            # Print ASCII map every 50 loops (~20 seconds)
            map_print_counter += 1
            if map_print_counter >= 50:
                print_map_ascii()
                map_print_counter = 0

        except KeyboardInterrupt:
            running = False
            break
        except Exception as e:
            log(f"Navigator error: {e}")

        # Maintain consistent loop timing
        elapsed = time.time() - loop_start
        sleep_time = max(0, POLL_INTERVAL - elapsed)
        time.sleep(sleep_time)

    # Cleanup
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

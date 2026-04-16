#!/usr/bin/env python3
"""
PiCar-X Motor Test — Option A turning
Front wheels turn + inside wheel stopped

Motor mapping:
  Motor 1 (left):  positive=forward, negative=backward
  Motor 2 (right): INVERTED — positive=backward, negative=forward

Run on the Pi:
    sudo systemctl stop picar-agent
    python3 motor_test.py
"""

import time
import sys

try:
    from picarx import Picarx
    px = Picarx()
    print("PiCar-X initialized.")
except Exception as e:
    print(f"Failed to initialize PiCar-X: {e}")
    sys.exit(1)

DRIVE_SPEED  = 30   # normal forward speed
TURN_SPEED   = 25   # turning speed (outside wheel)
TURN_ANGLE   = 35   # front wheel steering angle
TURN_TIME    = 1.5  # seconds per turn test
FORWARD_TIME = 1.0  # seconds for forward test
PAUSE        = 1.5  # pause between tests

def stop():
    px.set_motor_speed(1, 0)
    px.set_motor_speed(2, 0)
    px.set_dir_servo_angle(0)
    time.sleep(PAUSE)

def left_motor(speed):
    """Left motor: positive=forward, negative=backward."""
    px.set_motor_speed(1, speed)

def right_motor(speed):
    """Right motor: inverted wiring — negate to get correct direction."""
    px.set_motor_speed(2, -speed)

def forward(speed=DRIVE_SPEED, duration=FORWARD_TIME):
    print(f"  Forward speed={speed} for {duration}s")
    px.set_dir_servo_angle(0)
    left_motor(speed)
    right_motor(speed)
    time.sleep(duration)
    stop()

def turn_right(speed=TURN_SPEED, duration=TURN_TIME):
    """
    Option A right turn:
    - Front wheels angle right
    - Left (outside) wheel drives forward
    - Right (inside) wheel stopped
    """
    print(f"  Turn RIGHT: angle={TURN_ANGLE}, left fwd={speed}, right stopped")
    px.set_dir_servo_angle(TURN_ANGLE)
    left_motor(speed)
    right_motor(0)
    time.sleep(duration)
    stop()

def turn_left(speed=TURN_SPEED, duration=TURN_TIME):
    """
    Option A left turn:
    - Front wheels angle left
    - Right (outside) wheel drives forward
    - Left (inside) wheel stopped
    """
    print(f"  Turn LEFT: angle=-{TURN_ANGLE}, right fwd={speed}, left stopped")
    px.set_dir_servo_angle(-TURN_ANGLE)
    left_motor(0)
    right_motor(speed)
    time.sleep(duration)
    stop()

def turn_right_90(speed=TURN_SPEED):
    """Calibration test — try to turn ~90° right."""
    print(f"  90° RIGHT turn — adjust TURN_TIME if needed (currently {TURN_TIME}s)")
    turn_right(speed=speed, duration=TURN_TIME)

def turn_left_90(speed=TURN_SPEED):
    """Calibration test — try to turn ~90° left."""
    print(f"  90° LEFT turn — adjust TURN_TIME if needed (currently {TURN_TIME}s)")
    turn_left(speed=speed, duration=TURN_TIME)

# ── Main ───────────────────────────────────────────────────────────────────────
print("\n" + "="*50)
print("PiCar-X Turn Test — Option A")
print("Front wheels turn + inside wheel stopped")
print("="*50)
print(f"Drive speed:  {DRIVE_SPEED}")
print(f"Turn speed:   {TURN_SPEED}")
print(f"Steering angle: {TURN_ANGLE}°")
print(f"Turn duration:  {TURN_TIME}s")
print("\nStarting in 3 seconds — place car in open space!\n")
time.sleep(3)

try:
    print("[1] Forward — baseline")
    forward()

    print("\n[2] Option A — RIGHT turn")
    turn_right()

    print("\n[3] Option A — LEFT turn")
    turn_left()

    print("\n[4] Forward again — check straight tracking")
    forward()

    print("\n[5] Right turn at slower speed (20)")
    turn_right(speed=20)

    print("\n[6] Left turn at slower speed (20)")
    turn_left(speed=20)

    print("\n[7] Calibration — 90° RIGHT")
    turn_right_90()

    print("\n[8] Calibration — 90° LEFT")
    turn_left_90()

    print("\n[9] S-curve — right then left")
    print("  Right turn...")
    px.set_dir_servo_angle(TURN_ANGLE)
    left_motor(TURN_SPEED)
    right_motor(0)
    time.sleep(TURN_TIME)
    print("  Left turn...")
    px.set_dir_servo_angle(-TURN_ANGLE)
    left_motor(0)
    right_motor(TURN_SPEED)
    time.sleep(TURN_TIME)
    stop()

    print("\n" + "="*50)
    print("Tests complete!")
    print("\nTune these values in the script:")
    print(f"  TURN_SPEED  = {TURN_SPEED}  (increase for faster turns)")
    print(f"  TURN_ANGLE  = {TURN_ANGLE}  (increase for tighter turns)")
    print(f"  TURN_TIME   = {TURN_TIME}  (adjust for ~90° calibration)")
    print("\nOnce happy, these values go into config_local.py")
    print("="*50)

except KeyboardInterrupt:
    print("\nInterrupted.")
finally:
    px.set_motor_speed(1, 0)
    px.set_motor_speed(2, 0)
    px.set_dir_servo_angle(0)
    print("Motors stopped.")

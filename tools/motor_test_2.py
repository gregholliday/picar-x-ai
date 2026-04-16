from picarx import Picarx
import time

px = Picarx()

tests = [
    (1,  30, "Motor 1 FORWARD"),
    (1, -30, "Motor 1 BACKWARD"),
    (1,   0, "Motor 1 STOP"),
    (2,  30, "Motor 2 FORWARD"),
    (2, -30, "Motor 2 BACKWARD"),
    (2,   0, "Motor 2 STOP"),
]

for motor, speed, label in tests:
    print(f"\n{label} — speed={speed}")
    print("Watch which wheel moves and in what direction...")
    px.set_motor_speed(motor, speed)
    time.sleep(2)
    px.set_motor_speed(motor, 0)
    time.sleep(1)

print("\nDone.")
#!/usr/bin/env python3
"""
compass_calibrate.py — QMC5883L Compass Calibration Tool

Run this on the Pi BEFORE using the compass for navigation.
Slowly rotate the car 360 degrees (at least 2 full rotations) while
this script runs. It captures the min/max magnetic field values on
each axis and saves calibration offsets to compass_cal.json.

These offsets correct for:
  - Hard iron distortion (permanent magnets, motors, steel in chassis)
  - Sensor zero offset

Usage:
  python3 compass_calibrate.py

Saves calibration to:
  /home/pi/picar-x-ai/compass_cal.json
"""

import time
import json
import sys
import math
import signal

try:
    import smbus2 as smbus
except ImportError:
    try:
        import smbus
    except ImportError:
        print("ERROR: smbus2 not installed. Run: pip3 install smbus2 --break-system-packages")
        sys.exit(1)

# ── QMC5883L I2C config ────────────────────────────────────────────────────────
QMC5883L_ADDR   = 0x0D
REG_DATA        = 0x00   # X LSB, X MSB, Y LSB, Y MSB, Z LSB, Z MSB
REG_STATUS      = 0x06
REG_CONTROL1    = 0x09
REG_CONTROL2    = 0x0A
REG_SET_RESET   = 0x0B

# Output data rate 200Hz, full scale 8G, oversampling 512
CTRL1_CONTINUOUS = 0x1D

CAL_FILE = "/home/pi/picar-x-ai/compass_cal.json"


def init_compass(bus):
    """Initialize QMC5883L for continuous measurement."""
    bus.write_byte_data(QMC5883L_ADDR, REG_SET_RESET, 0x01)
    time.sleep(0.01)
    bus.write_byte_data(QMC5883L_ADDR, REG_CONTROL1, CTRL1_CONTINUOUS)
    time.sleep(0.01)
    print("QMC5883L initialized.")


def read_raw(bus):
    """Read raw X, Y, Z magnetic field values."""
    data = bus.read_i2c_block_data(QMC5883L_ADDR, REG_DATA, 6)
    x = (data[1] << 8) | data[0]
    y = (data[3] << 8) | data[2]
    z = (data[5] << 8) | data[4]
    # Convert to signed 16-bit
    if x > 32767: x -= 65536
    if y > 32767: y -= 65536
    if z > 32767: z -= 65536
    return x, y, z


def main():
    print("=" * 55)
    print("  QMC5883L Compass Calibration")
    print("=" * 55)
    print("\nThis calibration corrects for magnetic interference")
    print("from the PiCar motors and chassis.")
    print("\nInstructions:")
    print("  1. Place the car on a flat surface")
    print("  2. When recording starts, SLOWLY rotate the car")
    print("     through at least 2 complete 360-degree rotations")
    print("  3. Keep rotations slow and smooth")
    print("  4. Press Ctrl+C when done\n")

    try:
        bus = smbus.SMBus(1)
        init_compass(bus)
    except Exception as e:
        print(f"ERROR: Cannot initialize compass: {e}")
        print("Check wiring: VCC=3.3V, GND=GND, SDA=GPIO2, SCL=GPIO3")
        sys.exit(1)

    print("Starting in 3 seconds — get ready to rotate the car...")
    time.sleep(3)
    print("\nRECORDING — rotate car slowly now. Press Ctrl+C when done.\n")

    x_min = y_min = z_min =  32767
    x_max = y_max = z_max = -32768
    samples = 0
    recording = True

    def stop(sig, frame):
        nonlocal recording
        recording = False

    signal.signal(signal.SIGINT, stop)

    while recording:
        try:
            x, y, z = read_raw(bus)
            x_min = min(x_min, x); x_max = max(x_max, x)
            y_min = min(y_min, y); y_max = max(y_max, y)
            z_min = min(z_min, z); z_max = max(z_max, z)
            samples += 1

            # Compute current heading for live feedback
            x_cal = x - (x_max + x_min) / 2
            y_cal = y - (y_max + y_min) / 2
            heading = math.degrees(math.atan2(y_cal, x_cal))
            if heading < 0:
                heading += 360

            print(f"  Samples: {samples:4d}  Heading: {heading:6.1f}°  "
                  f"X:[{x_min:6d},{x_max:6d}]  Y:[{y_min:6d},{y_max:6d}]",
                  end="\r")
            time.sleep(0.05)

        except Exception as e:
            print(f"\nRead error: {e}")
            time.sleep(0.1)

    print(f"\n\nRecorded {samples} samples.")

    if samples < 100:
        print("WARNING: Too few samples. Run again with more rotations.")

    # Compute offsets (hard iron correction)
    x_offset = (x_max + x_min) / 2
    y_offset = (y_max + y_min) / 2
    z_offset = (z_max + z_min) / 2

    # Compute scale factors (soft iron correction)
    x_range = (x_max - x_min) / 2
    y_range = (y_max - y_min) / 2
    z_range = (z_max - z_min) / 2
    avg_range = (x_range + y_range + z_range) / 3

    x_scale = avg_range / x_range if x_range > 0 else 1.0
    y_scale = avg_range / y_range if y_range > 0 else 1.0
    z_scale = avg_range / z_range if z_range > 0 else 1.0

    cal = {
        "x_offset": round(x_offset, 2),
        "y_offset": round(y_offset, 2),
        "z_offset": round(z_offset, 2),
        "x_scale":  round(x_scale,  4),
        "y_scale":  round(y_scale,  4),
        "z_scale":  round(z_scale,  4),
        "x_min": x_min, "x_max": x_max,
        "y_min": y_min, "y_max": y_max,
        "z_min": z_min, "z_max": z_max,
        "samples":  samples,
    }

    with open(CAL_FILE, "w") as f:
        json.dump(cal, f, indent=2)

    print(f"\nCalibration saved to: {CAL_FILE}")
    print(f"\nOffsets:  X={x_offset:.1f}  Y={y_offset:.1f}  Z={z_offset:.1f}")
    print(f"Scales:   X={x_scale:.3f}  Y={y_scale:.3f}  Z={z_scale:.3f}")
    print("\nRun this again if the car's heading seems unreliable.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
compass_reader.py — BNO055 Compass Reader Module

Uses the Adafruit BNO055 9-DOF IMU for reliable heading.
The BNO055 handles all sensor fusion internally — no calibration
math needed, just read euler[0] for heading in degrees.

I2C address: 0x28 (default) or 0x29 (ADR pin high)
No conflicts with Robot Hat MCU (0x14) or QMC compass (0x2C)

Install library:
    pip3 install adafruit-circuitpython-bno055 --break-system-packages

Usage:
    from compass_reader import CompassReader
    compass = CompassReader()
    heading = compass.read_heading()  # Returns 0-360 degrees
"""

import time
import math
import json
import os

try:
    import board
    import adafruit_bno055
    BNO055_AVAILABLE = True
except ImportError:
    BNO055_AVAILABLE = False
    print("Warning: adafruit-circuitpython-bno055 not installed.")
    print("Run: pip3 install adafruit-circuitpython-bno055 --break-system-packages")

CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "compass_cal.json")


class CompassReader:
    def __init__(self):
        self.sensor           = None
        self._heading_history = []
        self._cal_data        = None
        self._init()

    def _init(self):
        """Initialize BNO055."""
        if not BNO055_AVAILABLE:
            print("BNO055 library not available.")
            return
        try:
            i2c          = board.I2C()
            self.sensor  = adafruit_bno055.BNO055_I2C(i2c)

            # Try saved calibration file first
            cal = self._load_calibration()
            if cal:
                self.sensor.offsets_magnetometer  = tuple(cal["mag_offsets"])
                self.sensor.offsets_accelerometer = tuple(cal["accel_offsets"])
                self.sensor.offsets_gyroscope     = tuple(cal["gyro_offsets"])
                print("BNO055 initialized — calibration restored from file.")
            else:
                # Use hardcoded offsets from Adafruit calibration tool
                # Generated: 2026-06-04, Anderson SC garage environment
                self.sensor.offsets_magnetometer  = (48, 187, -6)
                self.sensor.offsets_gyroscope     = (-2, -1, 3)
                self.sensor.offsets_accelerometer = (5, -38, -19)
                print("BNO055 initialized — using hardcoded calibration offsets.")

        except Exception as e:
            print(f"BNO055 init error: {e}")
            self.sensor = None

    def _load_calibration(self):
        """Load saved calibration offsets."""
        if os.path.exists(CAL_FILE):
            try:
                with open(CAL_FILE) as f:
                    return json.load(f)
            except Exception:
                pass
        return None

    def save_calibration(self):
        """
        Save current calibration offsets to file.
        Call this when calibration_status shows (3,3,3,3).
        Offsets are restored on next startup for instant calibration.
        """
        if not self.sensor:
            return False
        try:
            cal = {
                "mag_offsets":   list(self.sensor.offsets_magnetometer),
                "accel_offsets": list(self.sensor.offsets_accelerometer),
                "gyro_offsets":  list(self.sensor.offsets_gyroscope),
            }
            with open(CAL_FILE, "w") as f:
                json.dump(cal, f, indent=2)
            print(f"Calibration saved to {CAL_FILE}")
            return True
        except Exception as e:
            print(f"Failed to save calibration: {e}")
            return False

    def read_heading(self):
        """
        Read compass heading in degrees (0-360).
        0=North, 90=East, 180=South, 270=West.
        Returns None on error.
        """
        if not self.sensor:
            return None
        try:
            heading = self.sensor.euler[0]
            if heading is None:
                return None

            # Smooth with circular rolling average (3 readings)
            self._heading_history.append(heading)
            if len(self._heading_history) > 3:
                self._heading_history.pop(0)

            sin_sum = sum(math.sin(math.radians(h)) for h in self._heading_history)
            cos_sum = sum(math.cos(math.radians(h)) for h in self._heading_history)
            smoothed = math.degrees(math.atan2(sin_sum, cos_sum)) % 360

            return round(smoothed, 1)

        except Exception as e:
            print(f"Compass read error: {e}")
            return None

    def read_heading_raw(self):
        """Read heading without smoothing."""
        if not self.sensor:
            return None
        try:
            h = self.sensor.euler[0]
            return round(h, 1) if h is not None else None
        except Exception:
            return None

    def calibration_status(self):
        """
        Returns (system, gyro, accel, mag) calibration levels (0-3 each).
        3 = fully calibrated, 0 = not calibrated.
        System=3 means all sensors are fully calibrated.
        """
        if not self.sensor:
            return (0, 0, 0, 0)
        try:
            return self.sensor.calibration_status
        except Exception:
            return (0, 0, 0, 0)

    def is_calibrated(self, min_level=2):
        """
        Returns True if magnetometer calibration is at or above min_level.
        min_level=2 is good enough for navigation.
        min_level=3 is fully calibrated.
        """
        cal = self.calibration_status()
        return cal[3] >= min_level  # cal[3] = magnetometer

    def cardinal(self, heading):
        """Convert heading to cardinal direction string."""
        if heading is None:
            return "?"
        dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        return dirs[round(heading / 45) % 8]

    @property
    def ok(self):
        """True if sensor is connected and returning valid readings."""
        return self.sensor is not None and self.read_heading_raw() is not None


if __name__ == "__main__":
    print("Testing BNO055 compass...")
    print("Move car in figure-8 pattern to calibrate. Ctrl+C to stop.\n")

    c = CompassReader()

    if not c.ok:
        print("ERROR: BNO055 not available.")
        exit(1)

    try:
        while True:
            h   = c.read_heading()
            r   = c.read_heading_raw()
            cal = c.calibration_status()
            print(f"  Heading: {h:6.1f}°  Raw: {r:6.1f}°  "
                  f"Dir: {c.cardinal(h):<2}  "
                  f"Cal: sys={cal[0]} gyro={cal[1]} accel={cal[2]} mag={cal[3]}",
                  end="\r", flush=True)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nSaving calibration...")
        cal = c.calibration_status()
        if cal[3] >= 2:
            c.save_calibration()
            print("Done! Calibration saved — will be restored on next startup.")
        else:
            print(f"Magnetometer calibration too low ({cal[3]}/3) — not saving.")
            print("Move car in figure-8 pattern and try again.")
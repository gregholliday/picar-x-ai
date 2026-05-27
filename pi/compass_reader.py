#!/usr/bin/env python3
"""
compass_reader.py — QMC5883L Compass Reader Module

Import this module in picar_agent.py to get heading readings.

Usage:
    from compass_reader import CompassReader
    compass = CompassReader()
    heading = compass.read_heading()  # Returns 0-360 degrees
"""

import math
import time
import json
import os

try:
    import smbus2 as smbus
except ImportError:
    import smbus

# ── QMC5883L registers ─────────────────────────────────────────────────────────
QMC5883L_ADDR   = 0x0D
REG_DATA        = 0x00
REG_CONTROL1    = 0x09
REG_SET_RESET   = 0x0B
CTRL1_CONTINUOUS = 0x1D

CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "compass_cal.json")

# Magnetic declination for Anderson, SC (degrees)
# Positive = east, negative = west
# Current declination for Anderson SC ~-7.5 degrees (update annually if needed)
DECLINATION = -7.5


class CompassReader:
    def __init__(self, bus_num=1):
        self.bus         = smbus.SMBus(bus_num)
        self.calibration = self._load_calibration()
        self._init()
        self._heading_history = []
        print(f"CompassReader initialized. Calibration: {'loaded' if self.calibration else 'NONE - run compass_calibrate.py'}")

    def _init(self):
        """Initialize QMC5883L."""
        try:
            self.bus.write_byte_data(QMC5883L_ADDR, REG_SET_RESET, 0x01)
            time.sleep(0.01)
            self.bus.write_byte_data(QMC5883L_ADDR, REG_CONTROL1, CTRL1_CONTINUOUS)
            time.sleep(0.01)
        except Exception as e:
            print(f"Compass init error: {e}")

    def _load_calibration(self):
        """Load calibration from file if it exists."""
        if os.path.exists(CAL_FILE):
            with open(CAL_FILE) as f:
                return json.load(f)
        return None

    def _read_raw(self):
        """Read raw X, Y, Z values."""
        data = self.bus.read_i2c_block_data(QMC5883L_ADDR, REG_DATA, 6)
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]
        if x > 32767: x -= 65536
        if y > 32767: y -= 65536
        if z > 32767: z -= 65536
        return x, y, z

    def _apply_calibration(self, x, y, z):
        """Apply hard and soft iron corrections."""
        if not self.calibration:
            return x, y, z
        cal = self.calibration
        x = (x - cal["x_offset"]) * cal["x_scale"]
        y = (y - cal["y_offset"]) * cal["y_scale"]
        z = (z - cal["z_offset"]) * cal["z_scale"]
        return x, y, z

    def read_heading(self):
        """
        Read compass heading in degrees (0-360).
        0 = North (magnetic), 90 = East, 180 = South, 270 = West.
        Returns None on error.
        """
        try:
            x, y, z = self._read_raw()
            x, y, z = self._apply_calibration(x, y, z)

            # Calculate heading from X and Y axes
            heading = math.degrees(math.atan2(y, x))

            # Apply magnetic declination for Anderson, SC
            heading += DECLINATION

            # Normalize to 0-360
            heading = heading % 360

            # Smooth with rolling average (last 5 readings)
            self._heading_history.append(heading)
            if len(self._heading_history) > 5:
                self._heading_history.pop(0)

            # Circular mean for smoothing
            sin_sum = sum(math.sin(math.radians(h)) for h in self._heading_history)
            cos_sum = sum(math.cos(math.radians(h)) for h in self._heading_history)
            smoothed = math.degrees(math.atan2(sin_sum, cos_sum)) % 360

            return round(smoothed, 1)

        except Exception as e:
            print(f"Compass read error: {e}")
            return None

    def read_heading_raw(self):
        """Read heading without smoothing."""
        try:
            x, y, z = self._read_raw()
            x, y, z = self._apply_calibration(x, y, z)
            heading = (math.degrees(math.atan2(y, x)) + DECLINATION) % 360
            return round(heading, 1)
        except Exception:
            return None

    def cardinal(self, heading):
        """Convert heading to cardinal direction string."""
        if heading is None:
            return "?"
        dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        idx  = round(heading / 45) % 8
        return dirs[idx]


if __name__ == "__main__":
    print("Testing compass... Press Ctrl+C to stop.")
    c = CompassReader()
    while True:
        h = c.read_heading()
        r = c.read_heading_raw()
        print(f"  Heading: {h:6.1f}°  Raw: {r:6.1f}°  Direction: {c.cardinal(h)}", end="\r")
        time.sleep(0.1)

#!/usr/bin/env python3
"""
compass_reader.py — QMC5883P Compass Reader Module

QMC5883P register map (different from QMC5883L):
  0x00 = Chip ID (default 0x80)
  0x01-0x02 = X axis LSB/MSB
  0x03-0x04 = Y axis LSB/MSB
  0x05-0x06 = Z axis LSB/MSB
  0x09 = Status register
  0x0A = Control register 1
  Default I2C address: 0x2C

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

# ── QMC5883P registers ─────────────────────────────────────────────────────────
QMC5883P_ADDR   = 0x2C
REG_CHIP_ID     = 0x00   # Should return 0x80
REG_DATA        = 0x01   # X LSB, X MSB, Y LSB, Y MSB, Z LSB, Z MSB
REG_STATUS      = 0x09
REG_CONTROL1    = 0x0A

# Control register 1 settings
# Bits: [7:6]=OSR(oversampling) [5:4]=ODR(rate) [3:2]=RNG(range) [1:0]=MODE
# OSR=8(11), ODR=200Hz(11), RNG=8G(10), MODE=Normal(01) = 0xFF
CTRL1_NORMAL    = 0xFF

CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "compass_cal.json")

# Magnetic declination for Anderson, SC (~-7.5 degrees)
DECLINATION = -7.5


class CompassReader:
    def __init__(self, bus_num=1):
        self.bus             = smbus.SMBus(bus_num)
        self.calibration     = self._load_calibration()
        self._heading_history = []
        self._init()

    def _init(self):
        """Initialize QMC5883P for continuous measurement."""
        try:
            # Verify chip ID
            chip_id = self.bus.read_byte_data(QMC5883P_ADDR, REG_CHIP_ID)
            if chip_id != 0x80:
                print(f"Warning: Unexpected chip ID 0x{chip_id:02X} (expected 0x80)")

            # Set continuous measurement mode
            self.bus.write_byte_data(QMC5883P_ADDR, REG_CONTROL1, CTRL1_NORMAL)
            time.sleep(0.05)
            print(f"QMC5883P initialized at 0x{QMC5883P_ADDR:02X}. "
                  f"Calibration: {'loaded' if self.calibration else 'NONE — run compass_calibrate.py'}")
        except Exception as e:
            print(f"Compass init error: {e}")

    def _load_calibration(self):
        if os.path.exists(CAL_FILE):
            with open(CAL_FILE) as f:
                return json.load(f)
        return None

    def _read_raw(self):
        """Read raw X, Y, Z values from QMC5883P."""
        # Read 6 bytes starting at REG_DATA (0x01)
        data = self.bus.read_i2c_block_data(QMC5883P_ADDR, REG_DATA, 6)
        # QMC5883P: LSB first, then MSB
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]
        # Convert to signed 16-bit
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
        0 = North, 90 = East, 180 = South, 270 = West.
        Returns None on error.
        """
        try:
            x, y, z = self._read_raw()
            x, y, z = self._apply_calibration(x, y, z)

            heading  = math.degrees(math.atan2(y, x))
            heading += DECLINATION
            heading  = heading % 360

            # Smooth with circular rolling average
            self._heading_history.append(heading)
            if len(self._heading_history) > 5:
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
        try:
            x, y, z = self._read_raw()
            x, y, z = self._apply_calibration(x, y, z)
            heading = (math.degrees(math.atan2(y, x)) + DECLINATION) % 360
            return round(heading, 1)
        except Exception:
            return None

    def read_xyz(self):
        """Read raw calibrated X, Y, Z values."""
        try:
            x, y, z = self._read_raw()
            return self._apply_calibration(x, y, z)
        except Exception:
            return None, None, None

    def cardinal(self, heading):
        """Convert heading to cardinal direction string."""
        if heading is None:
            return "?"
        dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        idx  = round(heading / 45) % 8
        return dirs[idx]


if __name__ == "__main__":
    print("Testing QMC5883P compass... Press Ctrl+C to stop.")
    c = CompassReader()
    while True:
        try:
            h   = c.read_heading()
            r   = c.read_heading_raw()
            xyz = c.read_xyz()
            if h is not None:
                print(f"  Heading: {h:6.1f}°  Raw: {r:6.1f}°  "
                      f"Dir: {c.cardinal(h):<2}  "
                      f"XYZ: ({xyz[0]:.0f}, {xyz[1]:.0f}, {xyz[2]:.0f})",
                      end="\r")
            else:
                print("  No reading", end="\r")
            time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nDone.")
            break
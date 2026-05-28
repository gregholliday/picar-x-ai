#!/usr/bin/env python3
"""
compass_reader_hmc5883l.py — HMC5883L Compass Reader Module

For the genuine Honeywell HMC5883L chip (GY-273 module).
I2C address: 0x1E (confirms genuine chip — clones use 0x0D or 0x2C)

Register map (different from QMC variants):
  0x00 = Configuration Register A
  0x01 = Configuration Register B (gain)
  0x02 = Mode Register
  0x03-0x08 = Data Output Registers (X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB)
  0x09 = Status Register
  0x0A-0x0C = Identification Registers (should read 'H','4','3')

Usage:
    from compass_reader_hmc5883l import CompassReader
    compass = CompassReader()
    heading = compass.read_heading()  # Returns 0-360 degrees

Note: Deploy this file as compass_reader.py on the Pi when HMC5883L is confirmed.
"""

import math
import time
import json
import os

try:
    import smbus2 as smbus
except ImportError:
    import smbus

# ── HMC5883L registers ─────────────────────────────────────────────────────────
HMC5883L_ADDR  = 0x1E

REG_CONFIG_A   = 0x00   # Configuration A: samples, data rate, measurement mode
REG_CONFIG_B   = 0x01   # Configuration B: gain
REG_MODE       = 0x02   # Mode: continuous or single measurement
REG_DATA_X_MSB = 0x03   # X axis MSB
REG_DATA_X_LSB = 0x04   # X axis LSB
REG_DATA_Z_MSB = 0x05   # Z axis MSB (note: Z before Y in HMC5883L)
REG_DATA_Z_LSB = 0x06   # Z axis LSB
REG_DATA_Y_MSB = 0x07   # Y axis MSB
REG_DATA_Y_LSB = 0x08   # Y axis LSB
REG_STATUS     = 0x09   # Status
REG_ID_A       = 0x0A   # ID Register A (should be 'H' = 0x48)
REG_ID_B       = 0x0B   # ID Register B (should be '4' = 0x34)
REG_ID_C       = 0x0C   # ID Register C (should be '3' = 0x33)

# Configuration A: 8 samples averaged, 15Hz output, normal measurement
CONFIG_A_DEFAULT = 0x70  # 0b01110000

# Configuration B: Gain = 1.3 Ga (default, good for most environments)
# Higher gain = more sensitive but saturates in strong fields
# 0x20 = ±1.3Ga, 0x40 = ±1.9Ga, 0x60 = ±2.5Ga, 0x80 = ±4.0Ga
CONFIG_B_DEFAULT = 0x20

# Mode: continuous measurement
MODE_CONTINUOUS = 0x00

CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "compass_cal.json")

# Magnetic declination for Anderson/Easley SC (~-7.5 degrees west)
DECLINATION = -7.5


class CompassReader:
    def __init__(self, bus_num=1):
        self.bus              = smbus.SMBus(bus_num)
        self.calibration      = self._load_calibration()
        self._heading_history = []
        self._init()

    def _init(self):
        """Initialize HMC5883L for continuous measurement."""
        try:
            # Verify chip identity
            id_a = self.bus.read_byte_data(HMC5883L_ADDR, REG_ID_A)
            id_b = self.bus.read_byte_data(HMC5883L_ADDR, REG_ID_B)
            id_c = self.bus.read_byte_data(HMC5883L_ADDR, REG_ID_C)

            if id_a == 0x48 and id_b == 0x34 and id_c == 0x33:
                print("HMC5883L verified — genuine Honeywell chip ✓")
            else:
                print(f"Warning: Unexpected chip ID: 0x{id_a:02X} 0x{id_b:02X} 0x{id_c:02X}")
                print("Expected: 0x48 0x34 0x33 (H43)")
                print("May be a QMC5883L clone — consider using compass_reader.py instead")

            # Configure
            self.bus.write_byte_data(HMC5883L_ADDR, REG_CONFIG_A, CONFIG_A_DEFAULT)
            self.bus.write_byte_data(HMC5883L_ADDR, REG_CONFIG_B, CONFIG_B_DEFAULT)
            self.bus.write_byte_data(HMC5883L_ADDR, REG_MODE,     MODE_CONTINUOUS)
            time.sleep(0.1)  # Wait for first measurement

            print(f"HMC5883L initialized at 0x{HMC5883L_ADDR:02X}. "
                  f"Calibration: {'loaded' if self.calibration else 'NONE — run compass_calibrate.py'}")

        except Exception as e:
            print(f"Compass init error: {e}")
            print("Check wiring: VCC=3.3V, GND=GND, SDA=GPIO2, SCL=GPIO3")

    def _load_calibration(self):
        """Load calibration offsets from file."""
        if os.path.exists(CAL_FILE):
            with open(CAL_FILE) as f:
                return json.load(f)
        return None

    def _read_raw(self):
        """
        Read raw X, Y, Z values from HMC5883L.
        Note: HMC5883L data register order is X, Z, Y (not X, Y, Z!)
        """
        data = self.bus.read_i2c_block_data(HMC5883L_ADDR, REG_DATA_X_MSB, 6)

        # HMC5883L: MSB first, order is X, Z, Y
        x = (data[0] << 8) | data[1]
        z = (data[2] << 8) | data[3]
        y = (data[4] << 8) | data[5]

        # Convert to signed 16-bit
        if x > 32767: x -= 65536
        if y > 32767: y -= 65536
        if z > 32767: z -= 65536

        # Check for overflow (-4096 indicates saturation)
        if x == -4096 or y == -4096 or z == -4096:
            return None, None, None

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

    def self_test(self):
        """
        Run HMC5883L built-in self test.
        Returns True if sensor passes, False if failed.
        """
        try:
            print("Running HMC5883L self test...")

            # Enable positive bias self test
            self.bus.write_byte_data(HMC5883L_ADDR, REG_CONFIG_A, 0x71)  # positive bias
            self.bus.write_byte_data(HMC5883L_ADDR, REG_MODE, 0x00)
            time.sleep(0.1)

            x, y, z = self._read_raw()

            # Restore normal config
            self.bus.write_byte_data(HMC5883L_ADDR, REG_CONFIG_A, CONFIG_A_DEFAULT)
            self.bus.write_byte_data(HMC5883L_ADDR, REG_MODE, MODE_CONTINUOUS)

            if x is None:
                print("Self test failed — overflow detected")
                return False

            # At gain 1.3Ga, self test should produce values between 243-575
            passed = all(243 <= abs(v) <= 575 for v in [x, y, z] if v is not None)
            print(f"Self test: X={x} Y={y} Z={z} — {'PASSED ✓' if passed else 'FAILED ✗'}")
            return passed

        except Exception as e:
            print(f"Self test error: {e}")
            return False

    def read_heading(self):
        """
        Read compass heading in degrees (0-360).
        0=North, 90=East, 180=South, 270=West.
        Returns None on error or overflow.
        """
        try:
            x, y, z = self._read_raw()
            if x is None:
                return None

            x, y, z = self._apply_calibration(x, y, z)

            # Calculate heading
            heading  = math.degrees(math.atan2(y, x))
            heading += DECLINATION
            heading  = heading % 360

            # Smooth with circular rolling average (5 readings)
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
            if x is None:
                return None
            x, y, z = self._apply_calibration(x, y, z)
            return round((math.degrees(math.atan2(y, x)) + DECLINATION) % 360, 1)
        except Exception:
            return None

    def read_xyz(self):
        """Read calibrated X, Y, Z values."""
        try:
            x, y, z = self._read_raw()
            if x is None:
                return None, None, None
            return self._apply_calibration(x, y, z)
        except Exception:
            return None, None, None

    def cardinal(self, heading):
        """Convert heading to cardinal direction."""
        if heading is None:
            return "?"
        dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        return dirs[round(heading / 45) % 8]


if __name__ == "__main__":
    print("Testing HMC5883L compass... Press Ctrl+C to stop.")
    c = CompassReader()
    c.self_test()
    print()
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
                print("  No reading (overflow or error)", end="\r")
            time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nDone.")
            break

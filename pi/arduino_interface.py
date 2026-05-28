#!/usr/bin/env python3
"""
arduino_interface.py — Pi to Arduino Serial Interface

Handles communication between the Pi agent and the Arduino
motor controller on the new tracked/mecanum robot.

The Arduino sends status at 10Hz. The Pi sends commands as needed.

Usage:
    from arduino_interface import ArduinoInterface
    arduino = ArduinoInterface()
    arduino.drive(left=50, right=50)
    arduino.turn(90)
    arduino.drive_distance(500)
    status = arduino.get_status()
"""

import serial
import json
import time
import threading
from datetime import datetime


class ArduinoInterface:
    def __init__(self, port="/dev/ttyACM0", baud=9600, timeout=2.0):
        self.port    = port
        self.baud    = baud
        self._serial = None
        self._lock   = threading.Lock()
        self._status = {
            "connected":    False,
            "left_enc":     0,
            "right_enc":    0,
            "left_speed":   0,
            "right_speed":  0,
            "left_mm":      0.0,
            "right_mm":     0.0,
            "last_update":  None,
        }
        self._running = False
        self._connect()

    def _connect(self):
        """Connect to Arduino via serial."""
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=1.0)
            time.sleep(2.0)  # Wait for Arduino reset after serial connect
            self._status["connected"] = True
            print(f"Arduino connected on {self.port} at {self.baud} baud")

            # Start background reader thread
            self._running = True
            self._reader_thread = threading.Thread(
                target=self._read_loop, daemon=True
            )
            self._reader_thread.start()

        except Exception as e:
            print(f"Arduino connection failed: {e}")
            self._status["connected"] = False

    def _read_loop(self):
        """Background thread — reads status updates from Arduino at 10Hz."""
        while self._running:
            try:
                if self._serial and self._serial.in_waiting:
                    line = self._serial.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        try:
                            data = json.loads(line)
                            with self._lock:
                                self._status.update(data)
                                self._status["last_update"] = datetime.now().isoformat()
                                self._status["connected"]   = True
                        except json.JSONDecodeError:
                            pass  # Ignore malformed lines
            except Exception as e:
                self._status["connected"] = False
                print(f"Arduino read error: {e}")
                time.sleep(1.0)

    def _send(self, command: dict):
        """Send a command to Arduino."""
        if not self._status["connected"]:
            print("Arduino not connected")
            return False
        try:
            with self._lock:
                msg = json.dumps(command) + "\n"
                self._serial.write(msg.encode('utf-8'))
            return True
        except Exception as e:
            print(f"Arduino send error: {e}")
            return False

    def drive(self, left: int, right: int):
        """
        Drive motors at specified speeds.
        left, right: -100 to +100 (negative = reverse)
        """
        return self._send({"cmd": "drive", "left": left, "right": right})

    def stop(self):
        """Stop all motors."""
        return self._send({"cmd": "stop"})

    def turn(self, degrees: float, speed: int = 40):
        """
        Turn in place by specified degrees using encoder dead reckoning.
        Positive = right, negative = left.
        Blocks until complete.
        """
        self._send({"cmd": "turn", "degrees": degrees, "speed": speed})
        # Wait for done response
        start = time.time()
        while time.time() - start < 30:
            status = self.get_status()
            if status.get("action") == "turn" and status.get("status") == "done":
                return True
            time.sleep(0.1)
        print("Turn timeout")
        return False

    def drive_distance(self, mm: float, speed: int = 40):
        """
        Drive straight for specified distance in mm.
        Positive = forward, negative = backward.
        Blocks until complete.
        """
        self._send({"cmd": "drive_distance", "mm": mm, "speed": speed})
        # Wait for done response
        start = time.time()
        while time.time() - start < 60:
            status = self.get_status()
            if status.get("action") == "drive_distance" and status.get("status") == "done":
                return True
            time.sleep(0.1)
        print("Drive distance timeout")
        return False

    def reset_encoders(self):
        """Zero the encoder counts."""
        return self._send({"cmd": "reset_encoders"})

    def get_status(self):
        """Get current Arduino status."""
        with self._lock:
            return dict(self._status)

    def get_encoder_counts(self):
        """Get left and right encoder counts."""
        with self._lock:
            return self._status["left_enc"], self._status["right_enc"]

    def get_distance_mm(self):
        """Get distance traveled in mm (average of left and right)."""
        with self._lock:
            return (self._status["left_mm"] + self._status["right_mm"]) / 2

    def close(self):
        """Close serial connection."""
        self._running = False
        if self._serial:
            self.stop()
            time.sleep(0.1)
            self._serial.close()
        print("Arduino disconnected.")


if __name__ == "__main__":
    print("Testing Arduino interface...")
    arduino = ArduinoInterface()

    if not arduino.get_status()["connected"]:
        print("ERROR: Arduino not connected")
        exit(1)

    print("Arduino connected. Testing motors...")
    time.sleep(1)

    print("Forward 1 second...")
    arduino.drive(50, 50)
    time.sleep(1)
    arduino.stop()

    print(f"Encoder counts: {arduino.get_encoder_counts()}")
    print(f"Distance traveled: {arduino.get_distance_mm():.1f}mm")

    print("\nTurning right 90 degrees...")
    arduino.reset_encoders()
    arduino.turn(90)
    print(f"Turn complete. Encoder counts: {arduino.get_encoder_counts()}")

    arduino.close()
    print("Done.")

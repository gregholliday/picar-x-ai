#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rplidar
from rplidar import Rplidar
import time
import traceback

def read_lidar_scan(
    port_name='/dev/ttyUSB0',
    baudrate=115200,
    timeout_ms=1000
):
    """
    Read angle and distance data from a SLAMTEC C1M1 LiDAR.

    Args:
        port_name (str): Serial port name (default: /dev/ttyUSB0)
        baudrate (int): Communication baud rate (default: 115200)
        timeout_ms (int): Timeout for LiDAR operations in milliseconds

    Returns:
        list or None: List of (angle_deg, distance_mm) tuples, or None on failure
    """
    lidar = None
    scan_data = []

    try:
        print(f"Initializing LiDAR on {port_name}...")
        lidar = Rplidar(port_name, baudrate)

        if not lidar.is_working():
            print("Error: LiDAR is not working.")
            return None

        print("LiDAR connected successfully. Starting measurement...")
        lidar.stop()
        lidar.reset()
        time.sleep(0.5)

        if not lidar.start_motor():
            print("Error: Failed to start LiDAR motor.")
            return None

        lidar.start_scanning()
        time.sleep(0.2)  # Allow scan to initialize

        print("Reading scan data...")
        while not lidar.is_scanning():
            time.sleep(0.1)

        for _ in range(5):  # Read up to 5 consecutive measurements
            scan = lidar.get_distance()
            if scan is None:
                continue
            for point in scan:
                distance_mm = point.dist
                angle_rad = point.angle_q16
                angle_deg = angle_rad / 256.0
                scan_data.append((angle_deg, distance_mm))
            break  # Exit after one successful scan

        print(f"Successfully collected {len(scan_data)} points.")

    except PermissionError:
        print("Error: Permission denied. Try running with sudo or adjust udev rules.")
        traceback.print_exc()
    except OSError:
        print("Error: Serial port issue (e.g., port not found or busy).")
        traceback.print_exc()
    except Exception as e:
        print(f"Unexpected error occurred: {type(e).__name__}")
        print(str(e))
        traceback.print_exc()
    finally:
        if lidar:
            try:
                lidar.stop_scanning()
                lidar.stop_motor()
                lidar.disconnect()
                print("LiDAR disconnected successfully.")
            except Exception as e:
                print(f"Warning: Error during cleanup: {e}")

    return scan_data

if __name__ == "__main__":
    scan_data = read_lidar_scan()
    if scan_data:
        print(f"Sample data (first 5 points):")
        for i, (angle, dist) in enumerate(scan_data[:5]):
            print(f"{i+1}. Angle: {angle:.2f}°, Distance: {dist:.2f} mm")


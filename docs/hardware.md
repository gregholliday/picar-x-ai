# Hardware Setup

## Required Hardware

| Component | Notes |
|---|---|
| SunFounder PiCar-X Kit | Includes Robot HAT, servos, motors, grayscale sensors, ultrasonic sensor |
| Raspberry Pi 4 (2GB+) | Tested on Pi 4, 4GB |
| RPLidar C1M1 | 360° DTOF laser scanner, 12m range |
| USB-A cable | For LiDAR connection to Pi |
| Host machine | Linux, Windows, or Mac on the same network |
| MicroSD card | 32GB+ recommended, Class 10 |
| 7.4V LiPo battery | For PiCar-X power |

## Optional Hardware

| Component | Notes |
|---|---|
| Ollama machine | Any machine capable of running Gemma 3 (GPU recommended) |
| iPad/tablet | For touch-based dashboard control |

---

## PiCar-X Assembly

Follow the official SunFounder assembly guide:
- https://docs.sunfounder.com/projects/picar-x/en/latest/

The standard kit includes:
- Robot HAT (I2C-based servo/motor controller)
- 2x rear drive motors
- 3x servo motors (steering, camera pan, camera tilt)
- Ultrasonic distance sensor (forward-facing)
- 3x grayscale sensors (downward-facing, line following/cliff detection)
- Camera module (OV5647 CSI)

---

## RPLidar C1M1 Mounting

The C1M1 mounts on top of the PiCar-X body. Key points:

- Mount with the **arrow/indicator pointing forward** (same direction as the car's front)
- Connect via USB to any available USB port on the Pi
- Ensure the spinning head has **clearance** — nothing should be within 5cm of the sensor
- The USB cable should be routed so it doesn't interfere with the spinning head

### Verifying Orientation

With the car facing forward, run:
```bash
curl http://PI_IP:8000/api/sensors
```

Place an object on the **right side** of the car. The `right` distance value should be small. If it shows under `left`, the LiDAR is mounted 180° rotated — either physically rotate it or swap the angle ranges in `picar_agent_v7.py`.

---

## Raspberry Pi Setup

### 1. Flash Raspberry Pi OS

Use Raspberry Pi Imager to flash **Raspberry Pi OS (64-bit)** to your SD card.

Enable SSH and set your username/password in the imager before flashing.

### 2. Install SunFounder Libraries

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install SunFounder PiCar-X libraries
pip install picarx robot_hat vilib
```

### 3. Install LiDAR Library

```bash
pip install rplidarc1 pyserial
```

### 4. Configure Serial Permissions

```bash
sudo usermod -a -G dialout $USER
# Log out and back in for this to take effect
```

### 5. Configure Sudo for Shutdown

To allow the dashboard shutdown button to work:

```bash
sudo visudo
```

Add at the bottom (replace `pi` with your username):
```
pi ALL=(ALL) NOPASSWD: /usr/sbin/shutdown
```

---

## Network Setup

All components need to be on the same local network.

| Component | Typical IP |
|---|---|
| Raspberry Pi | 192.168.1.x (set static in router) |
| Host machine | 192.168.1.x |
| Ollama machine | 192.168.1.x |

### Setting a Static IP

Set a static DHCP reservation for the Pi in your router using its MAC address. This ensures the Pi always gets the same IP and you don't need to update configuration files.

---

## Sensor Calibration

See [calibration.md](calibration.md) for detailed sensor calibration instructions, including:
- Servo centering
- Grayscale cliff detection thresholds
- Ultrasonic validation

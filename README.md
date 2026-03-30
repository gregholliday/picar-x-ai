# PiCar-X AI Mission Control

A full-featured autonomous driving platform for the [SunFounder PiCar-X](https://www.sunfounder.com/products/picar-x) robot car, featuring:

- 🎥 **Live camera feed** via MJPEG stream
- 🔄 **360° LiDAR mapping** with RPLidar C1M1
- 🧠 **Autonomous navigation** with obstacle avoidance and cliff detection
- 🕹️ **Manual remote control** via WASD keyboard or D-pad
- 📡 **Real-time telemetry** dashboard
- 🤖 **Gemma 3 vision integration** via Ollama (optional)
- 🚀 **Three deployment options**: Kubernetes, Docker, or bare Python

![PiCar-X Mission Control Dashboard](docs/images/dashboard.png)

---

## Hardware Requirements

| Component | Details |
|---|---|
| SunFounder PiCar-X Kit | Includes robot HAT, servos, motors |
| Raspberry Pi 4 | 2GB+ RAM recommended |
| RPLidar C1M1 | 360° DTOF laser scanner |
| Host machine | Any Linux/Windows/Mac machine on the same network |
| Ollama (optional) | For Gemma 3 vision integration |

See [docs/hardware.md](docs/hardware.md) for full hardware setup and wiring details.

---

## Quick Start

### 1. Set Up the Pi Agent

```bash
# Clone the repo on your Raspberry Pi
git clone https://github.com/gregholliday/picar-x-ai.git
cd picar-x-ai

# Install Pi dependencies
pip install -r pi/requirements.txt

# Install and start the agent as a system service
sudo cp pi/picar-agent.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable picar-agent
sudo systemctl start picar-agent
```

### 2. Deploy the Dashboard

Choose your deployment method:

| Method | Best For | Guide |
|---|---|---|
| **Kubernetes (K3s)** | Always-on, production-like | [setup-kubernetes.md](docs/setup-kubernetes.md) |
| **Docker** | Simple, no K8s needed | [setup-docker.md](docs/setup-docker.md) |
| **Bare Python** | Quickest start, no containers | [setup-bare.md](docs/setup-bare.md) |

### 3. Run the Navigator (Optional)

The navigator runs on your host machine and drives the car autonomously:

```bash
# Install navigator dependencies
pip install -r navigator/requirements.txt

# Run the navigator
python3 navigator/picar_navigator.py
```

Switch the dashboard to **AUTO** mode to activate autonomous driving.

---

## Architecture

```
Raspberry Pi 4 (PiCar-X)
├── picar_agent_v7.py         ← FastAPI server
│   ├── Camera stream (Vilib) → :9000/mjpg
│   ├── LiDAR C1M1 scanning
│   ├── Ultrasonic sensor
│   ├── Grayscale/cliff sensors
│   └── Reflex safety loop (10Hz)
│
└── Exposes REST API on :8000
      ├── GET  /api/status
      ├── GET  /api/sensors
      ├── GET  /api/lidar
      ├── GET  /api/lidar/summary
      ├── POST /api/drive
      ├── POST /api/stop
      ├── POST /api/mode/{mode}
      └── POST /api/shutdown

Host Machine (Linux/Windows/Mac)
├── picar_dashboard.html      ← Mission control UI
│   ├── Live camera feed
│   ├── LiDAR radar visualization
│   ├── Drive controls (WASD + D-pad)
│   └── Real-time telemetry
│
└── picar_navigator.py        ← Autonomous driving brain
    ├── Obstacle avoidance
    ├── Cliff detection
    ├── Room mapping
    └── Gemma 3 vision (optional)
```

---

## API Reference

### GET /api/status
Returns current vehicle state.

```json
{
  "mode": "manual",
  "speed": 0,
  "angle": 0,
  "lidar_ok": true,
  "lidar_points": 420,
  "stream_url": "http://192.168.1.225:9000/mjpg",
  "cliff_detected": false,
  "obstacle_close": false,
  "reflex_active": false
}
```

### GET /api/sensors
Returns all sensor readings in one call (used by navigator).

```json
{
  "ultrasonic_cm": 65.5,
  "grayscale": [1478, 1466, 1513],
  "cliff_detected": false,
  "obstacle_close": false,
  "reflex_active": false,
  "lidar": {
    "points": 420,
    "front": 621.8,
    "left": 355.0,
    "back": 940.2,
    "right": 1009.5
  }
}
```

### POST /api/drive?speed={n}&angle={n}
Send drive command. Speed: -100 to 100. Angle: -40 to 40 degrees.

### POST /api/mode/{mode}
Set drive mode. Values: `manual` or `autonomous`.

### POST /api/stop
Emergency stop.

### POST /api/shutdown
Safely shutdown the Raspberry Pi.

---

## Configuration

Edit the thresholds at the top of `pi/picar_agent_v7.py`:

```python
ULTRASONIC_STOP = 15    # cm  — emergency stop distance
ULTRASONIC_SLOW = 30    # cm  — slow down distance
CLIFF_STOP      = 100   # ADC — cliff detection threshold
CLIFF_WARN      = 500   # ADC — near-edge warning threshold
```

Edit navigator behavior in `navigator/picar_navigator.py`:

```python
BASE_SPEED    = 40     # normal cruising speed (0-100)
SLOW_SPEED    = 25     # speed when navigating obstacles
TURN_ANGLE    = 35     # steering angle for turns (degrees)
FRONT_CLEAR   = 600    # mm — path clear, drive forward
FRONT_CAUTION = 350    # mm — slow down threshold
SIDE_CLEAR    = 300    # mm — minimum side clearance for turns
```

---

## Ollama / Gemma 3 Integration

The navigator supports optional vision-guided navigation using Gemma 3 running on a separate machine via Ollama. Set the Ollama URL in `navigator/picar_navigator.py`:

```python
OLLAMA_URL   = "http://192.168.1.x:11434"  # your Ollama machine IP
OLLAMA_MODEL = "gemma3"
```

See [docs/setup-bare.md](docs/setup-bare.md) for full Ollama integration details.

---

## Sensor Calibration

Before running autonomous mode, calibrate your sensors:

```bash
# On the Pi
python3 pi/calibrate_sensors.py
```

See [docs/calibration.md](docs/calibration.md) for detailed calibration instructions.

---

## Contributing

Pull requests welcome! Please open an issue first to discuss major changes.

---

## License

MIT License — see [LICENSE](LICENSE) for details.

## Author

**Greg Holliday** — [@gregholliday](https://github.com/gregholliday)

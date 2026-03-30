# Setup — Bare Python (No Containers)

This is the quickest way to get started. No Docker or Kubernetes required.

---

## Prerequisites

- Raspberry Pi 4 with PiCar-X assembled and running Raspberry Pi OS
- RPLidar C1M1 connected via USB
- Host machine (Linux, Windows, or Mac) on the same network
- Python 3.9+ on the host machine

---

## Step 1 — Set Up the Pi Agent

SSH into your Raspberry Pi:

```bash
ssh pi@YOUR_PI_IP
```

Clone the repo and install dependencies:

```bash
git clone https://github.com/gregholliday/picar-x-ai.git
cd picar-x-ai
pip install -r pi/requirements.txt
```

Edit the agent to set your Pi's IP address:

```bash
nano pi/picar_agent_v7.py
```

Find and update the stream URL (search for `192.168.1.225`):
```python
"stream_url": "http://YOUR_PI_IP:9000/mjpg"
```

Install and start the systemd service:

```bash
# Edit the service file to use your username
nano pi/picar-agent.service
# Change 'pi' to your actual username in User=, Group=,
# Environment=LOGNAME=, Environment=HOME=, Environment=USER=

sudo cp pi/picar-agent.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable picar-agent
sudo systemctl start picar-agent
```

Verify it's running:

```bash
sudo systemctl status picar-agent
curl http://localhost:8000/api/status
```

---

## Step 2 — Serve the Dashboard

On your **host machine**, clone the repo:

```bash
git clone https://github.com/gregholliday/picar-x-ai.git
cd picar-x-ai
```

Edit the dashboard to point to your Pi's IP:

```bash
nano dashboard/picar_dashboard.html
```

Find and replace all occurrences of `192.168.1.225` with your Pi's IP address:
```bash
sed -i 's/192.168.1.225/YOUR_PI_IP/g' dashboard/picar_dashboard.html
```

Serve the dashboard with Python's built-in HTTP server:

```bash
cd dashboard
python3 -m http.server 8085
```

Open in your browser:
```
http://localhost:8085
```

Or from another device on your network:
```
http://YOUR_HOST_IP:8085
```

---

## Step 3 — Run the Navigator (Optional)

The navigator enables autonomous driving. Run it on your host machine:

```bash
# Install dependencies
pip install -r navigator/requirements.txt

# Edit the navigator to set your Pi's IP
nano navigator/picar_navigator.py
# Change AGENT_URL = "http://192.168.1.225:8000"
# to    AGENT_URL = "http://YOUR_PI_IP:8000"

# Run the navigator
python3 navigator/picar_navigator.py
```

Switch the dashboard to **AUTO** mode to activate autonomous driving.
Press `Ctrl+C` to stop the navigator and return to manual control.

---

## Step 4 — Ollama / Gemma 3 Integration (Optional)

If you have Ollama running on a separate machine with Gemma 3:

```bash
# Verify Ollama is accessible
curl http://YOUR_OLLAMA_IP:11434/api/tags
```

Edit the navigator:

```bash
nano navigator/picar_navigator.py
```

Set the Ollama URL:
```python
OLLAMA_URL   = "http://YOUR_OLLAMA_IP:11434"
OLLAMA_MODEL = "gemma3"
OLLAMA_ENABLED = True
```

---

## Calibration

Before running autonomous mode, calibrate your sensors:

See [calibration.md](calibration.md) for detailed instructions.

---

## Troubleshooting

### Camera feed not showing
- Wait 10-15 seconds after agent starts for Vilib to initialize
- Refresh the browser page
- Check: `curl http://YOUR_PI_IP:9000/mjpg` — should return data

### LiDAR not working
- Check USB connection: `ls /dev/ttyUSB*`
- Check permissions: `sudo usermod -a -G dialout $USER` then log out/in
- Check agent logs: `sudo journalctl -u picar-agent -n 50`

### Cliff detection triggering on normal floor
- Calibrate your grayscale threshold — see [calibration.md](calibration.md)
- Adjust `CLIFF_STOP` in `pi/picar_agent_v7.py`

### Left/right LiDAR readings swapped
- See [calibration.md](calibration.md) section 4

### Agent won't start
```bash
sudo journalctl -u picar-agent -n 50
```
Common cause: `os.getlogin()` failure — ensure the `os.getlogin = lambda: 'yourusername'`
patch at the top of `picar_agent_v7.py` uses your actual username.

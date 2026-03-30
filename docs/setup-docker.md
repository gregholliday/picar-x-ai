# Setup — Docker

Run the dashboard in a Docker container on your host machine. No Kubernetes required.

---

## Prerequisites

- Docker installed on your host machine
- Raspberry Pi agent running (see [setup-bare.md](setup-bare.md) Step 1)
- Docker Compose (usually included with Docker Desktop)

---

## Step 1 — Set Up the Pi Agent

Follow Step 1 from [setup-bare.md](setup-bare.md) to get the Pi agent running.

---

## Step 2 — Configure the Dashboard

Clone the repo on your host machine:

```bash
git clone https://github.com/gregholliday/picar-x-ai.git
cd picar-x-ai
```

Update the Pi IP address in the dashboard:

```bash
sed -i 's/192.168.1.225/YOUR_PI_IP/g' dashboard/picar_dashboard.html
```

---

## Step 3 — Build and Run with Docker Compose

```bash
# Build and start the dashboard container
docker compose up -d

# Verify it's running
docker compose ps
```

The dashboard is now available at:
```
http://localhost:8085
http://YOUR_HOST_IP:8085  ← from other devices on your network
```

---

## Step 4 — Run the Navigator (Optional)

The navigator runs directly on your host machine (not in Docker):

```bash
pip install -r navigator/requirements.txt

# Edit the navigator IP
sed -i 's/192.168.1.225/YOUR_PI_IP/g' navigator/picar_navigator.py

python3 navigator/picar_navigator.py
```

Switch the dashboard to **AUTO** mode to activate.

---

## Updating the Dashboard

When you make changes to `dashboard/picar_dashboard.html`:

```bash
docker compose down
docker compose up -d --build
```

---

## Useful Docker Commands

```bash
# View logs
docker compose logs -f

# Stop
docker compose down

# Restart
docker compose restart

# Check status
docker compose ps
```

---

## Ollama / Gemma 3 Integration (Optional)

If Ollama is running on a separate machine, edit `navigator/picar_navigator.py`:

```python
OLLAMA_URL    = "http://YOUR_OLLAMA_IP:11434"
OLLAMA_MODEL  = "gemma3"
OLLAMA_ENABLED = True
```

---

## Troubleshooting

### Port 8085 already in use
Edit `docker-compose.yml` and change the port mapping:
```yaml
ports:
  - "8090:80"   # change 8085 to any free port
```

### Dashboard can't reach Pi agent
The browser makes direct requests to the Pi agent from your browser,
not through Docker. Ensure:
- Pi agent is running: `curl http://YOUR_PI_IP:8000/api/status`
- Pi IP is correctly set in `dashboard/picar_dashboard.html`
- Pi and host are on the same network

### Camera feed not showing
The camera stream comes directly from the Pi (port 9000), not through Docker.
Ensure Vilib has had time to start (wait ~15 seconds after agent starts).

# ── PiCar-X AI Configuration ──────────────────────────────────────────────────
# This file contains default settings safe to commit to GitHub.
# For machine-specific settings (IP addresses, credentials, tuned values),
# create a config_local.py file in the same directory — it will be auto-loaded
# and is gitignored so it never gets pushed to the repo.
#
# Example config_local.py:
#   PI_IP         = "192.168.1.225"
#   OLLAMA_IP     = "192.168.1.57"
#   STEERING_TRIM = -2
#   OLLAMA_MODEL  = "qwen2.5vl:latest"

# ── Network ────────────────────────────────────────────────────────────────────
PI_IP        = "YOUR_PI_IP"       # IP address of your Raspberry Pi
OLLAMA_IP    = "YOUR_OLLAMA_IP"   # IP of machine running Ollama
OLLAMA_PORT  = 11434
OLLAMA_MODEL = "qwen2.5vl:latest" # Vision model for goal detection

# ── Pi Username ────────────────────────────────────────────────────────────────
PI_USERNAME  = "pi"               # Your username on the Raspberry Pi

# ── Agent Settings ─────────────────────────────────────────────────────────────
AGENT_PORT   = 8000
CAMERA_PORT  = 9000

# ── LiDAR Settings ─────────────────────────────────────────────────────────────
LIDAR_PORT     = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800

# ── Steering Trim ──────────────────────────────────────────────────────────────
# Negative = corrects rightward drift, Positive = corrects leftward drift
# Start with small values (-1, -2, -3) and test until the car drives straight.
STEERING_TRIM = 0

# ── Safety Thresholds ──────────────────────────────────────────────────────────
ULTRASONIC_STOP = 15    # cm  — emergency stop distance
ULTRASONIC_SLOW = 30    # cm  — slow down distance
CLIFF_STOP      = 100   # ADC — below this = cliff detected
CLIFF_WARN      = 500   # ADC — below this = near edge

# ── Navigator Settings ─────────────────────────────────────────────────────────
BASE_SPEED    = 30      # Normal cruising speed (0-100)
SLOW_SPEED    = 20      # Obstacle avoidance speed
TURN_ANGLE    = 35      # Steering angle for turns (degrees)

# LiDAR distance thresholds (mm)
FRONT_CLEAR   = 800     # Path is clear, drive forward
FRONT_CAUTION = 500     # Slow down
SIDE_CLEAR    = 300     # Minimum side clearance for turns

# ── Vision / Goal Detection ────────────────────────────────────────────────────
VISION_ENABLED        = True
VISION_INTERVAL       = 3      # seconds between vision queries while searching
VISION_TIMEOUT        = 30     # seconds to wait for model response
TARGET_CONFIRM_NEEDED = 2      # consecutive confirmations before approaching
APPROACH_STOP_CM      = 20     # ultrasonic distance to stop when approaching

# Search pattern
SEARCH_FORWARD_STEPS  = 25     # loop iterations to drive forward during search
SEARCH_ROTATE_STEPS   = 15     # loop iterations to rotate during search

# ── LiDAR Orientation ──────────────────────────────────────────────────────────
# If left/right readings are swapped, set this to True
LIDAR_SWAP_LR = False

# ── Local Overrides ────────────────────────────────────────────────────────────
# Load machine-specific settings from config_local.py if it exists.
# This file is gitignored — put your IPs, credentials, and tuned values here.
try:
    from config_local import *  # noqa: F401,F403
except ImportError:
    pass

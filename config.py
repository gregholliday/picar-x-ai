# ── PiCar-X AI Configuration ──────────────────────────────────────────────────
# Edit this file to match your setup before running the agent or navigator.
# This is the only file you need to edit for basic configuration.

# ── Network ────────────────────────────────────────────────────────────────────
PI_IP        = "YOUR_PI_IP"       # IP address of your Raspberry Pi
OLLAMA_IP    = "YOUR_OLLAMA_IP"   # IP of machine running Ollama (optional)
OLLAMA_PORT  = 11434
OLLAMA_MODEL = "gemma3"

# ── Pi Username ────────────────────────────────────────────────────────────────
# Your username on the Raspberry Pi (default is 'pi')
PI_USERNAME  = "pi"

# ── Agent Settings ─────────────────────────────────────────────────────────────
AGENT_PORT   = 8000
CAMERA_PORT  = 9000

# ── LiDAR Settings ─────────────────────────────────────────────────────────────
LIDAR_PORT     = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800

# ── Steering Trim ──────────────────────────────────────────────────────────────
# If the car drifts left or right when driving straight, adjust this value.
# Negative = corrects rightward drift (turns wheels slightly left)
# Positive = corrects leftward drift (turns wheels slightly right)
# Start with small values (-1, -2, -3) and test until the car drives straight.
STEERING_TRIM = 0

# ── Safety Thresholds ──────────────────────────────────────────────────────────
# Ultrasonic sensor (cm)
ULTRASONIC_STOP = 15    # Emergency stop distance
ULTRASONIC_SLOW = 30    # Slow down distance

# Grayscale cliff detection (ADC value)
# Run calibration to find the right value for your floor:
#   Normal floor: typically 1400-1500
#   Cliff/edge:   typically 10-50
CLIFF_STOP      = 100   # Below this = cliff detected
CLIFF_WARN      = 500   # Below this = near edge

# ── Navigator Settings ─────────────────────────────────────────────────────────
BASE_SPEED    = 40      # Normal cruising speed (0-100)
SLOW_SPEED    = 25      # Obstacle avoidance speed
TURN_ANGLE    = 35      # Steering angle for turns (degrees)

# LiDAR distance thresholds (mm)
FRONT_CLEAR   = 600     # Path is clear, drive forward
FRONT_CAUTION = 350     # Slow down
SIDE_CLEAR    = 300     # Minimum side clearance for turns

# ── LiDAR Orientation ──────────────────────────────────────────────────────────
# If left/right readings are swapped, set this to True
# Test: place object on RIGHT side, check if 'right' value is small
LIDAR_SWAP_LR = False

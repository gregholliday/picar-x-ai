# Sensor Calibration

## Before You Start

Stop the agent if it's running, since calibration needs exclusive access to the hardware:

```bash
sudo systemctl stop picar-agent
```

Restart when done:
```bash
sudo systemctl start picar-agent
```

---

## 1. Servo Centering

The steering, camera pan, and camera tilt servos need to be centered so the car drives straight and the camera points forward.

```bash
python3 -c "
from picarx import Picarx
import time
px = Picarx()
time.sleep(0.5)
px.set_dir_servo_angle(0)
px.set_cam_pan_angle(0)
px.set_cam_tilt_angle(0)
print('Servos centered. Check alignment.')
time.sleep(3)
"
```

Look at the car from above:
- **Steering**: wheels should point straight forward
- **Camera pan**: camera should point straight ahead
- **Camera tilt**: camera should be level

If any servo is off-center, use the SunFounder calibration tool to adjust the offset values.

---

## 2. Grayscale Cliff Detection

The grayscale sensors detect cliffs and edges by measuring surface reflectivity. You need to establish what values your specific floor returns.

### Step 1 — Measure Normal Floor

Place the car flat on your normal driving surface:

```bash
python3 -c "
from picarx import Picarx
import time
px = Picarx()
time.sleep(0.5)
print('Normal floor readings:')
for i in range(10):
    gs = px.grayscale.read()
    print(f'  {gs}')
    time.sleep(0.3)
"
```

Note the typical values — this is your **normal floor baseline**.

### Step 2 — Measure Edge/Cliff

Carefully hold the car over a table edge or significant drop:

```bash
python3 -c "
from picarx import Picarx
import time
px = Picarx()
time.sleep(0.5)
print('Edge/cliff readings:')
for i in range(10):
    gs = px.grayscale.read()
    print(f'  {gs}')
    time.sleep(0.3)
"
```

### Step 3 — Set Threshold

The `CLIFF_STOP` threshold in `pi/picar_agent_v7.py` should be set between your normal floor value and your cliff value.

Example:
```
Normal floor:  ~1400-1500
Cliff/edge:    ~10-20
Good threshold: 100  (well below normal, well above cliff)
```

Update in `pi/picar_agent_v7.py`:
```python
CLIFF_STOP = 100   # Adjust based on your calibration
```

---

## 3. Ultrasonic Validation

Test that the ultrasonic sensor reads accurately:

```bash
python3 -c "
from picarx import Picarx
import time
px = Picarx()
time.sleep(0.5)
print('Ultrasonic readings (place objects at known distances):')
for i in range(20):
    dist = px.ultrasonic.read()
    print(f'  {dist}cm')
    time.sleep(0.5)
"
```

Place an object at 30cm, 50cm, and 100cm in front of the car and compare readings. If readings are consistently off, note the offset for your environment.

The agent filters out readings below 2cm as invalid, so very close readings will be ignored.

---

## 4. LiDAR Orientation

Verify the LiDAR is oriented correctly:

```bash
# Start the agent
sudo systemctl start picar-agent

# Place an object on the RIGHT side of the car
curl http://localhost:8000/api/sensors
```

The `right` distance should be small (close to the object). If it shows under `left`, swap the angle ranges in `pi/picar_agent_v7.py`:

```python
# If left/right are swapped, change:
left  = [m for m in scan if 225 <  m["angle"] <  315]
right = [m for m in scan if 45  <  m["angle"] <= 135]

# To:
left  = [m for m in scan if 45  <  m["angle"] <= 135]
right = [m for m in scan if 225 <  m["angle"] <  315]
```

This change needs to be made in every place quadrants are defined in the agent file, and also in `navigator/picar_navigator.py`.

---

## 5. Autonomous Driving Thresholds

Once sensors are calibrated, tune the autonomous driving thresholds in `pi/picar_agent_v7.py`:

```python
ULTRASONIC_STOP = 15    # cm  — stop immediately
ULTRASONIC_SLOW = 30    # cm  — slow down
CLIFF_STOP      = 100   # ADC — cliff detected
```

And in `navigator/picar_navigator.py`:

```python
FRONT_CLEAR   = 600    # mm — clear to drive forward
FRONT_CAUTION = 350    # mm — slow down
SIDE_CLEAR    = 300    # mm — side clear enough to turn into
BASE_SPEED    = 40     # cruising speed
SLOW_SPEED    = 25     # obstacle avoidance speed
```

Start conservative (lower speeds, larger distances) and tune from there.

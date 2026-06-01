# robot_motor_controller

Arduino Mega 2560 motor controller for tracked/mecanum robot chassis.

## Architecture

```
Fedora (navigator scripts)
    ↓ WiFi
Raspberry Pi 4B (picar agent / robot agent)
    ↓ Serial USB (ttyACM0, 9600 baud)
Arduino Mega 2560 (this sketch)
    ↓ PWM + direction signals
2× DFR0601 Dual Motor Drivers (12A per channel)
    ↓ 12V power
4× DFRobot 28PA51G motors (12V, 146RPM, 10kg·cm, 51:1 planetary)
    ↑ Hall encoder feedback (663 PPR via FIT0324 adapter)
```

## Hardware

### Motors
- **Model:** DFRobot 28PA51G / 28ZYT47-1280ME13
- **Voltage:** 12V DC
- **Output speed:** 146 RPM (after 51:1 planetary gearbox)
- **Torque:** 10 kg·cm rated
- **Stall current:** 3.6A
- **Encoder:** Hall effect, 13 PPR (663 PPR on output shaft)
- **Shaft:** 6mm D-shaft
- **Size:** 123×36×36mm

### Encoder Adapter
- **Model:** DFRobot FIT0324
- **Purpose:** Pull-up resistors for open-collector Hall encoder output
- **Logic voltage:** 5V (compatible with Arduino)

### Motor Drivers
- **Model:** DFRobot DFR0601 Dual-Channel DC Motor Driver-12A
- **Voltage:** 6.5V-37V
- **Continuous current:** 12A per channel
- **Peak current:** 70A (100ms)
- **Signal:** 3V-5V PWM + direction
- **Protection:** Over-voltage, under-voltage, overheat

### Power
- **Battery:** 4S LiPo 14.8V nominal
- **Motor supply:** Buck converter → 12V → DFR0601 drivers
- **Arduino supply:** Buck converter → 12V → Arduino Mega Vin
- **Pi supply:** Buck converter → 5V/3A → Raspberry Pi

---

## Pin Assignments

### Driver 1 — Left Side Motors

| Signal | Arduino Pin | Notes |
|--------|------------|-------|
| M1 direction A | 4 | Left front (or left tread) |
| M1 direction B | 5 | Left front (or left tread) |
| M1 PWM speed | 2 | PWM capable |
| M2 direction A | 6 | Left rear (mecanum only) |
| M2 direction B | 7 | Left rear (mecanum only) |
| M2 PWM speed | 3 | PWM capable |

### Driver 2 — Right Side Motors

| Signal | Arduino Pin | Notes |
|--------|------------|-------|
| M3 direction A | 8 | Right front (or right tread) |
| M3 direction B | 9 | Right front (or right tread) |
| M3 PWM speed | 10 | PWM capable |
| M4 direction A | 11 | Right rear (mecanum only) |
| M4 direction B | 12 | Right rear (mecanum only) |
| M4 PWM speed | 44 | PWM capable (avoids pin 13 LED) |

### Encoders (via FIT0324 adapter)

| Signal | Arduino Pin | Notes |
|--------|------------|-------|
| Left encoder A | 18 | Interrupt 5 |
| Left encoder B | 19 | Interrupt 4 |
| Right encoder A | 20 | Interrupt 3 |
| Right encoder B | 21 | Interrupt 2 |

### DFR0601 Wiring

```
DFR0601 connector:
  M+  → Motor positive terminal
  M-  → Motor negative terminal
  VIN → 12V from buck converter
  GND → Common ground
  INA → Direction pin A (from Arduino)
  INB → Direction pin B (from Arduino)
  PWM → Speed pin (from Arduino)
  GND → Common ground
```

### FIT0324 Encoder Adapter Wiring

```
FIT0324 connector (6-pin JST):
  Red    → 5V
  Black  → GND
  Yellow → Hall sensor A → Arduino interrupt pin
  Green  → Hall sensor B → Arduino digital pin
  Blue   → Motor power (connects to motor driver output)
  White  → Motor power (connects to motor driver output)
```

---

## Serial Protocol

**Baud rate:** 9600  
**Format:** JSON, newline terminated  
**Connection:** USB (appears as /dev/ttyACM0 on Pi/Linux)

### Commands (Pi → Arduino)

```json
// Drive motors (speed: -100 to +100, negative = reverse)
{"cmd":"drive","left":50,"right":50}

// Stop all motors
{"cmd":"stop"}

// Request status
{"cmd":"status"}

// Zero encoder counts
{"cmd":"reset_encoders"}

// Turn in place by degrees (positive=right, negative=left)
{"cmd":"turn","degrees":90,"speed":40}

// Drive straight by distance in mm
{"cmd":"drive_distance","mm":500,"speed":40}
```

### Responses (Arduino → Pi)

```json
// Status (sent every 100ms automatically + on request)
{"status":"ok","left_enc":1234,"right_enc":1234,"left_speed":50,"right_speed":50,"left_mm":794.5,"right_mm":794.5}

// Action complete
{"status":"done","action":"turn","degrees":90,"left_enc":234,"right_enc":-234}
{"status":"done","action":"drive_distance","mm":500,"left_enc":776,"right_enc":779}

// Error
{"status":"error","msg":"unknown command"}

// Startup
{"status":"ready","msg":"robot_motor_controller v1"}
```

---

## Encoder Math

```
Motor encoder:     663 PPR (pulses per revolution on output shaft)
Wheel diameter:    ~100mm (measure actual wheel and update)
Wheel circumference: PI × diameter = ~314mm
mm per pulse:      314 / 663 = 0.474mm

Wheelbase:         ~200mm (measure actual chassis and update)
Pulses per 360°:   (PI × wheelbase) / mm_per_pulse
```

**⚠️ Important:** Update `MM_PER_PULSE` and `WHEELBASE_MM` constants in the sketch
after measuring your actual wheels and chassis. Dead reckoning accuracy depends on these values.

---

## Dependencies

| Library | Version | Install via |
|---------|---------|-------------|
| ArduinoJson | 6.x | Arduino Library Manager |

Install in Arduino IDE: Sketch → Include Library → Manage Libraries → search "ArduinoJson" by Benoit Blanchon

---

## Chassis Configurations

This sketch supports two chassis configurations:

### Tracked (2 motors)
- M1 = Left tread motor
- M3 = Right tread motor  
- M2, M4 unused (comment out or set to same signal as M1/M3)
- Update `WHEELBASE_MM` to distance between tread centerlines

### Mecanum (4 motors)
- M1 = Front left
- M2 = Rear left
- M3 = Front right
- M4 = Rear right
- All 4 motors active
- Update `WHEELBASE_MM` to distance between wheel centerlines

**Mecanum wheel orientation (viewed from above):**
```
Front-Left (M1): rollers \    Front-Right (M3): rollers /
Rear-Left  (M2): rollers /    Rear-Right  (M4): rollers \
```

---

## Calibration

After chassis assembly, calibrate these constants:

1. **MM_PER_PULSE** — measure actual wheel circumference, divide by 663
2. **WHEELBASE_MM** — measure distance between left and right wheel centerlines
3. **Verify turn accuracy** — command 90° turn, measure actual angle
4. **Verify distance accuracy** — command 500mm drive, measure actual distance

---

## Files

| File | Location | Purpose |
|------|----------|---------|
| `robot_motor_controller.ino` | `arduino/robot_motor_controller/` | This sketch |
| `arduino_interface.py` | `navigator/` | Pi-side serial interface |
| `robot_agent.py` | `pi/` | Pi agent (future — replaces picar_agent.py) |

---

## Known Issues / TODO

- [ ] Measure actual wheel diameter and update MM_PER_PULSE
- [ ] Measure actual wheelbase and update WHEELBASE_MM
- [ ] Test encoder direction (may need to swap A/B pins if counting backwards)
- [ ] Add mecanum strafing commands
- [ ] Add PID speed control for better straight-line tracking
- [ ] Add battery voltage monitoring via analog pin

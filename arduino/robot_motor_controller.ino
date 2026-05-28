/*
 * robot_motor_controller.ino
 * 
 * Arduino Mega 2560 — Motor Controller for tracked/mecanum robot
 * 
 * Architecture:
 *   Pi (navigation brain) <--> Serial <--> Arduino Mega (motor controller)
 *   Arduino --> DFR0601 motor drivers --> 28PA51G motors
 *   28PA51G Hall encoders --> FIT0324 adapter --> Arduino interrupt pins
 * 
 * Hardware connections (DFR0601 dual motor driver):
 *   Driver 1 (Left side):
 *     INA1 (direction) --> Pin 4
 *     INB1 (direction) --> Pin 5  
 *     PWM1 (speed)     --> Pin 2  (PWM pin)
 *     INA2 (direction) --> Pin 6
 *     INB2 (direction) --> Pin 7
 *     PWM2 (speed)     --> Pin 3  (PWM pin)
 * 
 *   Driver 2 (Right side):
 *     INA1 (direction) --> Pin 8
 *     INB1 (direction) --> Pin 9
 *     PWM1 (speed)     --> Pin 10 (PWM pin)
 *     INA2 (direction) --> Pin 11
 *     INB2 (direction) --> Pin 12
 *     PWM2 (speed)     --> Pin 13 (PWM pin)  -- Note: avoid if using LED
 *
 * Encoder connections (FIT0324 adapter, 5V):
 *   Left Motor encoder A  --> Pin 18 (interrupt)
 *   Left Motor encoder B  --> Pin 19 (interrupt)
 *   Right Motor encoder A --> Pin 20 (interrupt)
 *   Right Motor encoder B --> Pin 21 (interrupt)
 * 
 * Serial protocol (9600 baud, JSON):
 *   Commands from Pi:
 *     {"cmd":"drive","left":50,"right":50}      -- drive both sides
 *     {"cmd":"stop"}                             -- stop all motors
 *     {"cmd":"status"}                           -- request status
 *     {"cmd":"reset_encoders"}                   -- zero encoder counts
 *     {"cmd":"turn","degrees":90}                -- turn by degrees (dead reckoning)
 *     {"cmd":"drive_distance","mm":500,"speed":50} -- drive distance in mm
 * 
 *   Responses to Pi:
 *     {"status":"ok","left_enc":1234,"right_enc":1234,"left_speed":50,"right_speed":50}
 *     {"status":"done","action":"turn"}          -- action completed
 *     {"status":"done","action":"drive_distance"}
 *     {"status":"error","msg":"..."}
 * 
 * Speed values: -100 to +100 (negative = reverse)
 * Encoder counts: positive = forward
 */

#include <ArduinoJson.h>

// ── Pin definitions ────────────────────────────────────────────────────────────
// Driver 1 — Left side motors (M1=left front, M2=left rear for mecanum)
//           or Left tread motor for tracked
#define M1_INA  4
#define M1_INB  5
#define M1_PWM  2

#define M2_INA  6
#define M2_INB  7
#define M2_PWM  3

// Driver 2 — Right side motors
#define M3_INA  8
#define M3_INB  9
#define M3_PWM  10

#define M4_INA  11
#define M4_INB  12
#define M4_PWM  44  // Use pin 44 for PWM to avoid LED conflict on 13

// Encoder pins (interrupt capable on Mega: 2,3,18,19,20,21)
#define ENC_LEFT_A   18
#define ENC_LEFT_B   19
#define ENC_RIGHT_A  20
#define ENC_RIGHT_B  21

// ── Constants ──────────────────────────────────────────────────────────────────
// 663 pulses per revolution, ~136mm wheel diameter
// Circumference = PI * 136 = 427mm
// mm per pulse = 427 / 663 = 0.644mm
const float MM_PER_PULSE = 0.644;

// Wheelbase for turn calculations
// Distance between left and right wheel centers
// Update this after measuring your actual chassis
const float WHEELBASE_MM = 200.0;  // ~8 inches, adjust after build

// Pulses needed for 360 degree turn
// Arc length = PI * wheelbase
// Pulses = arc_length / mm_per_pulse
const float PULSES_PER_360 = (PI * WHEELBASE_MM) / MM_PER_PULSE;

// ── Encoder state ──────────────────────────────────────────────────────────────
volatile long left_encoder  = 0;
volatile long right_encoder = 0;
volatile bool left_dir      = true;   // true = forward
volatile bool right_dir     = true;

// ── Motor state ────────────────────────────────────────────────────────────────
int current_left_speed  = 0;
int current_right_speed = 0;

// ── Serial buffer ──────────────────────────────────────────────────────────────
String serial_buffer = "";


// ── Encoder interrupt handlers ─────────────────────────────────────────────────
void left_encoder_isr() {
    if (left_dir) {
        left_encoder++;
    } else {
        left_encoder--;
    }
}

void right_encoder_isr() {
    if (right_dir) {
        right_encoder++;
    } else {
        right_encoder--;
    }
}


// ── Motor control functions ────────────────────────────────────────────────────
void set_motor(int ina, int inb, int pwm_pin, int speed) {
    speed = constrain(speed, -100, 100);
    int pwm_val = map(abs(speed), 0, 100, 0, 255);

    if (speed > 0) {
        digitalWrite(ina, HIGH);
        digitalWrite(inb, LOW);
    } else if (speed < 0) {
        digitalWrite(ina, LOW);
        digitalWrite(inb, HIGH);
    } else {
        // Brake
        digitalWrite(ina, LOW);
        digitalWrite(inb, LOW);
    }
    analogWrite(pwm_pin, pwm_val);
}

void drive_left(int speed) {
    left_dir = (speed >= 0);
    set_motor(M1_INA, M1_INB, M1_PWM, speed);
    set_motor(M2_INA, M2_INB, M2_PWM, speed);
    current_left_speed = speed;
}

void drive_right(int speed) {
    right_dir = (speed >= 0);
    set_motor(M3_INA, M3_INB, M3_PWM, speed);
    set_motor(M4_INA, M4_INB, M4_PWM, speed);
    current_right_speed = speed;
}

void stop_all() {
    drive_left(0);
    drive_right(0);
}


// ── Dead reckoning actions ─────────────────────────────────────────────────────
void turn_degrees(float degrees, int speed) {
    /*
     * Turn in place by specified degrees using encoder dead reckoning.
     * Positive degrees = turn right, negative = turn left.
     */
    long target_pulses = abs((long)(PULSES_PER_360 * (abs(degrees) / 360.0)));
    long start_left    = left_encoder;
    long start_right   = right_encoder;

    if (degrees > 0) {
        // Turn right: left forward, right backward
        drive_left(speed);
        drive_right(-speed);
    } else {
        // Turn left: left backward, right forward
        drive_left(-speed);
        drive_right(speed);
    }

    while (true) {
        long left_moved  = abs(left_encoder  - start_left);
        long right_moved = abs(right_encoder - start_right);
        long avg_moved   = (left_moved + right_moved) / 2;

        if (avg_moved >= target_pulses) break;
        delay(5);
    }

    stop_all();

    // Send completion response
    StaticJsonDocument<128> doc;
    doc["status"] = "done";
    doc["action"] = "turn";
    doc["degrees"] = degrees;
    doc["left_enc"]  = left_encoder;
    doc["right_enc"] = right_encoder;
    serializeJson(doc, Serial);
    Serial.println();
}

void drive_distance(float mm, int speed) {
    /*
     * Drive straight for specified distance in mm.
     * Uses both encoders averaged for accuracy.
     * Simple proportional correction to keep straight.
     */
    long target_pulses = abs((long)(mm / MM_PER_PULSE));
    long start_left    = left_encoder;
    long start_right   = right_encoder;
    int  dir           = (mm > 0) ? 1 : -1;

    drive_left(speed  * dir);
    drive_right(speed * dir);

    while (true) {
        long left_moved  = abs(left_encoder  - start_left);
        long right_moved = abs(right_encoder - start_right);
        long avg_moved   = (left_moved + right_moved) / 2;

        if (avg_moved >= target_pulses) break;

        // Simple straight-line correction
        long diff = left_moved - right_moved;
        if (abs(diff) > 10) {
            int correction = constrain((int)(diff / 5), -10, 10);
            set_motor(M1_INA, M1_INB, M1_PWM, (speed - correction) * dir);
            set_motor(M2_INA, M2_INB, M2_PWM, (speed - correction) * dir);
            set_motor(M3_INA, M3_INB, M3_PWM, (speed + correction) * dir);
            set_motor(M4_INA, M4_INB, M4_PWM, (speed + correction) * dir);
        }

        delay(5);
    }

    stop_all();

    // Send completion response
    StaticJsonDocument<128> doc;
    doc["status"]    = "done";
    doc["action"]    = "drive_distance";
    doc["mm"]        = mm;
    doc["left_enc"]  = left_encoder;
    doc["right_enc"] = right_encoder;
    serializeJson(doc, Serial);
    Serial.println();
}


// ── Serial command handler ─────────────────────────────────────────────────────
void handle_command(String json_str) {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, json_str);

    if (err) {
        Serial.println("{\"status\":\"error\",\"msg\":\"invalid json\"}");
        return;
    }

    const char* cmd = doc["cmd"];

    if (strcmp(cmd, "drive") == 0) {
        int left  = doc["left"]  | 0;
        int right = doc["right"] | 0;
        drive_left(left);
        drive_right(right);
        send_status();

    } else if (strcmp(cmd, "stop") == 0) {
        stop_all();
        send_status();

    } else if (strcmp(cmd, "status") == 0) {
        send_status();

    } else if (strcmp(cmd, "reset_encoders") == 0) {
        left_encoder  = 0;
        right_encoder = 0;
        Serial.println("{\"status\":\"ok\",\"msg\":\"encoders reset\"}");

    } else if (strcmp(cmd, "turn") == 0) {
        float degrees = doc["degrees"] | 0.0;
        int   speed   = doc["speed"]   | 40;
        turn_degrees(degrees, speed);

    } else if (strcmp(cmd, "drive_distance") == 0) {
        float mm    = doc["mm"]    | 0.0;
        int   speed = doc["speed"] | 40;
        drive_distance(mm, speed);

    } else {
        Serial.println("{\"status\":\"error\",\"msg\":\"unknown command\"}");
    }
}

void send_status() {
    StaticJsonDocument<256> doc;
    doc["status"]       = "ok";
    doc["left_enc"]     = left_encoder;
    doc["right_enc"]    = right_encoder;
    doc["left_speed"]   = current_left_speed;
    doc["right_speed"]  = current_right_speed;
    doc["left_mm"]      = (float)(left_encoder  * MM_PER_PULSE);
    doc["right_mm"]     = (float)(right_encoder * MM_PER_PULSE);
    serializeJson(doc, Serial);
    Serial.println();
}


// ── Setup ──────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(9600);

    // Motor control pins
    int motor_pins[] = {M1_INA, M1_INB, M1_PWM,
                        M2_INA, M2_INB, M2_PWM,
                        M3_INA, M3_INB, M3_PWM,
                        M4_INA, M4_INB, M4_PWM};

    for (int pin : motor_pins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    // Encoder pins with pull-up (FIT0324 has built-in pull-ups but no harm)
    pinMode(ENC_LEFT_A,  INPUT_PULLUP);
    pinMode(ENC_LEFT_B,  INPUT_PULLUP);
    pinMode(ENC_RIGHT_A, INPUT_PULLUP);
    pinMode(ENC_RIGHT_B, INPUT_PULLUP);

    // Attach encoder interrupts
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  left_encoder_isr,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), right_encoder_isr, CHANGE);

    stop_all();

    Serial.println("{\"status\":\"ready\",\"msg\":\"robot_motor_controller v1\"}");
}


// ── Main loop ──────────────────────────────────────────────────────────────────
void loop() {
    // Read serial commands
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            if (serial_buffer.length() > 0) {
                handle_command(serial_buffer);
                serial_buffer = "";
            }
        } else {
            serial_buffer += c;
        }
    }

    // Send periodic status every 100ms (10Hz) so Pi can monitor encoders
    static unsigned long last_status = 0;
    if (millis() - last_status >= 100) {
        send_status();
        last_status = millis();
    }
}

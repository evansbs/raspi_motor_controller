/*
 * Antenna Rotator Firmware for Arduino Nano
 *
 * Hardware:
 *   - Arduino Nano (ATmega328P)
 *   - MPU6050 IMU on I2C (address 0x68) — accelerometer/gyro for pitch & roll
 *   - QMC5883L magnetometer on I2C (address 0x0D) — compass heading / azimuth
 *     (jumper USE_MAGNETOMETER to 0 if no mag chip is fitted; AZ falls back to
 *      dead-reckoning based on known motor speed)
 *   - L298N dual H-bridge motor driver:
 *       Motor A → Azimuth motor    (AZ_IN1 / AZ_IN2)
 *       Motor B → Elevation motor  (EL_IN1 / EL_IN2)
 *
 * Serial Protocol  (115200 baud, ASCII line-oriented, \n terminated)
 * ─────────────────────────────────────────────────────────────────
 * Commands (Pi → Arduino):
 *   STATUS                   Request immediate status report
 *   AZ <degrees>             Set AZ target and start moving (0–360)
 *   EL <degrees>             Set EL target and start moving (−90–90)
 *   MOVE AZ <f> EL <f>       Move to both targets simultaneously
 *   STOP                     Stop all motors immediately
 *   DECL <degrees>           Set magnetic declination correction
 *
 * Responses (Arduino → Pi):
 *   INIT MPU6050=<OK|FAIL> QMC5883L=<OK|FAIL>   — sent once at startup
 *   POS AZ=<f> EL=<f> HDG=<f> PITCH=<f> ROLL=<f> MOVING=<0|1>
 *       Broadcast automatically every STATUS_INTERVAL ms.
 *       Also sent immediately in response to STATUS.
 *   OK <echo of command>     — command accepted
 *   ERR <message>            — command rejected or parse error
 */

#include <Wire.h>
#include <math.h>

// ============================================================
// Build-time configuration
// ============================================================
// Set to 0 if no QMC5883L is connected; AZ will use dead-reckoning.
#define USE_MAGNETOMETER 1

// ============================================================
// Pin Assignments  (change to match your wiring)
// ============================================================
#define AZ_IN1  9   // L298N IN1  → OUT3  AZ+
#define AZ_IN2  10  // L298N IN2  → OUT4  AZ-
#define EL_IN1  5   // L298N IN3  → OUT1  EL+
#define EL_IN2  6   // L298N IN4  → OUT2  EL-
// If your L298N board has separate ENA/ENB header pins (not jumpered),
// uncomment the two lines below and wire PWM pins to them.
// #define AZ_ENA  3
// #define EL_ENB  9

// ============================================================
// I2C addresses
// ============================================================
#define MPU6050_ADDR  0x68
#define QMC5883L_ADDR 0x0D

// ============================================================
// Tunable parameters
// ============================================================
#define SERIAL_BAUD      115200UL
#define STATUS_INTERVAL  500      // ms between automatic POS broadcasts
#define IMU_INTERVAL     100      // ms between IMU reads
#define MOTOR_DEADBAND   2.0f     // degrees — stop motors within this error
#define AZ_DEG_PER_SEC   3.6f     // ~0.6 rpm → 3.6 °/s (dead-reckoning fallback)
#define EL_DEG_PER_SEC   3.6f     // same motor speed assumed for elevation

// Magnetic declination for your location (degrees, positive = east).
// Can be updated at runtime with the DECL command.
float magDeclination = 0.0f;

// ============================================================
// Global state
// ============================================================
float currentAz     = 0.0f;   // Compass-derived AZ (0–360)
float currentEl     = 0.0f;   // Pitch-derived EL
float currentHdg    = 0.0f;   // Tilt-compensated heading
float currentPitch  = 0.0f;   // Accel pitch (degrees)
float currentRoll   = 0.0f;   // Accel roll  (degrees)

float targetAz  = 0.0f;
float targetEl  = 0.0f;
bool  movingAz  = false;
bool  movingEl  = false;

// Dead-reckoning fallback when no magnetometer
float drAz = 0.0f;            // dead-reckoned azimuth
unsigned long drAzLastMs = 0;
int   drAzDir = 0;            // +1 CW, -1 CCW, 0 stopped

// Timing
unsigned long lastStatusMs = 0;
unsigned long lastImuMs    = 0;

// Command accumulation buffer
#define CMD_BUF_SIZE 80
char    cmdBuf[CMD_BUF_SIZE];
uint8_t cmdLen = 0;

// MPU6050 hardware availability
bool mpuOk = false;
bool magOk = false;

// ============================================================
// MPU-6050 helpers
// ============================================================

static bool mpu6050_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool mpu6050_init() {
    // Wake from sleep, use internal 8 MHz oscillator
    if (!mpu6050_write(0x6B, 0x00)) return false;
    // ACCEL_CONFIG: ±2 g full-scale
    mpu6050_write(0x1C, 0x00);
    // DLPF ~44 Hz
    mpu6050_write(0x1A, 0x03);
    return true;
}

// Returns true and fills ax/ay/az in units of g.
bool mpu6050_read_accel(float &ax, float &ay, float &azv) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);   // ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)6, (uint8_t)true) < 6) return false;

    int16_t raw_ax = ((int16_t)Wire.read() << 8) | Wire.read();
    int16_t raw_ay = ((int16_t)Wire.read() << 8) | Wire.read();
    int16_t raw_az = ((int16_t)Wire.read() << 8) | Wire.read();

    const float scale = 1.0f / 16384.0f;   // ±2 g
    ax  = raw_ax * scale;
    ay  = raw_ay * scale;
    azv = raw_az * scale;
    return true;
}

// ============================================================
// QMC5883L helpers
// ============================================================
#if USE_MAGNETOMETER

static bool qmc_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool qmc5883l_init() {
    // Software reset
    if (!qmc_write(0x0A, 0x80)) return false;
    delay(5);
    // Set/Reset period register (must be 0x01)
    if (!qmc_write(0x0B, 0x01)) return false;
    // Control Register 1:
    //   MODE=Continuous(01), ODR=10 Hz(00), RNG=8 G(01), OSR=512(00)
    //   Byte = 0b00010001 = 0x11  → RNG=8G gives 3000 LSB/gauss
    if (!qmc_write(0x09, 0x11)) return false;
    return true;
}

// Returns true and fills mx/my/mz in raw LSB counts.
bool qmc5883l_read(int16_t &mx, int16_t &my, int16_t &mz) {
    // Check data-ready bit in status register (0x06)
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(0x06);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)QMC5883L_ADDR, (uint8_t)1, (uint8_t)true);
    if (!Wire.available()) return false;
    uint8_t status = Wire.read();
    if (!(status & 0x01)) return false;   // DRDY not set

    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(0x00);   // Data starts at register 0
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)QMC5883L_ADDR, (uint8_t)6, (uint8_t)true) < 6) return false;

    // QMC5883L is little-endian
    mx = (int16_t)(Wire.read() | ((uint16_t)Wire.read() << 8));
    my = (int16_t)(Wire.read() | ((uint16_t)Wire.read() << 8));
    mz = (int16_t)(Wire.read() | ((uint16_t)Wire.read() << 8));
    return true;
}

#endif  // USE_MAGNETOMETER

// ============================================================
// IMU fusion: compute heading, pitch, roll
// ============================================================
void updateIMU() {
    float ax, ay, azv;
    if (!mpu6050_read_accel(ax, ay, azv)) return;

    // Pitch and roll from accelerometer (static tilt, no gyro integration)
    float pitch_rad = atan2f(-ax, sqrtf(ay * ay + azv * azv));
    float roll_rad  = atan2f(ay,  azv);

    currentPitch = pitch_rad * (180.0f / (float)M_PI);
    currentRoll  = roll_rad  * (180.0f / (float)M_PI);
    currentEl    = currentPitch;    // Elevation ≈ pitch angle

#if USE_MAGNETOMETER
    if (magOk) {
        int16_t mx16, my16, mz16;
        if (qmc5883l_read(mx16, my16, mz16)) {
            float mx = (float)mx16;
            float my = (float)my16;
            float mz = (float)mz16;

            // Tilt-compensated heading
            float cp = cosf(pitch_rad), sp = sinf(pitch_rad);
            float cr = cosf(roll_rad),  sr = sinf(roll_rad);

            float mx_h = mx * cp + mz * sp;
            float my_h = mx * sr * sp + my * cr - mz * sr * cp;

            float hdg_rad = atan2f(-my_h, mx_h);
            float hdg_deg = hdg_rad * (180.0f / (float)M_PI) + magDeclination;

            if (hdg_deg < 0.0f)   hdg_deg += 360.0f;
            if (hdg_deg >= 360.0f) hdg_deg -= 360.0f;

            currentHdg = hdg_deg;
            currentAz  = hdg_deg;
        }
    }
#else
    // Dead-reckoning: update drAz based on motor direction & elapsed time
    unsigned long now = millis();
    if (drAzDir != 0) {
        float elapsed = (now - drAzLastMs) / 1000.0f;
        drAz += drAzDir * AZ_DEG_PER_SEC * elapsed;
        if (drAz <   0.0f) drAz += 360.0f;
        if (drAz >= 360.0f) drAz -= 360.0f;
    }
    drAzLastMs  = now;
    currentAz   = drAz;
    currentHdg  = drAz;
#endif
}

// ============================================================
// Motor control helpers
// ============================================================
void azStop() {
    digitalWrite(AZ_IN1, LOW);
    digitalWrite(AZ_IN2, LOW);
    movingAz = false;
#if !USE_MAGNETOMETER
    drAzDir = 0;
#endif
}

void azCW() {
    digitalWrite(AZ_IN1, HIGH);
    digitalWrite(AZ_IN2, LOW);
    movingAz = true;
#if !USE_MAGNETOMETER
    drAzDir = +1;
    drAzLastMs = millis();
#endif
}

void azCCW() {
    digitalWrite(AZ_IN1, LOW);
    digitalWrite(AZ_IN2, HIGH);
    movingAz = true;
#if !USE_MAGNETOMETER
    drAzDir = -1;
    drAzLastMs = millis();
#endif
}

void elStop() {
    digitalWrite(EL_IN1, LOW);
    digitalWrite(EL_IN2, LOW);
    movingEl = false;
}

void elUp() {
    digitalWrite(EL_IN1, HIGH);
    digitalWrite(EL_IN2, LOW);
    movingEl = true;
}

void elDown() {
    digitalWrite(EL_IN1, LOW);
    digitalWrite(EL_IN2, HIGH);
    movingEl = true;
}

void stopAll() {
    azStop();
    elStop();
}

// Called regularly: check if we have reached the target and stop / continue.
void updateMotors() {
    if (movingAz) {
        float diff = targetAz - currentAz;
        // Shortest arc
        if (diff >  180.0f) diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;

        if (fabsf(diff) <= MOTOR_DEADBAND) {
            azStop();
        } else if (diff > 0.0f) {
            azCW();
        } else {
            azCCW();
        }
    }

    if (movingEl) {
        float diff = targetEl - currentEl;
        if (fabsf(diff) <= MOTOR_DEADBAND) {
            elStop();
        } else if (diff > 0.0f) {
            elUp();
        } else {
            elDown();
        }
    }
}

// ============================================================
// Status broadcast
// ============================================================
void sendStatus() {
    Serial.print(F("POS AZ="));
    Serial.print(currentAz, 1);
    Serial.print(F(" EL="));
    Serial.print(currentEl, 1);
    Serial.print(F(" HDG="));
    Serial.print(currentHdg, 1);
    Serial.print(F(" PITCH="));
    Serial.print(currentPitch, 1);
    Serial.print(F(" ROLL="));
    Serial.print(currentRoll, 1);
    Serial.print(F(" MOVING="));
    Serial.println((movingAz || movingEl) ? 1 : 0);
}

// ============================================================
// Command parser
// ============================================================

// Parse a float from a C-string; return false if invalid.
static bool parseFloat(const char *str, float &out) {
    char *end;
    float val = strtod(str, &end);
    if (end == str) return false;
    out = val;
    return true;
}

void processCommand(const char *cmd) {
    // Tokenise on spaces
    char buf[CMD_BUF_SIZE];
    strncpy(buf, cmd, CMD_BUF_SIZE - 1);
    buf[CMD_BUF_SIZE - 1] = '\0';

    char *tok = strtok(buf, " \t");
    if (!tok) return;

    // ── STATUS ──────────────────────────────────────────────
    if (strcasecmp(tok, "STATUS") == 0) {
        sendStatus();
        Serial.println(F("OK STATUS"));
        return;
    }

    // ── STOP ────────────────────────────────────────────────
    if (strcasecmp(tok, "STOP") == 0) {
        stopAll();
        Serial.println(F("OK STOP"));
        return;
    }

    // ── AZ <degrees> ────────────────────────────────────────
    if (strcasecmp(tok, "AZ") == 0) {
        tok = strtok(NULL, " \t");
        float val;
        if (!tok || !parseFloat(tok, val)) {
            Serial.println(F("ERR AZ requires a numeric argument"));
            return;
        }
        if (val < 0.0f || val > 360.0f) {
            Serial.println(F("ERR AZ must be 0-360"));
            return;
        }
        targetAz = val;
        movingAz = true;
        Serial.print(F("OK AZ "));
        Serial.println(val, 1);
        return;
    }

    // ── EL <degrees> ────────────────────────────────────────
    if (strcasecmp(tok, "EL") == 0) {
        tok = strtok(NULL, " \t");
        float val;
        if (!tok || !parseFloat(tok, val)) {
            Serial.println(F("ERR EL requires a numeric argument"));
            return;
        }
        if (val < -90.0f || val > 90.0f) {
            Serial.println(F("ERR EL must be -90 to 90"));
            return;
        }
        targetEl = val;
        movingEl = true;
        Serial.print(F("OK EL "));
        Serial.println(val, 1);
        return;
    }

    // ── MOVE AZ <f> EL <f> ──────────────────────────────────
    if (strcasecmp(tok, "MOVE") == 0) {
        // Expect: AZ <float> EL <float>
        float newAz = targetAz, newEl = targetEl;
        bool gotAz = false, gotEl = false;
        char *key;
        while ((key = strtok(NULL, " \t")) != NULL) {
            char *numStr = strtok(NULL, " \t");
            float val;
            if (!numStr || !parseFloat(numStr, val)) continue;
            if (strcasecmp(key, "AZ") == 0) { newAz = val; gotAz = true; }
            if (strcasecmp(key, "EL") == 0) { newEl = val; gotEl = true; }
        }
        if (!gotAz && !gotEl) {
            Serial.println(F("ERR MOVE expects AZ <f> EL <f>"));
            return;
        }
        if (gotAz) { targetAz = newAz; movingAz = true; }
        if (gotEl) { targetEl = newEl; movingEl = true; }
        Serial.print(F("OK MOVE AZ="));
        Serial.print(targetAz, 1);
        Serial.print(F(" EL="));
        Serial.println(targetEl, 1);
        return;
    }

    // ── DECL <degrees> ──────────────────────────────────────
    if (strcasecmp(tok, "DECL") == 0) {
        tok = strtok(NULL, " \t");
        float val;
        if (!tok || !parseFloat(tok, val)) {
            Serial.println(F("ERR DECL requires a numeric argument"));
            return;
        }
        magDeclination = val;
        Serial.print(F("OK DECL "));
        Serial.println(val, 2);
        return;
    }

    Serial.print(F("ERR Unknown command: "));
    Serial.println(cmd);
}

// Accumulate incoming bytes into cmdBuf; call processCommand on newline.
void readSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdLen > 0) {
                cmdBuf[cmdLen] = '\0';
                processCommand(cmdBuf);
                cmdLen = 0;
            }
        } else if (cmdLen < CMD_BUF_SIZE - 1) {
            cmdBuf[cmdLen++] = c;
        }
    }
}

// ============================================================
// Arduino setup / loop
// ============================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    Wire.begin();
    Wire.setClock(400000UL);    // Fast mode I2C

    // Motor pins
    pinMode(AZ_IN1, OUTPUT);
    pinMode(AZ_IN2, OUTPUT);
    pinMode(EL_IN1, OUTPUT);
    pinMode(EL_IN2, OUTPUT);
#ifdef AZ_ENA
    pinMode(AZ_ENA, OUTPUT);
    digitalWrite(AZ_ENA, HIGH);
#endif
#ifdef EL_ENB
    pinMode(EL_ENB, OUTPUT);
    digitalWrite(EL_ENB, HIGH);
#endif
    stopAll();

    // Initialise IMU chips
    delay(100);
    mpuOk = mpu6050_init();

#if USE_MAGNETOMETER
    magOk = qmc5883l_init();
#else
    magOk = false;
#endif

    // Report init status
    Serial.print(F("INIT MPU6050="));
    Serial.print(mpuOk ? F("OK") : F("FAIL"));
    Serial.print(F(" QMC5883L="));
    Serial.println(magOk ? F("OK") : (USE_MAGNETOMETER ? F("FAIL") : F("DISABLED")));

    lastStatusMs = millis();
    lastImuMs    = millis();
#if !USE_MAGNETOMETER
    drAzLastMs   = millis();
#endif
}

void loop() {
    unsigned long now = millis();

    // Read IMU at IMU_INTERVAL
    if (now - lastImuMs >= IMU_INTERVAL) {
        lastImuMs = now;
        updateIMU();
        updateMotors();
    }

    // Broadcast status at STATUS_INTERVAL
    if (now - lastStatusMs >= STATUS_INTERVAL) {
        lastStatusMs = now;
        sendStatus();
    }

    // Parse incoming serial commands
    readSerial();
}

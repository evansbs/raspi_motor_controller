/*
 * Antenna Rotator Firmware for Arduino Nano
 *
 * Hardware:
 *   - Arduino Nano (ATmega328P)
 *   - MPU6050 IMU on I2C (address 0x68 or 0x69) — accelerometer/gyro
 *   - HMC5883L/QMC5883L-compatible magnetometer on I2C (0x1E / 0x0D)
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
// I2C addresses (auto-detected in setup)
// ============================================================
#define MPU6050_ADDR_PRIMARY   0x69
#define MPU6050_ADDR_FALLBACK  0x68
#define MAG_ADDR_HMC5883L      0x1E
#define MAG_ADDR_QMC5883L      0x0D
#define LIS3DH_ADDR            0x19

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
bool lis3dhDetected = false;
bool lis3dhOk = false;
uint8_t activeMpuAddr = MPU6050_ADDR_PRIMARY;

enum MagType {
    MAG_NONE = 0,
    MAG_HMC5883L,
    MAG_QMC5883L
};

MagType magType = MAG_NONE;
uint8_t activeMagAddr = 0;

static bool i2c_device_present(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

static void printHexByte(uint8_t value) {
    if (value < 16) Serial.print('0');
    Serial.print(value, HEX);
}

static bool mpu_who_am_i_compatible(uint8_t whoAmI) {
    switch (whoAmI) {
        case 0x68: // MPU-6050 / MPU-6000 family
        case 0x69:
        case 0x70: // compatible InvenSense-family clones / variants
        case 0x71:
        case 0x72:
        case 0x73:
        case 0x75:
            return true;
        default:
            return false;
    }
}

// ============================================================
// MPU-6050 helpers
// ============================================================

static bool mpu6050_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(activeMpuAddr);
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
    // Probe WHO_AM_I when available; some compatible parts/clones identify
    // with nearby InvenSense-family values while keeping the same accel regs.
    Wire.beginTransmission(activeMpuAddr);
    Wire.write(0x75);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)activeMpuAddr, (uint8_t)1, (uint8_t)true) == 1) {
        uint8_t whoAmI = Wire.read();
        if (!mpu_who_am_i_compatible(whoAmI)) {
            // Some boards expose MPU-compatible accel registers but report a
            // different ID here; tolerate that and let live accel reads decide.
        }
    }
    return true;
}

// Returns true and fills ax/ay/az in units of g.
bool mpu6050_read_accel(float &ax, float &ay, float &azv) {
    Wire.beginTransmission(activeMpuAddr);
    Wire.write(0x3B);   // ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)activeMpuAddr, (uint8_t)6, (uint8_t)true) < 6) return false;

    int16_t raw_ax = ((int16_t)Wire.read() << 8) | Wire.read();
    int16_t raw_ay = ((int16_t)Wire.read() << 8) | Wire.read();
    int16_t raw_az = ((int16_t)Wire.read() << 8) | Wire.read();

    const float scale = 1.0f / 16384.0f;   // ±2 g
    ax  = raw_ax * scale;
    ay  = raw_ay * scale;
    azv = raw_az * scale;
    return true;
}

static bool lis3dh_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(LIS3DH_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool lis3dh_init() {
    Wire.beginTransmission(LIS3DH_ADDR);
    Wire.write(0x0F);   // WHO_AM_I
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)LIS3DH_ADDR, (uint8_t)1, (uint8_t)true) == 1) {
        uint8_t whoAmI = Wire.read();
        if (whoAmI != 0x33) return false;
    }

    // 100 Hz, normal mode, XYZ enabled
    if (!lis3dh_write(0x20, 0x57)) return false;
    // BDU enabled, high-resolution, ±2 g
    if (!lis3dh_write(0x23, 0x88)) return false;
    return true;
}

bool lis3dh_read_accel(float &ax, float &ay, float &azv) {
    Wire.beginTransmission(LIS3DH_ADDR);
    Wire.write(0x28 | 0x80);   // OUT_X_L with auto-increment
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)LIS3DH_ADDR, (uint8_t)6, (uint8_t)true) < 6) return false;

    int16_t raw_ax = (int16_t)(Wire.read() | ((uint16_t)Wire.read() << 8));
    int16_t raw_ay = (int16_t)(Wire.read() | ((uint16_t)Wire.read() << 8));
    int16_t raw_az = (int16_t)(Wire.read() | ((uint16_t)Wire.read() << 8));

    raw_ax >>= 4;
    raw_ay >>= 4;
    raw_az >>= 4;

    const float scale = 1.0f / 1000.0f;   // ±2 g high-resolution ≈ 1 mg/LSB
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
    Wire.beginTransmission(activeMagAddr);
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
    Wire.beginTransmission(activeMagAddr);
    Wire.write(0x06);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)activeMagAddr, (uint8_t)1, (uint8_t)true);
    if (!Wire.available()) return false;
    uint8_t status = Wire.read();
    if (!(status & 0x01)) return false;   // DRDY not set

    Wire.beginTransmission(activeMagAddr);
    Wire.write(0x00);   // Data starts at register 0
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)activeMagAddr, (uint8_t)6, (uint8_t)true) < 6) return false;

    // QMC5883L is little-endian
    mx = (int16_t)(Wire.read() | ((uint16_t)Wire.read() << 8));
    my = (int16_t)(Wire.read() | ((uint16_t)Wire.read() << 8));
    mz = (int16_t)(Wire.read() | ((uint16_t)Wire.read() << 8));
    return true;
}

#endif  // USE_MAGNETOMETER

#if USE_MAGNETOMETER
static bool hmc_write8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(activeMagAddr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool hmc5883l_init() {
    // CRA: 8-average, 15Hz, normal measurement
    if (!hmc_write8(0x00, 0x70)) return false;
    // CRB: gain setting
    if (!hmc_write8(0x01, 0x20)) return false;
    // Mode: continuous measurement
    if (!hmc_write8(0x02, 0x00)) return false;
    return true;
}

bool hmc5883l_read(int16_t &mx, int16_t &my, int16_t &mz) {
    Wire.beginTransmission(activeMagAddr);
    Wire.write(0x03);  // Data output X MSB
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)activeMagAddr, (uint8_t)6, (uint8_t)true) < 6) return false;

    // HMC5883L register order: X, Z, Y (big-endian)
    int16_t x = ((int16_t)Wire.read() << 8) | Wire.read();
    int16_t z = ((int16_t)Wire.read() << 8) | Wire.read();
    int16_t y = ((int16_t)Wire.read() << 8) | Wire.read();
    mx = x;
    my = y;
    mz = z;
    return true;
}
#endif

// ============================================================
// IMU fusion: compute heading, pitch, roll
// ============================================================
static void updateTiltFromAccel(float ax, float ay, float azv, float &pitch_rad, float &roll_rad) {
    pitch_rad = atan2f(-ax, sqrtf(ay * ay + azv * azv));
    roll_rad  = atan2f(ay,  azv);

    currentPitch = pitch_rad * (180.0f / (float)M_PI);
    currentRoll  = roll_rad  * (180.0f / (float)M_PI);
    currentEl    = currentPitch;    // Elevation ≈ pitch angle
}

void updateIMU() {
    float pitch_rad = currentPitch * ((float)M_PI / 180.0f);
    float roll_rad  = currentRoll  * ((float)M_PI / 180.0f);
    float ax, ay, azv;

    if (mpuOk && mpu6050_read_accel(ax, ay, azv)) {
        updateTiltFromAccel(ax, ay, azv, pitch_rad, roll_rad);
    } else if (lis3dhOk && lis3dh_read_accel(ax, ay, azv)) {
        updateTiltFromAccel(ax, ay, azv, pitch_rad, roll_rad);
    }

#if USE_MAGNETOMETER
    if (magOk) {
        int16_t mx16, my16, mz16;
        bool magReadOk = false;
        if (magType == MAG_HMC5883L) magReadOk = hmc5883l_read(mx16, my16, mz16);
        if (magType == MAG_QMC5883L) magReadOk = qmc5883l_read(mx16, my16, mz16);
        if (magReadOk) {
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
    if (i2c_device_present(MPU6050_ADDR_PRIMARY)) {
        activeMpuAddr = MPU6050_ADDR_PRIMARY;
        mpuOk = mpu6050_init();
    }
    if (!mpuOk && i2c_device_present(MPU6050_ADDR_FALLBACK)) {
        activeMpuAddr = MPU6050_ADDR_FALLBACK;
        mpuOk = mpu6050_init();
    }
    lis3dhDetected = i2c_device_present(LIS3DH_ADDR);
    if (lis3dhDetected) {
        lis3dhOk = lis3dh_init();
    }

#if USE_MAGNETOMETER
    if (i2c_device_present(MAG_ADDR_HMC5883L)) {
        activeMagAddr = MAG_ADDR_HMC5883L;
        magType = MAG_HMC5883L;
        magOk = hmc5883l_init();
    }
    if (!magOk && i2c_device_present(MAG_ADDR_QMC5883L)) {
        activeMagAddr = MAG_ADDR_QMC5883L;
        magType = MAG_QMC5883L;
        magOk = qmc5883l_init();
    }
#else
    magOk = false;
#endif

    // Report init status
    Serial.print(F("INIT MPU6050="));
    Serial.print(mpuOk ? F("OK") : F("FAIL"));
    Serial.print(F(" MAG="));
    if (!USE_MAGNETOMETER) {
        Serial.print(F("DISABLED"));
    } else if (!magOk) {
        Serial.print(F("FAIL"));
    } else if (magType == MAG_HMC5883L) {
        Serial.print(F("HMC5883L"));
    } else if (magType == MAG_QMC5883L) {
        Serial.print(F("QMC5883L"));
    } else {
        Serial.print(F("UNKNOWN"));
    }
    Serial.print(F(" MPU_ADDR=0x"));
    printHexByte(activeMpuAddr);
    Serial.print(F(" MAG_ADDR=0x"));
    if (activeMagAddr) printHexByte(activeMagAddr);
    else Serial.print(F("00"));
    Serial.print(F(" LIS3DH@0x19="));
    Serial.println(lis3dhDetected ? F("DETECTED") : F("NO"));

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

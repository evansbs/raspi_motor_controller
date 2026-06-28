# Raspberry Pi / Arduino Nano Antenna Rotator Controller

End-to-end AZ/EL antenna rotator system:  
**Raspberry Pi 3** desktop GUI ↔ **Arduino Nano** firmware  
using an L298N motor driver and MPU6050 + HMC/QMC-compatible magnetometer.

---

## Repository layout

```
firmware/
  antenna_rotator/
    antenna_rotator.ino   Arduino Nano sketch
gui/
  antenna_controller.py   Raspberry Pi tkinter GUI
  requirements.txt        Python dependencies
README.md
```

---

## Hardware

| Component | Notes |
|-----------|-------|
| Raspberry Pi 3 (or any Pi) | Runs the Python GUI |
| Arduino Nano (ATmega328P) | Motor control + IMU bridge |
| MPU6050 | Accelerometer/gyro over I2C (address **0x69** primary, 0x68 fallback) — pitch & roll |
| HMC5883L-compatible magnetometer | Magnetometer over I2C (address **0x1E** primary, 0x0D fallback) — compass heading |
| LIS3DH (optional/present on some boards) | Detected at I2C 0x19 and used as an accel fallback for pitch/roll/elevation |
| L298N motor driver | Dual H-bridge for AZ and EL motors |
| AZ motor | ~0.6 rpm, connected to L298N Motor-A outputs |
| EL motor | ~0.6 rpm, connected to L298N Motor-B outputs |

### Arduino wiring

| Arduino pin | L298N pin | L298N output | Function |
|-------------|-----------|--------------|----------|
| D9  | IN1 | OUT3 | AZ+ (CW)  |
| D10 | IN2 | OUT4 | AZ− (CCW) |
| D5  | IN3 | OUT1 | EL+ (up)  |
| D6  | IN4 | OUT2 | EL− (down)|
| 5 V | VCC | —    | Logic power |
| GND | GND | —    | Common ground |

> If your L298N module has separate ENA/ENB header pins (not jumpered),
> uncomment `#define AZ_ENA 3` and `#define EL_ENB 9` in the sketch and
> connect those pins to PWM-capable Arduino pins.

IMU modules use the Arduino's I2C bus (A4 = SDA, A5 = SCL, 3.3 V or 5 V power).

---

## Serial protocol

115200 baud, ASCII, newline-terminated (`\n`).

### Commands (Pi → Arduino)

| Command | Description |
|---------|-------------|
| `STATUS` | Request immediate status |
| `AZ <degrees>` | Move to AZ target (0–360) |
| `EL <degrees>` | Move to EL target (−90–90) |
| `MOVE AZ <f> EL <f>` | Move to both targets |
| `STOP` | Stop all motors immediately |
| `DECL <degrees>` | Set magnetic declination (+ east) |

### Responses (Arduino → Pi)

| Response | Description |
|----------|-------------|
| `POS AZ=<f> EL=<f> HDG=<f> PITCH=<f> ROLL=<f> MOVING=<0\|1>` | Status broadcast (every 500 ms + on demand) |
| `OK <echo>` | Command acknowledged |
| `ERR <message>` | Command rejected |
| `INIT MPU6050=<OK\|FAIL> MAG=<...> MPU_ADDR=0x.. MAG_ADDR=0x.. LIS3DH@0x19=<...>` | Startup report |

---

## Firmware setup

### Arduino libraries required

| Library | Used for |
|---------|----------|
| `Wire` (built-in) | I2C communication |

No external Arduino libraries are needed — the sketch communicates with
both ICs directly over I2C.

### Build & upload

1. Open **Arduino IDE** (or PlatformIO).  
2. Open `firmware/antenna_rotator/antenna_rotator.ino`.  
3. Select **Board → Arduino Nano**, processor **ATmega328P (Old Bootloader)**
   (or **ATmega328P** depending on your module).  
4. Select the correct COM/tty port.  
5. Click **Upload**.

### Compile-time options (top of sketch)

```cpp
#define USE_MAGNETOMETER 1   // Set to 0 if no magnetometer is fitted
#define MOTOR_DEADBAND   2.0 // Degrees — stop motors within this error
#define AZ_DEG_PER_SEC   3.6 // Dead-reckoning speed (0.6 rpm × 6)
#define MAG_DECLINATION  0.0 // Or use the DECL command at runtime
```

---

## Raspberry Pi GUI setup

### Prerequisites

```bash
sudo apt install python3-tk       # if not already installed
pip install pyserial              # or: pip install -r gui/requirements.txt
```

### Run

```bash
python3 gui/antenna_controller.py
```

### GUI overview

| Section | Function |
|---------|----------|
| **Connection bar** | Select serial port & baud, Connect/Disconnect |
| **Current Position / IMU** | Live AZ, EL, Heading, Pitch, Roll readouts |
| **Motor Control** | AZ/EL target entry, Move, STOP, Status refresh |
| **CSV Logging** | Choose file, Start/Stop logging |
| **Serial Console** | Raw serial traffic for debugging |

### CSV log format

```
timestamp,az_deg,el_deg,heading_deg,pitch_deg,roll_deg
2025-01-15T12:34:56.789,45.30,15.20,44.80,15.20,2.10
```

A new header row is written only when the file does not already exist,
so logging can be resumed across sessions.

---

## Magnetic declination

Local magnetic declination compensates for the difference between magnetic
north and true north.  Look up your value at
[ngdc.noaa.gov/geomag](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml).

Set it in the GUI **Mag Declination** field and click **Set** to send a
`DECL <degrees>` command to the Arduino.  Positive values are east of true north.

---

## Extending to hamlib

The serial protocol is intentionally close to the **GS-232B** standard used
by hamlib's `rotctld`.  To add hamlib compatibility, wrap
`antenna_controller.py` with a thin translation layer that accepts GS-232B
commands over a local TCP socket and forwards them as `AZ`/`EL`/`STOP`
commands to the serial port.  The `POS` broadcast lines can be mapped back
to GS-232B `AZ\nEL\n` replies.

---

## Troubleshooting

| Symptom | Likely cause |
|---------|-------------|
| `INIT MPU6050=FAIL` | Wiring error on SDA/SCL, wrong I2C address, or an incompatible IMU; if `LIS3DH@0x19=DETECTED`, pitch/roll/elevation may still work via fallback |
| `INIT ... MAG=FAIL` | Check 0x1E/0x0D wiring and I2C pull-ups |
| AZ reads 0.0 always | `USE_MAGNETOMETER=0` or DRDY never set |
| Motor runs continuously | Deadband too small, or IMU reading wrong axis |
| GUI shows "---°" | No POS line received yet — check baud rate |

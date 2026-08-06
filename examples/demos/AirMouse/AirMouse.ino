/*
 * OpenMote Gen 3.3 - Air Mouse  (Arduino IDE version)
 *
 * Turns the OpenMote into an absolute-pointing USB air mouse using 6-DOF sensor
 * fusion (accel+gyro). Pitch and roll are absolute via gravity, while yaw
 * (heading) is integrated from the gyro.
 *
 * Controls
 *   A      left click / hold = drag
 *   B      right click
 *   HOME   recenter - point at the middle of the screen and tap to re-zero aim
 *   +/-    sensitivity (smaller / larger field of view)
 *   1      recalibrate gyro (hold still; rejects if moved)
 *   2      scroll wheel down
 *   POWER  pause / resume pointing
 *
 * Serial console (USB CDC, 115200) live-tuning commands:
 *   c recenter   r reset filter   g gyro-bias cal
 *   + / -  field of view          d toggle debug stream
 *   x / y  invert X / Y axis      s swap yaw/pitch axes
 *
 * ---------------------------------------------------------------------------
 * Arduino IDE settings
 *
 * The standard OpenMote set from docs/ARDUINO_IDE_SETUP.md, with ONE change:
 *
 *   USB Mode:  USB-OTG (TinyUSB)      <-- required, this is a HID device
 *
 * The default "Hardware CDC and JTAG" mode does not compile the TinyUSB stack,
 * so USB.h / USBHID.h will not be available and this sketch will not build.
 * Depending on your USB CDC On Boot setting you may lose the Serial Monitor
 * while USB-OTG is selected; the sketch never requires it.
 *
 * Calibration is stored in NVS, so any Partition Scheme with a normal NVS
 * partition works. "Erase All Flash Before Sketch Upload" wipes the stored
 * gyro bias.
 * ---------------------------------------------------------------------------
 *
 * Ported from demos/airmouse in openmote-beta-kit. The original talks to the
 * LSM6DS3TR-C directly; this version reads it through the OpenMote library,
 * which configures the sensor identically (416 Hz, +/-2 g, 1000 dps) and does
 * the same 12-byte burst read from OUTX_L_G. The sensor fusion, pointing maths
 * and USB HID descriptor are unchanged.
 */
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

#include <OpenMote.h>

#include "USB.h"
#include "MahonyAHRS.h"
#include "AbsoluteMouse.h"

// ============================================================
// Tunable defaults
// ============================================================
static float  FOV_X_DEG   = 60.0f;
static float  FOV_Y_DEG   = 40.0f;
static float  MAHONY_KP   = 1.0f;
static float  MAHONY_KI   = 0.10f;
static float  POINT_SMOOTH = 0.70f;
static float  ACCEL_GATE_G = 0.35f;

static bool   INVERT_X    = false;
static bool   INVERT_Y    = true;
static bool   SWAP_AXES   = false;

// The board's aim axis in sensor-frame coordinates: straight out the nose.
#define AIM_AXIS_X   0.0f
#define AIM_AXIS_Y   1.0f
#define AIM_AXIS_Z   0.0f

// ============================================================
// Globals
// ============================================================
OpenMote      mote;
AbsoluteMouse Mouse;
MahonyAHRS    ahrs(MAHONY_KP, MAHONY_KI);
Preferences   prefs;

struct Calib {
    float gbx = 0, gby = 0, gbz = 0;   // gyro bias, degrees/second
} cal;

static float  azRef = 0, elRef = 0;
static float  smX = 0.5f, smY = 0.5f;
static bool   armed = false;
static bool   debugStream = false;
static uint32_t lastUs = 0;

static const float D2R = (float)M_PI / 180.0f;
static const float R2D = 180.0f / (float)M_PI;

// ============================================================
// Sensor
//
// OpenMote::readIMU() returns accel in g and gyro in degrees/second, already
// scaled. The bias subtraction and the conversion to rad/s that the filter
// wants stay here, because they are this sketch's business and not the
// library's.
// ============================================================
static bool readMotion(float &ax, float &ay, float &az,
                       float &gx, float &gy, float &gz) {
    IMUData d = mote.readIMU();
    if (!d.valid) return false;

    ax = d.ax; ay = d.ay; az = d.az;
    gx = (d.gx - cal.gbx) * D2R;
    gy = (d.gy - cal.gby) * D2R;
    gz = (d.gz - cal.gbz) * D2R;
    return true;
}

// ============================================================
// Calibration
// ============================================================
static void saveCalib() {
    prefs.begin("airmouse", false);
    prefs.putBytes("calib", &cal, sizeof(cal));
    prefs.end();
}
static void loadCalib() {
    prefs.begin("airmouse", true);
    if (prefs.getBytesLength("calib") == sizeof(cal))
        prefs.getBytes("calib", &cal, sizeof(cal));
    prefs.end();
}

// Reject a calibration if the per-axis gyro jitter (RMS / standard deviation)
// exceeds this - that means the controller was being moved, so the average
// would bake real rotation into the "bias" and cause permanent drift. RMS is
// used instead of peak-to-peak because a single noisy I2C sample blows up p2p
// and would reject an otherwise perfectly still board.
static const float GYRO_STILL_RMS_DPS = 1.0f;

// Returns true if a new bias was accepted, false if rejected (motion / no
// data). On reject the previous bias is left untouched.
static bool calibrateGyro(uint16_t samples = 400) {
    Serial.println("[gyro] hold the controller STILL...");
    double sx = 0, sy = 0, sz = 0;          // sums (for the mean / bias)
    double sx2 = 0, sy2 = 0, sz2 = 0;       // sums of squares (for the variance)
    uint16_t got = 0;

    // readIMU() reports raw dps, so unlike the original there is no need to
    // zero the stored bias first - we are averaging uncorrected samples.
    for (uint16_t i = 0; i < samples; i++) {
        IMUData d = mote.readIMU();
        if (d.valid) {
            sx  += d.gx;              sy  += d.gy;              sz  += d.gz;
            sx2 += (double)d.gx*d.gx; sy2 += (double)d.gy*d.gy; sz2 += (double)d.gz*d.gz;
            got++;
        }
        delay(3);
    }

    if (got <= 50) {
        Serial.println("[gyro] FAILED (no samples)");
        return false;
    }

    float mx = sx / got, my = sy / got, mz = sz / got;
    // var = E[x^2] - mean^2, clamped to 0 against float round-off.
    float vx = fmaxf(0.0f, (float)(sx2 / got) - mx * mx);
    float vy = fmaxf(0.0f, (float)(sy2 / got) - my * my);
    float vz = fmaxf(0.0f, (float)(sz2 / got) - mz * mz);
    float rms = sqrtf(fmaxf(vx, fmaxf(vy, vz)));

    if (rms > GYRO_STILL_RMS_DPS) {
        Serial.printf("[gyro] REJECTED - jitter rms=%.2f dps (limit %.2f); kept old bias\n",
                      rms, GYRO_STILL_RMS_DPS);
        return false;
    }

    cal.gbx = mx; cal.gby = my; cal.gbz = mz;
    Serial.printf("[gyro] bias dps  x=%.3f y=%.3f z=%.3f  (rms=%.2f)\n",
                  cal.gbx, cal.gby, cal.gbz, rms);
    saveCalib();
    return true;
}

// ============================================================
// LED feedback (4 player LEDs)
// ============================================================

// Restore the steady at-rest LED state (LED 1 = armed indicator, rest off).
static void ledsRestore() {
    mote.setLEDs(armed ? 0b0001 : 0b0000);
}

// Rejected calibration: flash all four LEDs together.
static void ledFlashAll(uint8_t times) {
    for (uint8_t t = 0; t < times; t++) {
        mote.setLEDs(0b1111); delay(120);
        mote.setLEDs(0b0000); delay(120);
    }
}

// Accepted calibration: one LED 1 -> LED 4 chase.
static void ledChaseOnce() {
    for (uint8_t i = 1; i <= 4; i++) {
        mote.setLED(i, true);
        delay(90);
        mote.setLED(i, false);
    }
}

// ============================================================
// Pointing
// ============================================================
static float wrapPi(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static void aimAngles(float &az, float &el) {
    float px, py, pz;
    ahrs.bodyToWorld(AIM_AXIS_X, AIM_AXIS_Y, AIM_AXIS_Z, px, py, pz);
    float n = sqrtf(px * px + py * py + pz * pz);
    if (n > 1e-6f) { px /= n; py /= n; pz /= n; }
    az = atan2f(px, py);
    el = asinf(constrain(pz, -1.0f, 1.0f));
}

static void recenter() {
    aimAngles(azRef, elRef);
    smX = smY = 0.5f;
    Serial.println("[aim] recentered");
}

// Recalibrate the gyro bias with visible pass/fail feedback. Pointing is paused
// while sampling so the cursor can't run off; on success the aim is re-zeroed.
static void recalibrateWithFeedback() {
    bool wasArmed = armed;
    armed = false;                       // freeze the pointer during sampling
    bool ok = calibrateGyro();
    if (ok) {
        ledChaseOnce();                  // accepted: chase animation
        if (wasArmed) recenter();        // drop accumulated drift, re-zero
    } else {
        ledFlashAll(4);                  // rejected: flash all LEDs, bias unchanged
    }
    armed = wasArmed;
    ledsRestore();
    lastUs = micros();                   // discard the long dt from the blocking cal
}

static void updatePointer() {
    float az, el;
    aimAngles(az, el);
    float dAz = wrapPi(az - azRef);
    float dEl = el - elRef;

    float fovX = radians(FOV_X_DEG);
    float fovY = radians(FOV_Y_DEG);

    float aX = SWAP_AXES ? dEl : dAz;
    float aY = SWAP_AXES ? dAz : dEl;
    float spanX = SWAP_AXES ? fovY : fovX;
    float spanY = SWAP_AXES ? fovX : fovY;

    float nx = 0.5f + (INVERT_X ? -1.0f : 1.0f) * (aX / spanX);
    float ny = 0.5f + (INVERT_Y ? -1.0f : 1.0f) * (aY / spanY);

    nx = constrain(nx, 0.0f, 1.0f);
    ny = constrain(ny, 0.0f, 1.0f);

    smX += (nx - smX) * (1.0f - POINT_SMOOTH);
    smY += (ny - smY) * (1.0f - POINT_SMOOTH);

    Mouse.moveTo((uint16_t)(smX * ABS_MOUSE_LOGICAL_MAX),
                 (uint16_t)(smY * ABS_MOUSE_LOGICAL_MAX));
}

// ============================================================
// Buttons
//
// The library debounces and gives press/release edges, so the hand-rolled edge
// and tap detectors from the original are gone. Tap detection still needs the
// press timestamp, because buttonHoldTime() reports 0 once a button is up.
// ============================================================
static const uint32_t HOLD_MS = 800;
static uint32_t pressedAt[BTN_COUNT] = {0};

// True on release, if the button was down for less than HOLD_MS.
static bool tapped(OpenMoteButton b) {
    if (mote.buttonPressed(b))  pressedAt[b] = millis();
    if (mote.buttonReleased(b)) return (millis() - pressedAt[b]) < HOLD_MS;
    return false;
}

static void feedback(uint8_t pulses) {
    for (uint8_t i = 0; i < pulses; i++) {
        mote.rumbleOn(255); mote.setLED(2, true);  delay(45);
        mote.rumbleOff();   mote.setLED(2, false); delay(90);
    }
    // LED 2 doubles as the drag indicator, so put it back if A is still held.
    mote.setLED(2, armed && mote.buttonDown(BTN_A));
}

static void adjustFov(float dx, float dy) {
    FOV_X_DEG = constrain(FOV_X_DEG + dx, 15.0f, 150.0f);
    FOV_Y_DEG = constrain(FOV_Y_DEG + dy, 10.0f, 120.0f);
    Serial.printf("[fov] x=%.0f y=%.0f deg\n", FOV_X_DEG, FOV_Y_DEG);
}

static void setPaused(bool paused) {
    if (paused == !armed) return;
    armed = !paused;
    if (paused) { Mouse.release(ABS_MOUSE_LEFT); Mouse.release(ABS_MOUSE_RIGHT); }
    else        { recenter(); }
    mote.setLED(1, armed);
    Serial.printf("[point] %s\n", armed ? "ACTIVE" : "PAUSED");
    feedback(armed ? 2 : 1);
}

static void handleButtons() {
    // A: left button, press-and-hold to drag.
    if (mote.buttonPressed(BTN_A) && armed) {
        Mouse.press(ABS_MOUSE_LEFT);
        mote.setLED(2, true);
    }
    if (mote.buttonReleased(BTN_A)) {
        Mouse.release(ABS_MOUSE_LEFT);
        mote.setLED(2, false);
    }

    // B: right button.
    if (mote.buttonPressed(BTN_B) && armed) Mouse.press(ABS_MOUSE_RIGHT);
    if (mote.buttonReleased(BTN_B))         Mouse.release(ABS_MOUSE_RIGHT);

    if (mote.buttonPressed(BTN_1))          recalibrateWithFeedback();
    if (mote.buttonPressed(BTN_2) && armed) Mouse.scroll(-1);

    if (tapped(BTN_HOME))  { recenter(); feedback(1); }
    if (tapped(BTN_POWER)) { setPaused(armed); }
    if (tapped(BTN_PLUS))  { adjustFov(-5.0f, -3.0f); feedback(1); }
    if (tapped(BTN_MINUS)) { adjustFov(+5.0f, +3.0f); feedback(1); }
}

// ============================================================
// Serial console
// ============================================================
static void handleSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        switch (c) {
            case 'c': recenter(); break;
            case 'r': ahrs.reset(); Serial.println("[ahrs] reset"); break;
            case 'g': recalibrateWithFeedback(); break;
            case '+': adjustFov(-5.0f, -3.0f); break;
            case '-': adjustFov(+5.0f, +3.0f); break;
            case 'x': INVERT_X = !INVERT_X; Serial.printf("[axis] invertX=%d\n", INVERT_X); break;
            case 'y': INVERT_Y = !INVERT_Y; Serial.printf("[axis] invertY=%d\n", INVERT_Y); break;
            case 's': SWAP_AXES = !SWAP_AXES; Serial.printf("[axis] swap=%d\n", SWAP_AXES); break;
            case '{': POINT_SMOOTH = constrain(POINT_SMOOTH - 0.05f, 0.0f, 0.98f);
                      Serial.printf("[smooth] %.2f\n", POINT_SMOOTH); break;
            case '}': POINT_SMOOTH = constrain(POINT_SMOOTH + 0.05f, 0.0f, 0.98f);
                      Serial.printf("[smooth] %.2f\n", POINT_SMOOTH); break;
            case '[': ACCEL_GATE_G = constrain(ACCEL_GATE_G - 0.02f, 0.02f, 0.5f);
                      Serial.printf("[gate] %.2f g\n", ACCEL_GATE_G); break;
            case ']': ACCEL_GATE_G = constrain(ACCEL_GATE_G + 0.02f, 0.02f, 0.5f);
                      Serial.printf("[gate] %.2f g\n", ACCEL_GATE_G); break;
            case 'd': debugStream = !debugStream; break;
            default: break;
        }
    }
}

// ============================================================
// Setup / loop
// ============================================================
void setup() {
    Serial.begin(115200);

    // begin() claims every pin on the board, brings up I2C and configures the
    // IMU at 416 Hz / +/-2 g / 1000 dps - the same settings the standalone
    // demo writes by hand. It also parks the rumble gate, which matters: the
    // motor is on an LEDC channel here, so the pin must not be driven with
    // digitalWrite().
    bool ok = mote.begin();

    Mouse.begin();
    USB.begin();

    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 1500) delay(10);
    Serial.println("\nOpenMote Gen 3.3 Air Mouse (No-Mag)");
    Serial.println(ok && mote.imuAvailable() ? "IMU OK" : "IMU FAIL");

    loadCalib();
    ahrs.setGains(MAHONY_KP, MAHONY_KI);
    calibrateGyro();

    Serial.println("[init] converging fusion...");
    float ax, ay, az, gx, gy, gz;
    uint32_t cs = millis(); lastUs = micros();
    while (millis() - cs < 1500) {
        if (readMotion(ax, ay, az, gx, gy, gz)) {
            uint32_t now = micros();
            float dt = (now - lastUs) * 1e-6f; lastUs = now;
            if (dt > 0 && dt < 0.2f) ahrs.update(gx, gy, gz, ax, ay, az, 0, 0, 0, dt);
        }
        delay(3);
    }

    recenter();
    armed = true;
    mote.setLED(1, true);
    feedback(2);
    lastUs = micros();
}

void loop() {
    mote.update();                       // debounce + rumble timers

    float ax, ay, az, gx, gy, gz;

    if (readMotion(ax, ay, az, gx, gy, gz)) {
        uint32_t now = micros();
        float dt = (now - lastUs) * 1e-6f;
        lastUs = now;
        if (dt > 0 && dt < 0.2f) {
            float amag = sqrtf(ax * ax + ay * ay + az * az);
            // Reject accel while the board is being shaken: only trust gravity
            // when the magnitude is close to 1 g.
            if (fabsf(amag - 1.0f) > ACCEL_GATE_G) { ax = ay = az = 0; }
            ahrs.update(gx, gy, gz, ax, ay, az, 0, 0, 0, dt);
            if (armed) updatePointer();
        }
    }

    handleButtons();
    handleSerial();

    if (debugStream) {
        static uint32_t tdbg = 0;
        if (millis() - tdbg > 100) {
            tdbg = millis();
            float az, el; aimAngles(az, el);
            Serial.printf("aim az=%.1f el=%.1f  cur=(%.2f,%.2f)\n",
                          degrees(az), degrees(el), smX, smY);
        }
    }
    delayMicroseconds(500);
}

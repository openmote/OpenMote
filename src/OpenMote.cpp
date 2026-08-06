#include "OpenMote.h"
#include <math.h>

// ---------------------------------------------------------------------------
// Arduino-ESP32 core 2.x / 3.x compatibility.
// The LEDC (PWM) API was rewritten in core 3.0: channels went away and you now
// address the pin directly. These two macros hide the difference.
// ---------------------------------------------------------------------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define OM_LEDC_ATTACH(pin, ch, freq, bits)  ledcAttach((pin), (freq), (bits))
  #define OM_LEDC_WRITE(pin, ch, duty)         ledcWrite((pin), (duty))
#else
  #define OM_LEDC_ATTACH(pin, ch, freq, bits) \
      (ledcSetup((ch), (freq), (bits)), ledcAttachPin((pin), (ch)), true)
  #define OM_LEDC_WRITE(pin, ch, duty)         ledcWrite((ch), (duty))
#endif

#define OM_RUMBLE_LEDC_CHANNEL  0
#define OM_RUMBLE_PWM_FREQ      20000   // above hearing, so no coil whine
#define OM_RUMBLE_PWM_BITS      8

// ---------------------------------------------------------------------------
// Static tables. Order must match the OpenMoteButton enum.
// ---------------------------------------------------------------------------
static const uint8_t kButtonPins[BTN_COUNT] = {
    OM_PIN_BTN_UP,   OM_PIN_BTN_DOWN,  OM_PIN_BTN_LEFT, OM_PIN_BTN_RIGHT,
    OM_PIN_BTN_A,    OM_PIN_BTN_B,     OM_PIN_BTN_1,    OM_PIN_BTN_2,
    OM_PIN_BTN_PLUS, OM_PIN_BTN_MINUS, OM_PIN_BTN_HOME, OM_PIN_BTN_POWER
};

static const char *const kButtonNames[BTN_COUNT] = {
    "Up", "Down", "Left", "Right",
    "A",  "B",    "1",    "2",
    "+",  "-",    "Home", "Power"
};

static const uint8_t kLEDPins[4] = {
    OM_PIN_LED1, OM_PIN_LED2, OM_PIN_LED3, OM_PIN_LED4
};

// ---- LSM6DS3TR-C registers ----
#define REG_IMU_WHOAMI      0x0F
#define REG_IMU_CTRL1_XL    0x10
#define REG_IMU_CTRL2_G     0x11
#define REG_IMU_CTRL3_C     0x12
#define REG_IMU_OUT_TEMP_L  0x20
#define REG_IMU_OUTX_L_G    0x22   // gyro X..Z then accel X..Z, 12 bytes
#define IMU_WHOAMI_VALUE    0x6A

// ===========================================================================

OpenMote::OpenMote() {}

bool OpenMote::begin(bool initSensors) {
    // Rumble first, and low: on Gen 3.3 the gate pulldown (R20, 10k) is weak
    // enough that the motor can twitch before firmware takes the pin.
    pinMode(OM_PIN_RUMBLE, OUTPUT);
    digitalWrite(OM_PIN_RUMBLE, LOW);
    OM_LEDC_ATTACH(OM_PIN_RUMBLE, OM_RUMBLE_LEDC_CHANNEL,
                   OM_RUMBLE_PWM_FREQ, OM_RUMBLE_PWM_BITS);
    OM_LEDC_WRITE(OM_PIN_RUMBLE, OM_RUMBLE_LEDC_CHANNEL, 0);
    _rumbleActive = false;

    // LEDs off
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(kLEDPins[i], OUTPUT);
        digitalWrite(kLEDPins[i], LOW);
    }
    _ledState = 0;

    // Buttons. Note this also claims GPIO43/44 away from UART0 — which is why
    // the board must log over USB CDC, not the hardware serial port.
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        pinMode(kButtonPins[i], INPUT_PULLUP);
    }
    _stable = _prevStable = _lastRaw = 0xFFFF;
    uint32_t now = millis();
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        _lastChange[i] = now;
        _pressedAt[i]  = 0;
    }

    // Battery ADC
    analogReadResolution(12);
    analogSetPinAttenuation(OM_PIN_BATT_ADC, ADC_11db);
    batteryVolts();   // prime the cache while Wi-Fi is guaranteed to be off

    if (!initSensors) return true;

    Wire.begin(OM_PIN_SDA, OM_PIN_SCL);
    Wire.setClock(400000);
    delay(10);

    _imuOk = initIMU();
    return _imuOk;
}

void OpenMote::update() {
    uint32_t now = millis();

    // --- buttons: sample, then only accept a level that has been stable ---
    uint16_t raw = 0;
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        if (digitalRead(kButtonPins[i])) raw |= (uint16_t)1 << i;
    }

    _prevStable = _stable;
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        uint16_t bit    = (uint16_t)1 << i;
        bool rawBit     = (raw & bit) != 0;
        bool lastRawBit = (_lastRaw & bit) != 0;

        if (rawBit != lastRawBit) {
            _lastChange[i] = now;                 // bouncing, restart the timer
        } else if ((now - _lastChange[i]) >= _debounceMs) {
            bool stableBit = (_stable & bit) != 0;
            if (rawBit != stableBit) {
                if (rawBit) _stable |= bit;       // released (pin pulled high)
                else      { _stable &= ~bit; _pressedAt[i] = now; }
            }
        }
    }
    _lastRaw = raw;

    // --- rumble auto-stop ---
    if (_rumbleActive && _rumbleUntil != 0 && (int32_t)(now - _rumbleUntil) >= 0) {
        rumbleOff();
    }
}

// ---------------------------------------------------------------------------
// Buttons.  A bit is 0 when the button is DOWN (active low).
// ---------------------------------------------------------------------------

bool OpenMote::buttonPressed(OpenMoteButton b) const {
    if (b >= BTN_COUNT) return false;
    uint16_t bit = (uint16_t)1 << b;
    return !(_stable & bit) && (_prevStable & bit);
}

bool OpenMote::buttonReleased(OpenMoteButton b) const {
    if (b >= BTN_COUNT) return false;
    uint16_t bit = (uint16_t)1 << b;
    return (_stable & bit) && !(_prevStable & bit);
}

bool OpenMote::buttonDown(OpenMoteButton b) const {
    if (b >= BTN_COUNT) return false;
    return !(_stable & ((uint16_t)1 << b));
}

uint32_t OpenMote::buttonHoldTime(OpenMoteButton b) const {
    if (!buttonDown(b)) return 0;
    return millis() - _pressedAt[b];
}

bool OpenMote::buttonHeldFor(OpenMoteButton b, uint32_t ms) const {
    return buttonDown(b) && (millis() - _pressedAt[b]) >= ms;
}

bool OpenMote::anyButtonDown() const {
    return (_stable & 0x0FFF) != 0x0FFF;
}

uint16_t OpenMote::buttonMask() const {
    return (uint16_t)(~_stable) & 0x0FFF;
}

const char *OpenMote::buttonName(OpenMoteButton b) {
    return (b < BTN_COUNT) ? kButtonNames[b] : "?";
}

uint8_t OpenMote::buttonPin(OpenMoteButton b) {
    return (b < BTN_COUNT) ? kButtonPins[b] : 0xFF;
}

// ---------------------------------------------------------------------------
// LEDs (numbered 1-4 on the silkscreen)
// ---------------------------------------------------------------------------

void OpenMote::setLED(uint8_t led, bool on) {
    if (led < 1 || led > 4) return;
    digitalWrite(kLEDPins[led - 1], on ? HIGH : LOW);
    if (on) _ledState |=  (uint8_t)(1 << (led - 1));
    else    _ledState &= (uint8_t)~(1 << (led - 1));
}

void OpenMote::toggleLED(uint8_t led) {
    if (led < 1 || led > 4) return;
    setLED(led, !getLED(led));
}

bool OpenMote::getLED(uint8_t led) const {
    if (led < 1 || led > 4) return false;
    return (_ledState & (1 << (led - 1))) != 0;
}

void OpenMote::setLEDs(uint8_t mask) {
    for (uint8_t i = 0; i < 4; i++) setLED(i + 1, (mask & (1 << i)) != 0);
}

void OpenMote::ledBar(uint8_t count) {
    if (count > 4) count = 4;
    setLEDs((uint8_t)((1 << count) - 1));
}

// ---------------------------------------------------------------------------
// Rumble
// ---------------------------------------------------------------------------

void OpenMote::rumbleOn(uint8_t strength) {
    OM_LEDC_WRITE(OM_PIN_RUMBLE, OM_RUMBLE_LEDC_CHANNEL, strength);
    _rumbleActive = (strength > 0);
    _rumbleUntil  = 0;
}

void OpenMote::rumble(uint16_t ms, uint8_t strength) {
    if (ms == 0 || strength == 0) { rumbleOff(); return; }
    OM_LEDC_WRITE(OM_PIN_RUMBLE, OM_RUMBLE_LEDC_CHANNEL, strength);
    _rumbleActive = true;
    _rumbleUntil  = millis() + ms;
    if (_rumbleUntil == 0) _rumbleUntil = 1;   // 0 means "no timeout"
}

void OpenMote::rumbleOff() {
    OM_LEDC_WRITE(OM_PIN_RUMBLE, OM_RUMBLE_LEDC_CHANNEL, 0);
    _rumbleActive = false;
    _rumbleUntil  = 0;
}

// ---------------------------------------------------------------------------
// Battery
//
// GPIO14 is ADC2_CH3. The Wi-Fi driver owns ADC2 while the radio is up, and the
// read then returns 0. We treat 0 as "unavailable" and hand back the last good
// value instead of a bogus 0 V.
// ---------------------------------------------------------------------------

float OpenMote::batteryVolts() {
    const uint8_t samples = 8;
    uint32_t total = 0;
    uint8_t  good  = 0;

    for (uint8_t i = 0; i < samples; i++) {
        uint32_t mv = analogReadMilliVolts(OM_PIN_BATT_ADC);
        if (mv > 0) { total += mv; good++; }
    }

    if (good == 0) return _battVolts;   // Wi-Fi has the ADC; use the cache

    float vAdc = (total / (float)good) / 1000.0f;
    _battVolts = vAdc * _battMultiplier;
    _battValid = true;
    return _battVolts;
}

uint8_t OpenMote::batteryPercent() {
    float v = batteryVolts();

    // 1S LiPo discharge curve, flattest in the middle.
    static const float kVolts[]   = {3.30f, 3.40f, 3.50f, 3.60f, 3.70f,
                                     3.80f, 3.90f, 4.00f, 4.10f, 4.20f};
    static const float kPercent[] = {0.0f,  5.0f,  10.0f, 20.0f, 35.0f,
                                     50.0f, 65.0f, 80.0f, 90.0f, 100.0f};
    const uint8_t n = sizeof(kVolts) / sizeof(kVolts[0]);

    if (v <= kVolts[0])     return 0;
    if (v >= kVolts[n - 1]) return 100;

    for (uint8_t i = 1; i < n; i++) {
        if (v < kVolts[i]) {
            float span = kVolts[i] - kVolts[i - 1];
            float frac = (v - kVolts[i - 1]) / span;
            float pct  = kPercent[i - 1] + frac * (kPercent[i] - kPercent[i - 1]);
            return (uint8_t)(pct + 0.5f);
        }
    }
    return 100;
}

// ---------------------------------------------------------------------------
// I2C helpers
// ---------------------------------------------------------------------------

bool OpenMote::i2cWrite(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool OpenMote::i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

bool OpenMote::i2cDetect(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

uint8_t OpenMote::i2cScan(Print &out) {
    uint8_t found = 0;
    out.println(F("Scanning I2C bus (SDA=16, SCL=15)..."));
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (i2cDetect(addr)) {
            out.printf("  0x%02X", addr);
            if (addr == OM_I2C_ADDR_IMU) out.print(F("  <- LSM6DS3TR-C IMU"));
            out.println();
            found++;
        }
    }
    if (found == 0) out.println(F("  nothing responded - check pull-ups and power"));
    return found;
}

// ---------------------------------------------------------------------------
// LSM6DS3TR-C inertial measurement unit
// ---------------------------------------------------------------------------

bool OpenMote::initIMU() {
    uint8_t who = 0;
    if (!i2cRead(OM_I2C_ADDR_IMU, REG_IMU_WHOAMI, &who, 1)) return false;
    if (who != IMU_WHOAMI_VALUE) return false;

    // BDU: don't let a sample update between the low and high byte of a read.
    // IF_INC: auto-increment the register pointer so burst reads work.
    if (!i2cWrite(OM_I2C_ADDR_IMU, REG_IMU_CTRL3_C, 0x44)) return false;

    setAccelRange(ACCEL_2G);      // 416 Hz
    setGyroRange(GYRO_1000DPS);   // 416 Hz
    delay(20);
    return true;
}

bool OpenMote::setAccelRange(OpenMoteAccelRange range) {
    // CTRL1_XL: ODR[7:4] = 0110 (416 Hz), FS_XL[3:2]
    uint8_t fs;
    float   mgPerLsb;
    switch (range) {
        case ACCEL_4G:  fs = 0b10; mgPerLsb = 0.122f; break;
        case ACCEL_8G:  fs = 0b11; mgPerLsb = 0.244f; break;
        case ACCEL_16G: fs = 0b01; mgPerLsb = 0.488f; break;
        case ACCEL_2G:
        default:        fs = 0b00; mgPerLsb = 0.061f; break;
    }
    if (!i2cWrite(OM_I2C_ADDR_IMU, REG_IMU_CTRL1_XL, (uint8_t)(0x60 | (fs << 2)))) {
        return false;
    }
    _accelScale = mgPerLsb / 1000.0f;
    return true;
}

bool OpenMote::setGyroRange(OpenMoteGyroRange range) {
    // CTRL2_G: ODR[7:4] = 0110 (416 Hz), FS_G[3:2]
    uint8_t fs;
    float   mdpsPerLsb;
    switch (range) {
        case GYRO_245DPS:  fs = 0b00; mdpsPerLsb =  8.75f; break;
        case GYRO_500DPS:  fs = 0b01; mdpsPerLsb = 17.50f; break;
        case GYRO_2000DPS: fs = 0b11; mdpsPerLsb = 70.00f; break;
        case GYRO_1000DPS:
        default:           fs = 0b10; mdpsPerLsb = 35.00f; break;
    }
    if (!i2cWrite(OM_I2C_ADDR_IMU, REG_IMU_CTRL2_G, (uint8_t)(0x60 | (fs << 2)))) {
        return false;
    }
    _gyroScale = mdpsPerLsb / 1000.0f;
    return true;
}

IMUData OpenMote::readIMU() {
    IMUData d;
    if (!_imuOk) return d;

    uint8_t b[12];
    if (!i2cRead(OM_I2C_ADDR_IMU, REG_IMU_OUTX_L_G, b, 12)) return d;

    int16_t rgx = (int16_t)(b[0]  | (b[1]  << 8));
    int16_t rgy = (int16_t)(b[2]  | (b[3]  << 8));
    int16_t rgz = (int16_t)(b[4]  | (b[5]  << 8));
    int16_t rax = (int16_t)(b[6]  | (b[7]  << 8));
    int16_t ray = (int16_t)(b[8]  | (b[9]  << 8));
    int16_t raz = (int16_t)(b[10] | (b[11] << 8));

    d.ax = rax * _accelScale;
    d.ay = ray * _accelScale;
    d.az = raz * _accelScale;
    d.gx = rgx * _gyroScale;
    d.gy = rgy * _gyroScale;
    d.gz = rgz * _gyroScale;
    d.valid = true;
    return d;
}

float OpenMote::readTemperature() {
    if (!_imuOk) return NAN;
    uint8_t b[2];
    if (!i2cRead(OM_I2C_ADDR_IMU, REG_IMU_OUT_TEMP_L, b, 2)) return NAN;
    int16_t raw = (int16_t)(b[0] | (b[1] << 8));
    return 25.0f + raw / 256.0f;   // 256 LSB/degC, zero point at 25 C
}

float OpenMote::pitch() {
    IMUData d = readIMU();
    if (!d.valid) return NAN;
    return atan2f(-d.ax, sqrtf(d.ay * d.ay + d.az * d.az)) * 180.0f / (float)M_PI;
}

float OpenMote::roll() {
    IMUData d = readIMU();
    if (!d.valid) return NAN;
    return atan2f(d.ay, d.az) * 180.0f / (float)M_PI;
}

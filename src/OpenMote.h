/*
 * OpenMote.h — common driver for the OpenMote Gen 3.3 board
 *
 * Covers: 12 buttons (debounced, with edge + hold detection), 4 player LEDs,
 * the rumble motor, battery voltage, and the LSM6DS3TR-C inertial sensor.
 *
 * Audio (I2S speaker / PDM mic) is deliberately NOT in this library — see
 * docs/TROUBLESHOOTING.md for why, and the demo firmware projects for examples.
 *
 * Minimal sketch:
 *
 *     #include <OpenMote.h>
 *     OpenMote mote;
 *
 *     void setup() { mote.begin(); }
 *
 *     void loop() {
 *         mote.update();                             // ALWAYS call this
 *         if (mote.buttonPressed(BTN_A)) {
 *             mote.setLED(1, true);
 *             mote.rumble(200);
 *         }
 *     }
 *
 * MIT licensed. See LICENSE.
 */
#ifndef OPENMOTE_H
#define OPENMOTE_H

#include <Arduino.h>
#include <Wire.h>
#include "OpenMotePins.h"

// ---------------------------------------------------------------------------
// Sensor result types
// ---------------------------------------------------------------------------

/** Accelerometer + gyroscope sample from the LSM6DS3TR-C. */
struct IMUData {
    float ax = 0, ay = 0, az = 0;   ///< acceleration, g (1.0 = gravity)
    float gx = 0, gy = 0, gz = 0;   ///< angular rate, degrees/second
    bool  valid = false;            ///< false if the I2C read failed
};

/** Accelerometer full-scale range. Larger range = less resolution. */
enum OpenMoteAccelRange : uint8_t { ACCEL_2G = 0, ACCEL_4G, ACCEL_8G, ACCEL_16G };

/** Gyroscope full-scale range, in degrees/second. */
enum OpenMoteGyroRange : uint8_t { GYRO_245DPS = 0, GYRO_500DPS, GYRO_1000DPS, GYRO_2000DPS };

// ---------------------------------------------------------------------------

class OpenMote {
public:
    OpenMote();

    // -- Setup ------------------------------------------------------------

    /**
     * Configure every pin on the board and bring up the I2C sensor.
     *
     * @param initSensors  false to skip IMU setup (saves ~50 ms and leaves the
     *                     I2C bus untouched).
     * @return true if everything requested came up. If it returns false, check
     *         imuAvailable() — the buttons, LEDs, rumble and battery still work
     *         regardless.
     */
    bool begin(bool initSensors = true);

    /**
     * Service button debouncing and the rumble timer.
     * Call this once at the top of every loop(). Without it, buttonPressed()
     * never fires and a timed rumble never stops.
     */
    void update();

    // -- Buttons ----------------------------------------------------------
    // All twelve buttons are active-low with internal pull-ups, debounced by
    // 15 ms. Pass one of the BTN_* values from OpenMotePins.h.

    /** True on the single update() where the button went from up to down. */
    bool buttonPressed(OpenMoteButton b) const;

    /** True on the single update() where the button went from down to up. */
    bool buttonReleased(OpenMoteButton b) const;

    /** True for as long as the button is physically held down. */
    bool buttonDown(OpenMoteButton b) const;

    /** True while the button has been held continuously for at least ms. */
    bool buttonHeldFor(OpenMoteButton b, uint32_t ms) const;

    /** Milliseconds the button has been held, or 0 if it is not down. */
    uint32_t buttonHoldTime(OpenMoteButton b) const;

    /** True if any button at all is down. */
    bool anyButtonDown() const;

    /** Bitmask of all buttons currently down; bit N = BTN_* value N. */
    uint16_t buttonMask() const;

    /** Human-readable name ("A", "Up", "Home"...) — handy for debug printing. */
    static const char *buttonName(OpenMoteButton b);

    /** The GPIO number behind a button, if you want to drive it yourself. */
    static uint8_t buttonPin(OpenMoteButton b);

    /** Debounce window in milliseconds. Default 15. */
    void setDebounceTime(uint16_t ms) { _debounceMs = ms; }

    // -- Player LEDs ------------------------------------------------------
    // Numbered 1-4 to match the silkscreen. Out-of-range values are ignored.

    /** Turn one LED on or off. */
    void setLED(uint8_t led, bool on);

    /** Flip one LED. */
    void toggleLED(uint8_t led);

    /** Read back what an LED was last set to. */
    bool getLED(uint8_t led) const;

    /** Set all four at once from a bitmask: bit0 = LED1 ... bit3 = LED4. */
    void setLEDs(uint8_t mask);

    /** Light the first `count` LEDs as a bar graph (0-4). */
    void ledBar(uint8_t count);

    /** Turn all four off. */
    void allLEDsOff() { setLEDs(0); }

    // -- Rumble -----------------------------------------------------------
    // The motor is PWM-driven through an SI2302 MOSFET on GPIO39.

    /**
     * Buzz for a fixed time, then stop automatically. Returns immediately —
     * update() does the stopping, so keep calling it.
     * @param ms        duration in milliseconds
     * @param strength  0-255 PWM duty. Below roughly 60 the motor may not spin.
     */
    void rumble(uint16_t ms, uint8_t strength = 255);

    /** Start the motor and leave it running until rumbleOff(). */
    void rumbleOn(uint8_t strength = 255);

    /** Stop the motor now, cancelling any timed rumble. */
    void rumbleOff();

    /** True while the motor is being driven. */
    bool isRumbling() const { return _rumbleActive; }

    // -- Battery ----------------------------------------------------------
    // NOTE: the sense pin is on ADC2, which the Wi-Fi radio takes priority over.
    // While Wi-Fi is running the hardware read fails; these functions then
    // return the last value read successfully. Take a reading before
    // WiFi.begin() so there is always something sane cached.

    /** Battery voltage in volts (a healthy 1S LiPo is 3.3 V - 4.2 V). */
    float batteryVolts();

    /** Rough state of charge, 0-100 %, from a 1S LiPo discharge curve. */
    uint8_t batteryPercent();

    /** False if no successful ADC read has happened yet, or Wi-Fi is blocking it. */
    bool batteryValid() const { return _battValid; }

    /**
     * Trim the divider ratio. Default is 5.2553 from the schematic
     * (R19 = 200k, R26 = 47k). Measure the pack with a multimeter and scale:
     *     new = 5.2553 * (measured_volts / reported_volts)
     */
    void setBatteryCalibration(float multiplier) { _battMultiplier = multiplier; }

    // -- Motion sensor ----------------------------------------------------

    /** True if the LSM6DS3TR-C answered on the bus during begin(). */
    bool imuAvailable() const { return _imuOk; }

    /** One accelerometer + gyroscope sample. Check .valid before using it. */
    IMUData readIMU();

    /** Die temperature from the IMU, in degrees C. Roughly +/- 2 C. */
    float readTemperature();

    /** Tilt angle around the board's X axis, -180..180 degrees. */
    float pitch();

    /** Tilt angle around the board's Y axis, -90..90 degrees. */
    float roll();

    /** Change the accelerometer full-scale range. Default ACCEL_2G. */
    bool setAccelRange(OpenMoteAccelRange range);

    /** Change the gyroscope full-scale range. Default GYRO_1000DPS. */
    bool setGyroRange(OpenMoteGyroRange range);

    // -- Low-level helpers ------------------------------------------------

    /** Write one register on any device on the shared I2C bus. */
    bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t value);

    /** Read `len` bytes starting at a register. */
    bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len);

    /** True if a device ACKs at this address. Useful for the J15 expansion port. */
    bool i2cDetect(uint8_t addr);

    /** Print every responding I2C address to a stream. Returns the count found. */
    uint8_t i2cScan(Print &out = Serial);

private:
    bool initIMU();

    // Buttons
    uint16_t _stable      = 0xFFFF;  // 1 bit per button, 1 = released (pin high)
    uint16_t _prevStable  = 0xFFFF;
    uint16_t _lastRaw     = 0xFFFF;
    uint32_t _lastChange[BTN_COUNT] = {0};
    uint32_t _pressedAt[BTN_COUNT]  = {0};
    uint16_t _debounceMs  = 15;

    // LEDs
    uint8_t _ledState = 0;

    // Rumble
    bool     _rumbleActive = false;
    uint32_t _rumbleUntil  = 0;      // 0 = run until rumbleOff()

    // Battery
    float _battMultiplier = OM_BATT_DIVIDER;
    float _battVolts      = 0.0f;
    bool  _battValid      = false;

    // Sensor
    bool  _imuOk = false;
    float _accelScale = 0.061f / 1000.0f;   // g per LSB  (+/-2 g default)
    float _gyroScale  = 35.0f / 1000.0f;    // dps per LSB (1000 dps default)
};

#endif // OPENMOTE_H

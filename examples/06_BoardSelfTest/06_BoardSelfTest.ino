/*
 * 06 — Board self-test
 *
 * Run this first on a new beta board. It checks everything the library covers
 * and prints a pass/fail summary you can paste into a bug report.
 *
 * The button test is interactive: press each of the twelve buttons once when
 * prompted, or wait 30 seconds to skip.
 */
#include <OpenMote.h>
#include <OpenMoteIR.h>

OpenMote   mote;
OpenMoteIR ir;

uint8_t passed = 0, failed = 0;

void report(const char *what, bool ok, const char *detail = "") {
    Serial.printf("  [%s] %-22s %s\n", ok ? "PASS" : "FAIL", what, detail);
    ok ? passed++ : failed++;
}

void testSensors() {
    Serial.println("\nSensors");
    report("LSM6DS3TR-C IMU", mote.imuAvailable(), "I2C 0x6A");

    if (mote.imuAvailable()) {
        IMUData d = mote.readIMU();
        float mag = sqrtf(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
        char buf[48];
        snprintf(buf, sizeof(buf), "%.2f g total (expect ~1.0)", mag);
        // At rest the accelerometer should read one gravity, whichever way up.
        report("accelerometer scale", d.valid && mag > 0.85f && mag < 1.15f, buf);

        float t = mote.readTemperature();
        snprintf(buf, sizeof(buf), "%.1f C", t);
        report("IMU temperature", !isnan(t) && t > 0 && t < 60, buf);

        snprintf(buf, sizeof(buf), "%+.1f deg pitch, %+.1f deg roll",
                 mote.pitch(), mote.roll());
        report("tilt angles", !isnan(mote.pitch()), buf);
    }
}

void testBattery() {
    Serial.println("\nPower");
    float v = mote.batteryVolts();
    char buf[48];
    snprintf(buf, sizeof(buf), "%.2f V, ~%u%%", v, mote.batteryPercent());
    // Anything outside this is either no battery, or a bad divider.
    report("battery sense", v > 2.5f && v < 4.6f, buf);
}

void testLEDs() {
    Serial.println("\nLEDs — watch the four player LEDs light in turn");
    for (uint8_t i = 1; i <= 4; i++) {
        mote.setLED(i, true);  delay(250);
        mote.setLED(i, false);
    }
    mote.setLEDs(0b1111); delay(400); mote.allLEDsOff();
    report("LED sweep", true, "visual check");
}

void testRumble() {
    Serial.println("\nHaptics — you should feel two buzzes");
    mote.rumbleOn(255); delay(300); mote.rumbleOff();
    delay(200);
    mote.rumbleOn(120); delay(300); mote.rumbleOff();
    report("rumble motor", true, "tactile check");
}

void testIR() {
    Serial.println("\nInfrared — cup your hand over the front of the board");
    delay(1500);
    ir.resume();
    ir.sendNEC(0x00, 0xAA);        // bounce a frame off your palm
    uint32_t start = millis();
    bool heard = false;
    while (millis() - start < 500) {
        if (ir.available()) { heard = true; break; }
        delay(5);
    }
    report("IR transmit + receive", heard,
           heard ? "loopback echo detected" : "no echo (try again, hand closer)");
    ir.resume();
}

void testButtons() {
    Serial.println("\nButtons — press each one; 30 s timeout");
    uint16_t seen = 0;
    const uint16_t all = (uint16_t)((1 << BTN_COUNT) - 1);
    uint32_t start = millis();

    while (seen != all && millis() - start < 30000) {
        mote.update();
        for (uint8_t i = 0; i < BTN_COUNT; i++) {
            if (mote.buttonPressed((OpenMoteButton)i) && !(seen & (1 << i))) {
                seen |= (uint16_t)(1 << i);
                Serial.printf("    got %-5s (%u of %u)\n",
                              OpenMote::buttonName((OpenMoteButton)i),
                              __builtin_popcount(seen), BTN_COUNT);
                mote.rumble(40);
            }
        }
        delay(5);
    }

    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        if (!(seen & (1 << i))) {
            char buf[32];
            snprintf(buf, sizeof(buf), "GPIO %u never went low",
                     OpenMote::buttonPin((OpenMoteButton)i));
            report(OpenMote::buttonName((OpenMoteButton)i), false, buf);
        }
    }
    if (seen == all) report("all 12 buttons", true, "");
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println("\n================================");
    Serial.println(" OpenMote Gen 3.3 self-test");
    Serial.println("================================");

    mote.begin();
    ir.begin();

    Serial.println("\nI2C bus");
    mote.i2cScan();

    testSensors();
    testBattery();
    testLEDs();
    testRumble();
    testIR();
    testButtons();

    Serial.println("\n================================");
    Serial.printf(" %u passed, %u failed\n", passed, failed);
    Serial.println("================================");

    // Green-ish result: all four LEDs. Any failure: LED 1 only.
    mote.setLEDs(failed == 0 ? 0b1111 : 0b0001);
}

void loop() {
    delay(1000);
}

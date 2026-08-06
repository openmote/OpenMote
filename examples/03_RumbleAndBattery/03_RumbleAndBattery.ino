/*
 * 03 — Rumble and battery
 *
 *   A        short buzz
 *   B        long buzz at half strength
 *   1        hold to run the motor continuously
 *   Home     print the battery reading
 *
 * The LED bar shows the charge level, refreshed every 5 seconds.
 *
 * Two things worth knowing about the battery on Gen 3.3:
 *
 *  1. The sense pin (GPIO14) sits on ADC2, which the Wi-Fi radio takes over.
 *     While Wi-Fi is running the read fails and batteryVolts() returns the last
 *     good value. Read it once before WiFi.begin() so the cache is warm.
 *
 *  2. The divider ratio comes from the schematic, not from your board. Measure
 *     the pack with a multimeter once and call setBatteryCalibration().
 */
#include <OpenMote.h>

OpenMote mote;

void setup() {
    Serial.begin(115200);
    delay(1000);
    mote.begin();

    // Example: if the meter says 3.95 V but this reports 4.05 V, scale it:
    //   mote.setBatteryCalibration(5.2553f * (3.95f / 4.05f));

    Serial.println("A = short buzz, B = long soft buzz, hold 1 = continuous.");
}

void loop() {
    mote.update();                  // also what stops a timed rumble

    if (mote.buttonPressed(BTN_A)) mote.rumble(80);
    if (mote.buttonPressed(BTN_B)) mote.rumble(600, 128);   // half strength

    // Hold-to-run: start on the press edge, stop on the release edge.
    if (mote.buttonPressed(BTN_1))  mote.rumbleOn(200);
    if (mote.buttonReleased(BTN_1)) mote.rumbleOff();

    if (mote.buttonPressed(BTN_HOME)) {
        Serial.printf("battery: %.3f V, ~%u%%%s\n",
                      mote.batteryVolts(), mote.batteryPercent(),
                      mote.batteryValid() ? "" : "  (stale - ADC unavailable)");
    }

    static uint32_t lastBar = 0;
    if (millis() - lastBar > 5000) {
        lastBar = millis();
        uint8_t pct = mote.batteryPercent();
        mote.ledBar((pct + 24) / 25);      // 0-4 LEDs, one per quarter
    }

    delay(10);
}

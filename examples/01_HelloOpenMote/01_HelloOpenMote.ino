/*
 * 01 — Hello OpenMote
 *
 * Start here. Prints what hardware answered, then reports every button press
 * and lights an LED while a button is down.
 *
 * Open Tools > Serial Monitor at 115200 baud. If nothing appears, check that
 * "USB CDC On Boot" is Enabled — see docs/ARDUINO_IDE_SETUP.md.
 */
#include <OpenMote.h>

OpenMote mote;

void setup() {
    Serial.begin(115200);
    delay(1000);                 // let the USB serial port enumerate

    Serial.println();
    Serial.println("=== OpenMote Gen 3.3 ===");

    mote.begin();

    Serial.printf("IMU (LSM6DS3TR-C) : %s\n", mote.imuAvailable() ? "found" : "NOT FOUND");
    Serial.printf("Battery           : %.2f V (%u%%)\n",
                  mote.batteryVolts(), mote.batteryPercent());
    Serial.println();
    mote.i2cScan();
    Serial.println("\nPress any button.");

    // Sweep the LEDs once so you can see all four work.
    for (uint8_t i = 1; i <= 4; i++) { mote.setLED(i, true); delay(120); }
    for (uint8_t i = 1; i <= 4; i++) { mote.setLED(i, false); delay(120); }
}

void loop() {
    mote.update();               // required — debouncing happens in here

    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        OpenMoteButton b = (OpenMoteButton)i;

        if (mote.buttonPressed(b)) {
            Serial.printf("pressed  %-5s (GPIO %u)\n",
                          OpenMote::buttonName(b), OpenMote::buttonPin(b));
            mote.rumble(60);
        }
        if (mote.buttonReleased(b)) {
            Serial.printf("released %s\n", OpenMote::buttonName(b));
        }
    }

    // LED 1 mirrors "is anything held down".
    mote.setLED(1, mote.anyButtonDown());

    delay(5);
}

/*
 * 02 — Buttons and LEDs
 *
 * The four ways to ask about a button, and the LED helpers.
 *
 *   +/-        move a bar graph up and down
 *   1 / 2      toggle LED 3 / LED 4 individually
 *   D-pad      shows the difference between "pressed" (edge) and "down" (level)
 *   Hold Home  for 1.5 s to flash all four LEDs
 */
#include <OpenMote.h>

OpenMote mote;

uint8_t barLevel = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    mote.begin();

    // Buttons are debounced by 15 ms out of the box. Raise it for a worn switch,
    // lower it if you need snappier response for a game.
    mote.setDebounceTime(15);

    Serial.println("+/- change the bar, 1/2 toggle LEDs, hold Home to flash.");
}

void loop() {
    mote.update();

    // --- edge detection: fires once per press, no matter how long you hold ---
    if (mote.buttonPressed(BTN_PLUS)  && barLevel < 4) mote.ledBar(++barLevel);
    if (mote.buttonPressed(BTN_MINUS) && barLevel > 0) mote.ledBar(--barLevel);

    if (mote.buttonPressed(BTN_1)) mote.toggleLED(3);
    if (mote.buttonPressed(BTN_2)) mote.toggleLED(4);

    // --- level detection: true the whole time the button is held ---
    if (mote.buttonDown(BTN_UP))    Serial.println("Up is down");
    if (mote.buttonDown(BTN_DOWN))  Serial.println("Down is down");

    // --- hold detection: an action that needs deliberate intent ---
    static bool flashed = false;
    if (mote.buttonHeldFor(BTN_HOME, 1500)) {
        if (!flashed) {
            for (uint8_t i = 0; i < 6; i++) {
                mote.setLEDs(i % 2 ? 0b0000 : 0b1111);
                delay(80);
            }
            mote.ledBar(barLevel);
            flashed = true;
        }
    }
    if (mote.buttonReleased(BTN_HOME)) flashed = false;

    // --- everything at once, as a bitmask ---
    static uint16_t lastMask = 0;
    uint16_t mask = mote.buttonMask();
    if (mask != lastMask) {
        Serial.print("buttons down:");
        if (mask == 0) Serial.print(" (none)");
        for (uint8_t i = 0; i < BTN_COUNT; i++) {
            if (mask & (1 << i)) {
                Serial.print(' ');
                Serial.print(OpenMote::buttonName((OpenMoteButton)i));
            }
        }
        Serial.println();
        lastMask = mask;
    }

    delay(10);
}

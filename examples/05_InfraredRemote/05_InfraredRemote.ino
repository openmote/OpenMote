/*
 * 05 — Infrared: learn a button, then play it back
 *
 * Point any household remote at the front of the OpenMote and press a button.
 * The code is captured and stored in the slot shown by the LEDs.
 *
 *   D-pad Left/Right   choose slot 1-4
 *   A                  learn: the next remote button pressed fills the slot
 *   B                  transmit the code in the current slot
 *
 * The receiver is an IRM-56384 on GPIO1; the LED is on GPIO41. Both are on the
 * front edge of the board. Range is a few metres, line of sight.
 *
 * This library decodes NEC, which covers most cheap remotes. For air
 * conditioners and other long protocols, use IRremoteESP8266 with the same
 * two pin numbers.
 */
#include <OpenMote.h>
#include <OpenMoteIR.h>

OpenMote   mote;
OpenMoteIR ir;

struct Slot {
    bool     used = false;
    uint32_t code = 0;
    uint8_t  address = 0;
    uint8_t  command = 0;
};

Slot slots[4];
uint8_t current  = 0;
bool    learning = false;

void showSlot() {
    mote.setLEDs((uint8_t)(1 << current));
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    mote.begin();
    ir.begin();                 // installs the receive interrupt on GPIO1

    Serial.println("IR learn/replay ready.");
    Serial.println("Left/Right pick a slot, A learns, B transmits.");
    showSlot();
}

void loop() {
    mote.update();

    // --- pick a slot ---
    if (mote.buttonPressed(BTN_LEFT)  && current > 0) { current--; showSlot(); }
    if (mote.buttonPressed(BTN_RIGHT) && current < 3) { current++; showSlot(); }

    // --- arm learning ---
    if (mote.buttonPressed(BTN_A)) {
        learning = true;
        Serial.printf("Slot %u: point a remote at the OpenMote and press a button.\n",
                      current + 1);
        ir.resume();            // throw away anything already in the buffer
    }

    // --- transmit ---
    if (mote.buttonPressed(BTN_B)) {
        if (slots[current].used) {
            Serial.printf("Slot %u: sending 0x%08lX\n",
                          current + 1, (unsigned long)slots[current].code);
            ir.sendNECRaw(slots[current].code);
            mote.rumble(50);
        } else {
            Serial.printf("Slot %u is empty.\n", current + 1);
        }
    }

    // --- anything arrived? ---
    if (ir.available()) {
        if (ir.isRepeat()) {
            Serial.println("  (repeat frame - button still held)");
        } else if (ir.isNEC()) {
            Serial.printf("NEC  address 0x%02X  command 0x%02X  raw 0x%08lX\n",
                          ir.address(), ir.command(), (unsigned long)ir.code());
            if (learning) {
                slots[current] = { true, ir.code(), ir.address(), ir.command() };
                learning = false;
                Serial.printf("Stored in slot %u.\n", current + 1);
                mote.rumble(120);
            }
        } else {
            Serial.printf("Unrecognised frame, %u edges. Raw timing:\n",
                          ir.rawLength());
            ir.printRaw();      // paste this into ir.sendRaw() to replay it
        }
        ir.resume();            // always re-arm, or nothing else is captured
    }

    delay(5);
}

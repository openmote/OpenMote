/*
 * OpenMote Gen 3.3 — IR Learn & Replay  (Arduino IDE version)
 *
 * Turns the OpenMote into a learning universal remote. Point any household
 * remote at it, capture a code, bind that code to a button, and from then on
 * pressing the button transmits it.
 *
 * How to use it
 *   1. Hold A + B together for 2 seconds.
 *        -> one rumble, and the LEDs start a chase pattern: it is listening.
 *   2. Point a remote at the front of the OpenMote and press a button.
 *        -> all four LEDs slow-pulse: the code is captured and waiting.
 *   3. Press the OpenMote button you want to bind it to.
 *        -> two rapid flashes and two rumbles: saved.
 *   4. Press that button any time to transmit the code.
 *
 * Any of the twelve buttons can be bound, including A and B themselves.
 * Binding a button that already has a code simply overwrites it.
 *
 * Getting out: hold A + B again for 2 seconds while it is listening to cancel.
 * Either learn step also gives up on its own after 30 seconds.
 *
 * Mappings are stored in NVS, so they survive a power cycle and a re-flash of
 * the same firmware.
 *
 * Idle power: after 10 minutes with no button activity the board enters deep
 * sleep. Press Home, 1, 2, +, - or D-pad Right to wake it. A, B, Up, Down and
 * Left cannot wake it — they are not on RTC-capable GPIOs. Learned codes are in
 * NVS, so nothing is lost. Note the USB serial port disappears while asleep.
 *
 * Everything is echoed over USB CDC serial (115200) as a backup record — but
 * the firmware never depends on a serial monitor being attached, and runs
 * identically with nothing plugged in. See the note on logf() below for why it
 * does not gate on `if (Serial)`.
 *
 * NEC codes are stored as 32 bits. Anything the NEC decoder does not recognise
 * is stored as raw mark/space timing instead and replayed on a 38 kHz carrier,
 * so most remotes work even though only NEC is properly decoded.
 *
 * Hardware: OpenMote Gen 3.3. IR receiver on GPIO1, IR LED on GPIO41 — both on
 * the front edge of the board.
 *
 * ---------------------------------------------------------------------------
 * Arduino IDE settings
 *
 * Board settings are the standard OpenMote set from docs/ARDUINO_IDE_SETUP.md:
 * ESP32S3 Dev Module, PSRAM = OPI PSRAM, Flash Size = 16MB, Flash Mode =
 * QIO 80MHz, USB CDC On Boot = Enabled, USB Mode = Hardware CDC and JTAG.
 *
 * Partition Scheme: any scheme with a normal NVS partition works — the stock
 * "16M Flash (3MB APP/9.9MB FATFS)" gives NVS the same 20 KB as this demo's
 * PlatformIO partitions.csv. A full set of twelve learned codes needs ~3.2 KB.
 *
 * Note that "Erase All Flash Before Sketch Upload" wipes NVS, and with it every
 * learned code. Leave it Disabled unless that is what you want.
 * ---------------------------------------------------------------------------
 *
 * This is a direct port of demos/ir_learn/src/main.cpp. See that folder's
 * README.md for the design notes.
 */
#include <Arduino.h>
#include <Preferences.h>
#include <stdarg.h>

#include <esp_sleep.h>
#include <driver/rtc_io.h>

#include <OpenMote.h>
#include <OpenMoteIR.h>

OpenMote    mote;
OpenMoteIR  ir;
Preferences prefs;

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------
#define IR_MAX_RAW        128     // raw durations stored per button
#define LEARN_HOLD_MS     2000    // how long A+B must be held
#define LEARN_TIMEOUT_MS  30000   // give up and return to normal use
#define CHASE_STEP_MS     120     // "listening" animation speed
#define PULSE_STEP_MS     500     // "got a code, pick a button" pulse speed
#define SEND_RUMBLE_MS    30      // tactile confirmation when transmitting
#define IR_CARRIER_KHZ    38
#define IDLE_SLEEP_MS     600000  // no input for this long (10 min) -> deep sleep

// ---------------------------------------------------------------------------
// Stored mapping — one per button, persisted to NVS
//
// Both types are defined up here, above the first function, so that the
// prototypes the IDE generates for this sketch land after them.
// ---------------------------------------------------------------------------
struct IRSlot {
    uint8_t  valid;
    uint8_t  isNec;
    uint32_t code;                 // NEC: the 32 bits, first bit received in b31
    uint16_t rawLen;               // otherwise: how many durations are used
    uint16_t raw[IR_MAX_RAW];      // alternating mark, space, in microseconds
};

static IRSlot slots[BTN_COUNT];
static IRSlot pending;             // captured, not yet bound to a button

enum State { ST_IDLE, ST_WAIT_IR, ST_WAIT_BUTTON };
static State    state       = ST_IDLE;
static uint32_t stateSince  = 0;
static bool     comboLatched = false;
static uint16_t suppressRelease = 0;   // buttons whose next release is ignored

// ---------------------------------------------------------------------------
// Logging
//
// Deliberately NOT gated on `if (Serial)`. That idiom looks right but breaks on
// this board: with USB Mode = Hardware CDC and JTAG, `Serial` is an HWCDC, and
// HWCDC::isCDC_Connected() returns false on its first call by design — it only
// arms the IN_EMPTY interrupt and flushes the FIFO, reporting true on a later
// call once the host has drained data. Gating on it therefore suppresses the
// very writes that would make it true, and you get silence forever.
//
// The guard is also unnecessary. HWCDC::write() already checks the connection
// itself and, when nothing is listening, discards through a ring-buffer FIFO
// policy rather than blocking — and every path is bounded by a 100 ms tx
// timeout. So writing with no monitor attached is safe and cheap.
// ---------------------------------------------------------------------------
static void logf(const char *fmt, ...) {
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    Serial.print(buf);
}

// ---------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------
static void slotKey(uint8_t i, char *out, size_t n) {
    snprintf(out, n, "b%u", (unsigned)i);
}

static void saveSlot(uint8_t i) {
    char key[8];
    slotKey(i, key, sizeof(key));
    prefs.putBytes(key, &slots[i], sizeof(IRSlot));
}

static void loadAllSlots() {
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        char key[8];
        slotKey(i, key, sizeof(key));
        if (prefs.getBytesLength(key) == sizeof(IRSlot)) {
            prefs.getBytes(key, &slots[i], sizeof(IRSlot));
        } else {
            slots[i].valid = 0;
        }
    }
}

// ---------------------------------------------------------------------------
// Serial backup record
// ---------------------------------------------------------------------------
static void printSlot(uint8_t i, bool withRaw) {
    const IRSlot &s = slots[i];
    if (!s.valid) return;

    if (s.isNec) {
        logf("  %-5s  NEC  0x%08lX\n",
             OpenMote::buttonName((OpenMoteButton)i), (unsigned long)s.code);
    } else {
        logf("  %-5s  RAW  %u durations @ %u kHz\n",
             OpenMote::buttonName((OpenMoteButton)i), s.rawLen, IR_CARRIER_KHZ);
        if (withRaw) {
            for (uint16_t n = 0; n < s.rawLen; n++) {
                logf("%5u%s", s.raw[n], (n % 12 == 11) ? ",\n" : ", ");
            }
            logf("\n");
        }
    }
}

static void printTable() {
    logf("\n--- stored IR mappings ---\n");
    uint8_t count = 0;
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        if (slots[i].valid) { printSlot(i, false); count++; }
    }
    if (count == 0) logf("  (none yet)\n");
    logf("--------------------------\n\n");
}

// ---------------------------------------------------------------------------
// Feedback patterns
// ---------------------------------------------------------------------------
static void feedbackEnterLearn() {
    mote.rumbleOn(255);
    delay(180);
    mote.rumbleOff();
}

// Two rapid flashes and two rumbles: the code is bound.
static void feedbackSaved() {
    for (uint8_t i = 0; i < 2; i++) {
        mote.setLEDs(0b1111);
        mote.rumbleOn(255);
        delay(90);
        mote.setLEDs(0b0000);
        mote.rumbleOff();
        delay(90);
    }
}

// One long buzz: cancelled or timed out, nothing was stored.
static void feedbackAborted() {
    mote.allLEDsOff();
    mote.rumbleOn(160);
    delay(400);
    mote.rumbleOff();
}

// Nothing bound to this button yet.
static void feedbackEmpty() {
    mote.setLED(4, true);
    delay(60);
    mote.setLED(4, false);
}

// Non-blocking animations, stepped from the main loop.
static void animateChase() {
    uint8_t step = (millis() / CHASE_STEP_MS) % 4;
    mote.setLEDs((uint8_t)(1 << step));
}

static void animatePulse() {
    bool on = ((millis() / PULSE_STEP_MS) % 2) == 0;
    mote.setLEDs(on ? 0b1111 : 0b0000);
}

// ---------------------------------------------------------------------------
// State changes
// ---------------------------------------------------------------------------
static void enterState(State s) {
    state      = s;
    stateSince = millis();
}

static void toIdle(bool aborted) {
    if (aborted) feedbackAborted();
    mote.allLEDsOff();
    enterState(ST_IDLE);
    ir.resume();
}

// A+B held together. Latched so it fires once per hold, not every loop.
static bool comboTriggered() {
    bool bothDown = mote.buttonDown(BTN_A) && mote.buttonDown(BTN_B);
    if (!bothDown) { comboLatched = false; return false; }
    if (comboLatched) return false;

    if (mote.buttonHeldFor(BTN_A, LEARN_HOLD_MS) &&
        mote.buttonHeldFor(BTN_B, LEARN_HOLD_MS)) {
        comboLatched = true;
        // Neither button should also fire its normal action on the way out.
        suppressRelease |= (uint16_t)((1 << BTN_A) | (1 << BTN_B));
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Transmit
// ---------------------------------------------------------------------------
static void transmit(uint8_t i) {
    const IRSlot &s = slots[i];
    const char *name = OpenMote::buttonName((OpenMoteButton)i);

    if (!s.valid) {
        logf("%s: nothing bound\n", name);
        feedbackEmpty();
        return;
    }

    mote.setLED(1, true);
    if (s.isNec) {
        logf("%s: sending NEC 0x%08lX\n", name, (unsigned long)s.code);
        ir.sendNECRaw(s.code);
    } else {
        logf("%s: sending %u raw durations\n", name, s.rawLen);
        ir.sendRaw(s.raw, s.rawLen, IR_CARRIER_KHZ);
    }
    mote.rumble(SEND_RUMBLE_MS);
    delay(40);
    mote.setLED(1, false);

    // Our own LED is bright enough for our own receiver to hear it. Drop that
    // echo, or the next available() would return our own transmission.
    ir.resume();
}

// ---------------------------------------------------------------------------
// Capture
// ---------------------------------------------------------------------------
static bool capturePending() {
    memset(&pending, 0, sizeof(pending));

    if (ir.isRepeat()) return false;      // a held-button frame carries no data

    pending.valid = 1;
    if (ir.isNEC()) {
        pending.isNec = 1;
        pending.code  = ir.code();
        logf("captured NEC 0x%08lX  (address 0x%02X, command 0x%02X)\n",
             (unsigned long)pending.code, ir.address(), ir.command());
    } else {
        uint16_t n = ir.rawLength();
        if (n > IR_MAX_RAW) {
            logf("captured raw frame of %u durations, truncated to %u\n",
                 n, (unsigned)IR_MAX_RAW);
            n = IR_MAX_RAW;
        } else {
            logf("captured raw frame, %u durations (not NEC)\n", n);
        }
        memcpy(pending.raw, ir.rawBuffer(), (size_t)n * sizeof(uint16_t));
        pending.rawLen = n;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Per-state loops
// ---------------------------------------------------------------------------
static void loopIdle() {
    if (comboTriggered()) {
        logf("\n=== learn mode: waiting for an IR code ===\n");
        feedbackEnterLearn();
        ir.resume();
        enterState(ST_WAIT_IR);
        return;
    }

    // Transmit on release rather than press, so that starting the A+B combo
    // does not fire A's and B's own codes on the way in.
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        OpenMoteButton b = (OpenMoteButton)i;
        if (!mote.buttonReleased(b)) continue;

        uint16_t bit = (uint16_t)(1 << i);
        if (suppressRelease & bit) { suppressRelease &= (uint16_t)~bit; continue; }
        transmit(i);
    }

    // Keep the receiver drained, and log anything seen for debugging.
    if (ir.available()) {
        if (ir.isNEC() && !ir.isRepeat()) {
            logf("(idle) saw NEC 0x%08lX — hold A+B for 2 s to bind it\n",
                 (unsigned long)ir.code());
        }
        ir.resume();
    }
}

static void loopWaitIR() {
    animateChase();

    if (comboTriggered()) {
        logf("learn mode cancelled\n");
        toIdle(true);
        return;
    }
    if (millis() - stateSince > LEARN_TIMEOUT_MS) {
        logf("learn mode timed out — no IR received\n");
        toIdle(true);
        return;
    }

    if (ir.available()) {
        bool got = capturePending();
        ir.resume();
        if (got) {
            logf("now press the button to bind it to\n");
            enterState(ST_WAIT_BUTTON);
        }
    }
}

static void loopWaitButton() {
    animatePulse();

    if (millis() - stateSince > LEARN_TIMEOUT_MS) {
        logf("learn mode timed out — code discarded\n");
        toIdle(true);
        return;
    }

    // Bind on press: this is the one place a bare A or B press is meaningful.
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        OpenMoteButton b = (OpenMoteButton)i;
        if (!mote.buttonPressed(b)) continue;

        bool overwrote = slots[i].valid;
        slots[i] = pending;
        saveSlot(i);

        // Don't let the release that follows immediately transmit it back.
        suppressRelease |= (uint16_t)(1 << i);

        logf("bound to %s%s\n", OpenMote::buttonName(b),
             overwrote ? " (overwrote previous code)" : "");
        printSlot(i, true);              // full backup record, raw included
        feedbackSaved();
        printTable();
        toIdle(false);
        return;
    }

    if (ir.available()) ir.resume();     // ignore further codes while binding
}

// ---------------------------------------------------------------------------
// Idle deep sleep
//
// Only GPIO0-21 are RTC GPIOs on the ESP32-S3, and only an RTC GPIO can wake
// the chip from deep sleep. That rules out A (38), Left (40), B (42), Up (43)
// and Down (44) — they simply cannot be wake sources on this board.
//
// POWER (GPIO0) is RTC-capable but is deliberately NOT in this list: it is the
// BOOT strapping pin. Waking from deep sleep resets the chip, and holding GPIO0
// low across a reset is exactly the gesture that enters download mode, so using
// it as the wake button can drop you into the bootloader instead of the sketch.
// ---------------------------------------------------------------------------
static const gpio_num_t kWakePins[] = {
    GPIO_NUM_17,   // Home
    GPIO_NUM_18,   // Button 1
    GPIO_NUM_8,    // Button 2
    GPIO_NUM_4,    // Plus
    GPIO_NUM_13,   // Minus
    GPIO_NUM_2,    // D-pad Right
};

static uint32_t lastActivity = 0;

static void enterDeepSleep() {
    logf("\nidle for %u min — entering deep sleep\n", (unsigned)(IDLE_SLEEP_MS / 60000));
    logf("press Home, 1, 2, +, - or D-pad Right to wake\n");
    logf("(the USB serial port will disappear until it wakes)\n");
    Serial.flush();

    mote.allLEDsOff();
    mote.rumbleOff();
    ir.end();                    // release the receive interrupt and PWM channel

    uint64_t mask = 0;
    for (uint8_t i = 0; i < sizeof(kWakePins) / sizeof(kWakePins[0]); i++) {
        // The INPUT_PULLUP set by mote.begin() is a digital pull-up and powers
        // down with the rest of the digital domain. Without the RTC pull-up the
        // pin floats in sleep, which gives phantom wakes or none at all.
        rtc_gpio_pullup_en(kWakePins[i]);
        rtc_gpio_pulldown_dis(kWakePins[i]);
        mask |= 1ULL << (uint8_t)kWakePins[i];
    }

    // Buttons are active low, so wake when any of them is pulled down.
    esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_LOW);
    esp_deep_sleep_start();      // does not return; the board reboots on wake
}

// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);

    // Wait for a monitor to attach, but give up so a board running standalone
    // is never held up. Native USB usually needs 1-2 s to enumerate and reopen
    // the port after a reset, which is why the banner used to go missing.
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) delay(10);
    delay(200);                          // let the host finish opening the port

    // No IMU needed here, so skip sensor bring-up and leave the I2C bus alone.
    mote.begin(false);
    ir.begin();

    prefs.begin("irlearn", false);
    loadAllSlots();

    logf("\nOpenMote — IR Learn & Replay\n");
    logf("Hold A + B for 2 s to capture a code, then press a button to bind it.\n");
    printTable();

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
        logf("(woke from deep sleep)\n");
    }

    // Brief sweep so it is obvious the firmware is alive without a monitor.
    for (uint8_t i = 1; i <= 4; i++) { mote.setLED(i, true); delay(70); }
    mote.allLEDsOff();

    lastActivity = millis();
}

void loop() {
    mote.update();

    // Any button touched, or any learn step in progress, counts as activity.
    if (mote.buttonMask() != 0 || state != ST_IDLE) lastActivity = millis();

    switch (state) {
        case ST_IDLE:        loopIdle();       break;
        case ST_WAIT_IR:     loopWaitIR();     break;
        case ST_WAIT_BUTTON: loopWaitButton(); break;
    }

    // Only sleep from idle — never mid-learn, where a capture would be lost.
    if (state == ST_IDLE && millis() - lastActivity >= IDLE_SLEEP_MS) {
        enterDeepSleep();
    }

    delay(2);
}

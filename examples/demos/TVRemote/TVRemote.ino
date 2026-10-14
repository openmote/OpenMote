/*
 * OpenMote Gen 3.3 - BLE TV Remote  (Arduino IDE version)
 *
 * A Bluetooth LE remote for a Fire TV (or anything that takes a BLE keyboard),
 * plus an infrared TV power button.
 *
 *   POWER            replay the stored TV power IR code (works with BLE down)
 *   A                Select / OK          (Enter)
 *   B                Back                 (Escape)
 *   HOME             Home menu            (consumer control: AC Home)
 *   D-pad            Navigation           (arrow keys)
 *   + / -            Volume up / down     (consumer control, repeats when held)
 *   1 / 2            unused
 *
 * LED 2 slow-blinks while BLE is disconnected.
 *
 * Switching between Bluetooth demos
 *   This sketch and demos/AirMouseBLE use the same Bluetooth address on a given
 *   remote. If a TV or computer was paired with one of them, remove the remote
 *   from that device's Bluetooth list before pairing the other - otherwise it
 *   tries to reuse the old pairing and the connection fails or drops.
 *
 * ---------------------------------------------------------------------------
 * No extra libraries needed
 *
 * The original demo used ESP32-BLE-Keyboard, which pins you to esp32 core 2.x
 * and needs a -DUSE_NIMBLE build flag that the Arduino IDE cannot set. This
 * version builds the HID service directly on the BLE library bundled with esp32
 * core 3.x, so the only thing to install is the OpenMote library itself.
 *
 * Infrared goes through OpenMoteIR rather than IRremoteESP8266. For a plain
 * 32-bit NEC frame the two are equivalent - both clock bit 31 out first. If you
 * need a long or unusual protocol (air conditioners), use IRremoteESP8266 with
 * OM_PIN_IR_TX instead.
 *
 * Arduino IDE settings: the standard OpenMote set from ARDUINO_IDE_SETUP.md.
 * Bluedroid BLE is not small - if the sketch overflows, pick a Partition Scheme
 * with a larger app slot, e.g. "16M Flash (3MB APP/9.9MB FATFS)".
 * ---------------------------------------------------------------------------
 */
#include <OpenMote.h>
#include <OpenMoteIR.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEHIDDevice.h>
#include <BLESecurity.h>

OpenMote   mote;
OpenMoteIR ir;

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------
#define TV_POWER_IR_CODE     0xB9467D02UL   // the NEC frame the Power button sends
#define VOLUME_REPEAT_MS     200            // auto-repeat rate while +/- is held
#define BLINK_MS             1000           // "BLE disconnected" blink period

// HID keyboard usage codes (HID Usage Table, page 0x07)
#define KEY_ENTER            0x28
#define KEY_ESCAPE           0x29
#define KEY_ARROW_RIGHT      0x4F
#define KEY_ARROW_LEFT       0x50
#define KEY_ARROW_DOWN       0x51
#define KEY_ARROW_UP         0x52

// Consumer control usages (HID Usage Table, page 0x0C)
#define CC_VOLUME_UP         0x00E9
#define CC_VOLUME_DOWN       0x00EA
#define CC_HOME              0x0223         // AC Home

// ---------------------------------------------------------------------------
// HID report map
//
// Two collections sharing one connection:
//   report 1  standard 8-byte keyboard  (modifiers, reserved, 6 key slots)
//   report 2  16-bit consumer control   (volume, home - not on the keyboard page)
//
// The consumer collection is what makes volume and Home work; they are not
// keyboard keycodes, which is why the original needed KEY_MEDIA_* helpers.
// ---------------------------------------------------------------------------
static const uint8_t kReportMap[] = {
    // ---- Keyboard, report ID 1 ----
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x05, 0x07,        //   Usage Page (Keyboard/Keypad)
    0x19, 0xE0,        //   Usage Minimum (Left Control)
    0x29, 0xE7,        //   Usage Maximum (Right GUI)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data,Var,Abs)   - modifier byte
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x03,        //   Input (Const,Var,Abs)  - reserved byte
    0x95, 0x06,        //   Report Count (6)
    0x75, 0x08,        //   Report Size (8)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (101)
    0x05, 0x07,        //   Usage Page (Keyboard/Keypad)
    0x19, 0x00,        //   Usage Minimum (0)
    0x29, 0x65,        //   Usage Maximum (101)
    0x81, 0x00,        //   Input (Data,Array,Abs) - 6 simultaneous keys
    0xC0,              // End Collection

    // ---- Consumer control, report ID 2 ----
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x02,        //   Report ID (2)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x03,  //   Logical Maximum (1023)
    0x19, 0x00,        //   Usage Minimum (0)
    0x2A, 0xFF, 0x03,  //   Usage Maximum (1023)
    0x75, 0x10,        //   Report Size (16)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x00,        //   Input (Data,Array,Abs)
    0xC0               // End Collection
};

// ---------------------------------------------------------------------------
// BLE state
// ---------------------------------------------------------------------------
static BLEHIDDevice     *hid           = nullptr;
static BLECharacteristic *inKeyboard   = nullptr;
static BLECharacteristic *inConsumer   = nullptr;
static volatile bool     bleConnected  = false;
static bool              wasConnected  = false;

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *) override {
        bleConnected = true;
    }
    void onDisconnect(BLEServer *) override {
        bleConnected = false;
        // Without this the remote never comes back after the TV drops the link.
        BLEDevice::startAdvertising();
    }
};

static void bleBegin() {
    BLEDevice::init("OpenMote");

    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    hid        = new BLEHIDDevice(server);
    inKeyboard = hid->inputReport(1);
    inConsumer = hid->inputReport(2);

    hid->manufacturer()->setValue("OpenMote.io");
    hid->pnp(0x02, 0xE502, 0xA111, 0x0210);
    hid->hidInfo(0x00, 0x01);

    // Bonding, so the TV reconnects on its own after a power cycle.
    BLESecurity *security = new BLESecurity();
    security->setAuthenticationMode(ESP_LE_AUTH_BOND);

    hid->reportMap((uint8_t *)kReportMap, sizeof(kReportMap));
    hid->startServices();
    hid->setBatteryLevel(100);

    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->setAppearance(0x03C1);                    // HID keyboard
    adv->addServiceUUID(hid->hidService()->getUUID());
    adv->setScanResponse(true);
    adv->start();
}

// A HID key is "pressed" then "released" - without the empty report the host
// sees the key as held down forever.
static void sendKey(uint8_t keycode) {
    uint8_t rpt[8] = {0};
    rpt[2] = keycode;
    inKeyboard->setValue(rpt, sizeof(rpt));
    inKeyboard->notify();

    delay(10);
    rpt[2] = 0;
    inKeyboard->setValue(rpt, sizeof(rpt));
    inKeyboard->notify();
}

static void sendConsumer(uint16_t usage) {
    uint8_t rpt[2] = { (uint8_t)(usage & 0xFF), (uint8_t)(usage >> 8) };
    inConsumer->setValue(rpt, sizeof(rpt));
    inConsumer->notify();

    delay(10);
    rpt[0] = rpt[1] = 0;
    inConsumer->setValue(rpt, sizeof(rpt));
    inConsumer->notify();
}

// ---------------------------------------------------------------------------
// LED feedback - same vocabulary as the original demo
// ---------------------------------------------------------------------------
static void flash(uint8_t mask, uint16_t ms) {
    mote.setLEDs(mask);
    delay(ms);
    mote.allLEDsOff();
}

#define FB_POWER()     flash(0b1111, 100)
#define FB_SELECT()    flash(0b0101,  80)   // LED 1 + 3
#define FB_NAV()       flash(0b0010,  60)   // LED 2
#define FB_VOL_UP()    flash(0b0011,  70)   // LED 1 + 2
#define FB_VOL_DOWN()  flash(0b1100,  70)   // LED 3 + 4
#define FB_BACK()      flash(0b1010,  80)   // LED 2 + 4
#define FB_HOME()      flash(0b1111, 120)

// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(600);

    mote.begin(false);          // no IMU needed
    ir.begin(false);            // transmit only - no receive interrupt

    Serial.println("\nOpenMote Gen 3.3 - BLE TV Remote");
    Serial.println("Starting BLE...");
    bleBegin();
    Serial.println("Advertising as \"OpenMote\". Pair from the TV.");
    Serial.printf("Power button sends IR 0x%08lX\n", (unsigned long)TV_POWER_IR_CODE);

    // Startup cascade, so it is obvious the firmware is alive.
    for (uint8_t i = 1; i <= 4; i++) { mote.setLED(i, true); delay(100); }
    mote.allLEDsOff();
}

void loop() {
    mote.update();
    uint32_t now = millis();

    // ---- Power: IR, and deliberately NOT gated on the BLE link ----
    if (mote.buttonPressed(BTN_POWER)) {
        Serial.printf(">>> POWER - sending IR 0x%08lX\n", (unsigned long)TV_POWER_IR_CODE);
        ir.sendNECRaw(TV_POWER_IR_CODE);
        FB_POWER();
    }

    // ---- Everything else needs the BLE link ----
    if (bleConnected) {
        if (!wasConnected) {
            wasConnected = true;
            Serial.println(">>> TV connected");
            flash(0b1111, 100);
        }

        if (mote.buttonPressed(BTN_A))     { Serial.println(">>> Select");  sendKey(KEY_ENTER);       FB_SELECT(); }
        if (mote.buttonPressed(BTN_B))     { Serial.println(">>> Back");    sendKey(KEY_ESCAPE);      FB_BACK();   }
        if (mote.buttonPressed(BTN_UP))    { Serial.println(">>> Up");      sendKey(KEY_ARROW_UP);    FB_NAV();    }
        if (mote.buttonPressed(BTN_DOWN))  { Serial.println(">>> Down");    sendKey(KEY_ARROW_DOWN);  FB_NAV();    }
        if (mote.buttonPressed(BTN_LEFT))  { Serial.println(">>> Left");    sendKey(KEY_ARROW_LEFT);  FB_NAV();    }
        if (mote.buttonPressed(BTN_RIGHT)) { Serial.println(">>> Right");   sendKey(KEY_ARROW_RIGHT); FB_NAV();    }
        if (mote.buttonPressed(BTN_HOME))  { Serial.println(">>> Home");    sendConsumer(CC_HOME);    FB_HOME();   }

        // Volume repeats while held: fire on the press edge, then every
        // VOLUME_REPEAT_MS for as long as the button stays down.
        static uint32_t lastVolume = 0;
        bool plus  = mote.buttonDown(BTN_PLUS);
        bool minus = mote.buttonDown(BTN_MINUS);

        if (plus || minus) {
            bool edge = mote.buttonPressed(BTN_PLUS) || mote.buttonPressed(BTN_MINUS);
            if (edge || now - lastVolume >= VOLUME_REPEAT_MS) {
                lastVolume = now;
                if (plus) { Serial.println(">>> Volume Up");   sendConsumer(CC_VOLUME_UP);   FB_VOL_UP();   }
                else      { Serial.println(">>> Volume Down"); sendConsumer(CC_VOLUME_DOWN); FB_VOL_DOWN(); }
            }
        }

    } else {
        if (wasConnected) {
            wasConnected = false;
            Serial.println(">>> TV disconnected - advertising again");
        }

        // Slow blink on LED 2 while unpaired.
        mote.setLED(2, (now / BLINK_MS) % 2);
    }

    delay(5);
}

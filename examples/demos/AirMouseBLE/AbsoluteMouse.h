#ifndef ABSOLUTE_MOUSE_H
#define ABSOLUTE_MOUSE_H

// ============================================================
// Absolute-positioning Bluetooth LE HID pointer for the OpenMote air mouse.
//
// Same report as the USB version in demos/AirMouse (3 buttons, absolute X/Y in
// 0..32767, relative wheel) and the same moveTo/press/release/scroll API, but
// carried over HID-over-GATT instead of TinyUSB so the remote works untethered.
//
// Built on the BLE library bundled with esp32 core 3.3+ (NimBLE on the S3) -
// no external libraries. The HID-over-GATT services are declared here by hand
// instead of through BLEHIDDevice, because BLEHIDDevice leaves every HID
// characteristic readable without encryption, and macOS will not run a pointer
// whose HID service it enumerated over an unencrypted link.
// ============================================================

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEHIDDevice.h>      // HID_MOUSE appearance value only
#include <BLESecurity.h>
#include <BLEDescriptor.h>

#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 3, 0)
#error "AirMouseBLE needs esp32 core 3.3.0 or newer - update it in Boards Manager"
#endif

#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 3, 8)
#if !defined(CONFIG_NIMBLE_ENABLED)
#error "AirMouseBLE needs esp32 core 3.3.8 or newer on this chip - update it in Boards Manager"
#endif
#include <host/ble_hs.h>       // ble_hs_cfg, see begin()
#endif

#define ABS_MOUSE_LOGICAL_MAX 32767  // full-scale absolute coordinate

// Buttons bitmask
#define ABS_MOUSE_LEFT   0x01
#define ABS_MOUSE_RIGHT  0x02
#define ABS_MOUSE_MIDDLE 0x04

// A BLE link only carries one batch of notifications per connection interval
// (7.5 ms at best), while the fusion loop runs far faster than that. Position
// updates are throttled to this period so the notify queue never backs up;
// button and wheel reports always go out immediately.
#define ABS_MOUSE_MOVE_INTERVAL_MS 10

class AbsoluteMouse : public BLEServerCallbacks, public BLESecurityCallbacks {
public:
    AbsoluteMouse() : _input(nullptr), _connected(false), _encrypted(false),
                      _pairEvent(0), _buttons(0), _x(0), _y(0), _lastMoveMs(0) {}

    void begin(const char *name = "OpenMote Air Mouse") {
        BLEDevice::init(name);

        // "Just Works" pairing with bonding: the remote has no display and no
        // keypad to confirm a passkey with.
        BLESecurity *security = new BLESecurity();
        security->setCapability(ESP_IO_CAP_NONE);
        BLEDevice::setSecurityCallbacks(this);

        // Don't push a Security Request at the host the moment it connects.
        // Apple hosts expect to start pairing themselves, when a protected
        // attribute turns them away (the report map and input report below); a
        // request racing their own service discovery makes macOS drop the link.
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 8)
        security->setAuthenticationMode(true, false, true);  // bonding, no MITM, secure connections
        BLESecurity::setForceAuthentication(false);
#else
        // Cores before 3.3.8 always send that request once setAuthenticationMode()
        // has been called, and have no switch to turn it off. Set the same
        // pairing parameters on the NimBLE host directly instead: the library
        // then never initiates security, and still pairs whenever the host asks.
        ble_hs_cfg.sm_bonding = 1;
        ble_hs_cfg.sm_mitm    = 0;
        ble_hs_cfg.sm_sc      = 1;
#endif

        BLEServer *server = BLEDevice::createServer();
        server->setCallbacks(this);

        const uint32_t RD     = BLECharacteristic::PROPERTY_READ;
        // Protected read: the host is refused with "insufficient encryption"
        // until it has paired, which is what makes it start pairing.
        const uint32_t RD_ENC = RD | BLECharacteristic::PROPERTY_READ_ENC;

        // ---- Device Information ----
        BLEService *dis = server->createService(BLEUUID((uint16_t)0x180a));
        dis->createCharacteristic((uint16_t)0x2a29, RD)->setValue("OpenMote.io");
        uint8_t pnp[] = { 0x02, 0xE5, 0x02, 0xA1, 0x12, 0x02, 0x10 };  // USB-IF, VID, PID, version
        dis->createCharacteristic((uint16_t)0x2a50, RD)->setValue(pnp, sizeof(pnp));

        // ---- HID ----
        BLEService *hid = server->createService(BLEUUID((uint16_t)0x1812), 40);
        uint8_t info[] = { 0x11, 0x01, 0x00, 0x01 };                   // HID 1.11, no country, remote wake
        hid->createCharacteristic((uint16_t)0x2a4a, RD)->setValue(info, sizeof(info));
        hid->createCharacteristic((uint16_t)0x2a4b, RD_ENC)
           ->setValue((uint8_t *)reportDescriptor, sizeof(reportDescriptor));
        hid->createCharacteristic((uint16_t)0x2a4c, BLECharacteristic::PROPERTY_WRITE_NR);
        uint8_t mode[] = { 0x01 };                                     // report protocol
        hid->createCharacteristic((uint16_t)0x2a4e, RD | BLECharacteristic::PROPERTY_WRITE_NR)
           ->setValue(mode, sizeof(mode));

        _input = hid->createCharacteristic((uint16_t)0x2a4d, RD_ENC | BLECharacteristic::PROPERTY_NOTIFY);
        // Report Reference descriptor: {report ID, 0x01 = input}.
        BLEDescriptor *ref = new BLEDescriptor(BLEUUID((uint16_t)0x2908));
        ref->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
        uint8_t refVal[] = { HID_REPORT_ID, 0x01 };
        ref->setValue(refVal, sizeof(refVal));
        _input->addDescriptor(ref);

        // ---- Battery ----
        BLEService *bat = server->createService(BLEUUID((uint16_t)0x180f));
        uint8_t level[] = { 100 };
        bat->createCharacteristic((uint16_t)0x2a19, RD | BLECharacteristic::PROPERTY_NOTIFY)
           ->setValue(level, sizeof(level));

        dis->start();
        hid->start();
        bat->start();
        server->start();

        BLEAdvertising *advertising = BLEDevice::getAdvertising();
        advertising->setAppearance(HID_MOUSE);
        advertising->addServiceUUID(hid->getUUID());
        advertising->setScanResponse(true);
        advertising->setMinPreferred(0x06);
        advertising->setMaxPreferred(0x12);
        BLEDevice::startAdvertising();
    }

    // True once a host is connected AND the link is encrypted (paired). HID
    // reports are never sent over an unencrypted link.
    bool ready() const { return _connected && _encrypted; }

    // Pairing result since the last call: 0 none, +1 paired, -1 failed. The BLE
    // callbacks run on the BLE host task and must never block on Serial, so
    // they only set flags; the sketch's loop does the logging.
    int8_t takePairEvent() { int8_t e = _pairEvent; _pairEvent = 0; return e; }

    // Move to an absolute screen position (0..32767 on each axis).
    void moveTo(uint16_t x, uint16_t y) {
        _x = clampAbs(x);
        _y = clampAbs(y);
        uint32_t now = millis();
        if (now - _lastMoveMs < ABS_MOUSE_MOVE_INTERVAL_MS) return;
        _lastMoveMs = now;
        sendReport();
    }

    void press(uint8_t b = ABS_MOUSE_LEFT)   { _buttons |= b;  sendReport(); }
    void release(uint8_t b = ABS_MOUSE_LEFT)  { _buttons &= ~b; sendReport(); }
    void click(uint8_t b = ABS_MOUSE_LEFT)    { press(b); release(b); }
    bool isPressed(uint8_t b) const           { return _buttons & b; }

    // Vertical scroll: +up / -down (relative wheel).
    void scroll(int8_t amount) { sendReport(amount); }

private:
    static uint16_t clampAbs(uint16_t v) {
        return v > ABS_MOUSE_LOGICAL_MAX ? ABS_MOUSE_LOGICAL_MAX : v;
    }

    // The report ID is implied by the characteristic, so it is not in the payload.
    void sendReport(int8_t wheel = 0) {
        if (!ready()) return;
        uint8_t report[6];
        report[0] = _buttons;
        report[1] = (uint8_t)(_x & 0xFF);
        report[2] = (uint8_t)(_x >> 8);
        report[3] = (uint8_t)(_y & 0xFF);
        report[4] = (uint8_t)(_y >> 8);
        report[5] = (uint8_t)wheel;
        _input->setValue(report, sizeof(report));
        _input->notify();
    }

    // ---- BLEServerCallbacks ----
    void onConnect(BLEServer *) override {
        _connected = true;
    }
    void onDisconnect(BLEServer *) override {
        _connected = false;
        _encrypted = false;
        _buttons = 0;                        // don't resume mid-drag on reconnect
        BLEDevice::startAdvertising();
    }

    // ---- BLESecurityCallbacks ----
    bool onSecurityRequest() override { return true; }
#if defined(CONFIG_BLUEDROID_ENABLED)
    void onAuthenticationComplete(esp_ble_auth_cmpl_t auth_cmpl) override {
        _encrypted = auth_cmpl.success;
        _pairEvent = auth_cmpl.success ? 1 : -1;
    }
#elif defined(CONFIG_NIMBLE_ENABLED)
    void onAuthenticationComplete(ble_gap_conn_desc *desc) override {
        _encrypted = desc->sec_state.encrypted;
        _pairEvent = desc->sec_state.encrypted ? 1 : -1;
    }
#endif
    uint32_t onPassKeyRequest() override { return 0; }
    void     onPassKeyNotify(uint32_t) override {}
    bool     onConfirmPIN(uint32_t) override { return true; }

    static const uint8_t HID_REPORT_ID = 1;
    BLECharacteristic *_input;
    volatile bool      _connected;
    volatile bool      _encrypted;
    volatile int8_t    _pairEvent;
    uint8_t  _buttons;
    uint16_t _x, _y;
    uint32_t _lastMoveMs;

    // Absolute pointer: 3 buttons + 5 bits padding, 16-bit absolute X/Y, 8-bit wheel.
    static constexpr uint8_t reportDescriptor[66] = {
        0x05, 0x01,                    // Usage Page (Generic Desktop)
        0x09, 0x02,                    // Usage (Mouse)
        0xA1, 0x01,                    // Collection (Application)
        0x85, HID_REPORT_ID,           //   Report ID (1)
        0x09, 0x01,                    //   Usage (Pointer)
        0xA1, 0x00,                    //   Collection (Physical)
        // --- Buttons (3) ---
        0x05, 0x09,                    //     Usage Page (Button)
        0x19, 0x01,                    //     Usage Minimum (Button 1)
        0x29, 0x03,                    //     Usage Maximum (Button 3)
        0x15, 0x00,                    //     Logical Minimum (0)
        0x25, 0x01,                    //     Logical Maximum (1)
        0x95, 0x03,                    //     Report Count (3)
        0x75, 0x01,                    //     Report Size (1)
        0x81, 0x02,                    //     Input (Data,Var,Abs)
        // --- Button padding (5 bits) ---
        0x95, 0x01,                    //     Report Count (1)
        0x75, 0x05,                    //     Report Size (5)
        0x81, 0x03,                    //     Input (Const,Var,Abs)
        // --- Absolute X/Y (16-bit, 0..32767) ---
        0x05, 0x01,                    //     Usage Page (Generic Desktop)
        0x09, 0x30,                    //     Usage (X)
        0x09, 0x31,                    //     Usage (Y)
        0x16, 0x00, 0x00,              //     Logical Minimum (0)
        0x26, 0xFF, 0x7F,              //     Logical Maximum (32767)
        0x75, 0x10,                    //     Report Size (16)
        0x95, 0x02,                    //     Report Count (2)
        0x81, 0x02,                    //     Input (Data,Var,Abs)
        // --- Wheel (relative, -127..127) ---
        0x09, 0x38,                    //     Usage (Wheel)
        0x15, 0x81,                    //     Logical Minimum (-127)
        0x25, 0x7F,                    //     Logical Maximum (127)
        0x75, 0x08,                    //     Report Size (8)
        0x95, 0x01,                    //     Report Count (1)
        0x81, 0x06,                    //     Input (Data,Var,Rel)
        0xC0,                          //   End Collection
        0xC0                           // End Collection
    };
};

#endif // ABSOLUTE_MOUSE_H

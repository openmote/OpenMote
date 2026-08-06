#ifndef ABSOLUTE_MOUSE_H
#define ABSOLUTE_MOUSE_H

// ============================================================
// Absolute-positioning USB HID pointer for the OpenMote air mouse.
//
// A normal HID mouse reports *relative* deltas, which can only ever nudge the
// cursor - it can't put the cursor "where you point." To get the invisible-line
// feel the cursor must be told an ABSOLUTE screen coordinate, so this device
// uses a digitizer-style report: X and Y are absolute values in 0..32767 that
// the OS maps across the whole desktop. Buttons + wheel ride along the same
// report so click / drag / scroll all work.
//
// Built directly on the ESP32 Arduino TinyUSB stack (USB.h / USBHID.h) - no
// external libraries. Requires ARDUINO_USB_MODE=1.
// ============================================================

#include "USB.h"
#include "USBHID.h"

#define ABS_MOUSE_LOGICAL_MAX 32767  // full-scale absolute coordinate

// Buttons bitmask
#define ABS_MOUSE_LEFT   0x01
#define ABS_MOUSE_RIGHT  0x02
#define ABS_MOUSE_MIDDLE 0x04

class AbsoluteMouse : public USBHIDDevice {
public:
    AbsoluteMouse() : _buttons(0), _x(0), _y(0) {
        static bool initialized = false;
        if (!initialized) {
            initialized = true;
            hid.addDevice(this, sizeof(reportDescriptor));
        }
    }

    void begin() { hid.begin(); }

    bool ready() { return hid.ready(); }

    // TinyUSB asks for our report descriptor here.
    uint16_t _onGetDescriptor(uint8_t *dst) override {
        memcpy(dst, reportDescriptor, sizeof(reportDescriptor));
        return sizeof(reportDescriptor);
    }

    // Move to an absolute screen position (0..32767 on each axis).
    void moveTo(uint16_t x, uint16_t y) {
        _x = clampAbs(x);
        _y = clampAbs(y);
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

    void sendReport(int8_t wheel = 0) {
        uint8_t report[6];
        report[0] = _buttons;
        report[1] = (uint8_t)(_x & 0xFF);
        report[2] = (uint8_t)(_x >> 8);
        report[3] = (uint8_t)(_y & 0xFF);
        report[4] = (uint8_t)(_y >> 8);
        report[5] = (uint8_t)wheel;
        hid.SendReport(HID_REPORT_ID, report, sizeof(report));
    }

    static const uint8_t HID_REPORT_ID = 1;
    USBHID  hid;
    uint8_t _buttons;
    uint16_t _x, _y;

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

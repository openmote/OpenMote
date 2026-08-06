/*
 * OpenMotePins.h — OpenMote Gen 3.3 hardware pin map
 *
 * Every GPIO on the board, in one place. You can include just this header if you
 * want the pin numbers without the rest of the library:
 *
 *     #include <OpenMotePins.h>
 *     pinMode(OM_PIN_LED1, OUTPUT);
 *
 * MCU: ESP32-S3-WROOM-1-N16R8  (16 MB flash, 8 MB Octal PSRAM)
 * Source of truth: docs/PINOUT.md
 */
#ifndef OPENMOTE_PINS_H
#define OPENMOTE_PINS_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// I2C bus — shared by the IMU and the J15 expansion port
// ---------------------------------------------------------------------------
#define OM_PIN_SCL          15
#define OM_PIN_SDA          16

// ---------------------------------------------------------------------------
// Sensor interrupt lines (LSM6DS3TR-C IMU)
// ---------------------------------------------------------------------------
#define OM_PIN_IMU_INT1     48
#define OM_PIN_IMU_INT2     47

// ---------------------------------------------------------------------------
// Battery voltage sense
// Divider: R19 = 200k (high side) / R26 = 47k (low side)
//   V_batt = V_adc * (247 / 47) = V_adc * 5.2553
// WARNING: GPIO14 is ADC2_CH3, and ADC2 is shared with the Wi-Fi radio.
// Reads fail while Wi-Fi is active — see OpenMote::batteryVolts().
// ---------------------------------------------------------------------------
#define OM_PIN_BATT_ADC     14
#define OM_BATT_DIVIDER     5.2553f   // calibrate per board against a multimeter

// ---------------------------------------------------------------------------
// Audio out — MAX98357A I2S class-D amplifier
// ---------------------------------------------------------------------------
#define OM_PIN_I2S_LRCLK    5
#define OM_PIN_I2S_BCLK     6
#define OM_PIN_I2S_DIN      7
// SD_MODE is an ANALOG mode pin on the MAX98357A, reached through R17 (680k) and
// the JP1 solder jumper. HIGH = enabled, (L+R)/2 mono. LOW = shutdown.
// See docs/PINOUT.md before you rely on this.
#define OM_PIN_AMP_SD       21

// ---------------------------------------------------------------------------
// Audio in — LMD4030T261-OA1 PDM MEMS microphone
// ---------------------------------------------------------------------------
#define OM_PIN_MIC_DATA     11
#define OM_PIN_MIC_CLK      12

// ---------------------------------------------------------------------------
// Haptics — vibration motor via SI2302 N-MOSFET (Q1)
// ---------------------------------------------------------------------------
#define OM_PIN_RUMBLE       39

// ---------------------------------------------------------------------------
// Player LEDs (active high)
// ---------------------------------------------------------------------------
#define OM_PIN_LED1         10
#define OM_PIN_LED2         9
#define OM_PIN_LED3         46   // strapping pin (ROM message)
#define OM_PIN_LED4         3    // strapping pin (JTAG source)

// ---------------------------------------------------------------------------
// Infrared
// ---------------------------------------------------------------------------
#define OM_PIN_IR_RX        1    // IRM-56384 receiver, ACTIVE LOW
#define OM_PIN_IR_TX        41   // IR LED (D6) via MMBT2222A driver

// ---------------------------------------------------------------------------
// Buttons — all active LOW, use INPUT_PULLUP
// ---------------------------------------------------------------------------
#define OM_PIN_BTN_UP       43   // also default U0TXD
#define OM_PIN_BTN_DOWN     44   // also default U0RXD
#define OM_PIN_BTN_LEFT     40
#define OM_PIN_BTN_RIGHT    2
#define OM_PIN_BTN_A        38
#define OM_PIN_BTN_B        42   // trigger
#define OM_PIN_BTN_1        18
#define OM_PIN_BTN_2        8
#define OM_PIN_BTN_PLUS     4
#define OM_PIN_BTN_MINUS    13
#define OM_PIN_BTN_HOME     17
#define OM_PIN_BTN_POWER    0    // also the BOOT strapping pin

// ---------------------------------------------------------------------------
// USB / expansion
// ---------------------------------------------------------------------------
#define OM_PIN_USB_DM       19   // native USB D-
#define OM_PIN_USB_DP       20   // native USB D+
#define OM_PIN_BREAKOUT1    45   // expansion / debug — VDD_SPI strapping pin

// ---------------------------------------------------------------------------
// I2C device addresses
// ---------------------------------------------------------------------------
#define OM_I2C_ADDR_IMU     0x6A  // LSM6DS3TR-C

// ---------------------------------------------------------------------------
// DO NOT USE — consumed internally by flash and Octal PSRAM on the N16R8 module
//   GPIO26-32 : SPI flash
//   GPIO33-37 : Octal (OPI) PSRAM
// Touching these will crash or hang the board.
// ---------------------------------------------------------------------------
#define OM_PIN_IS_RESERVED(p)  (((p) >= 26) && ((p) <= 37))

// ---------------------------------------------------------------------------
// Button identifiers — index into the library's button table.
// Order matters: it must match kButtonPins[] in OpenMote.cpp.
// ---------------------------------------------------------------------------
enum OpenMoteButton : uint8_t {
    BTN_UP = 0,
    BTN_DOWN,
    BTN_LEFT,
    BTN_RIGHT,
    BTN_A,
    BTN_B,
    BTN_1,
    BTN_2,
    BTN_PLUS,
    BTN_MINUS,
    BTN_HOME,
    BTN_POWER,
    BTN_COUNT
};

#endif // OPENMOTE_PINS_H

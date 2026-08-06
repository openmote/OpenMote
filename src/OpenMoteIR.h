/*
 * OpenMoteIR.h — infrared send and receive for the OpenMote Gen 3.3
 *
 * Self-contained: no external IR library needed. Handles the NEC protocol
 * (what most cheap remotes use) plus raw mark/space timing for everything else.
 *
 * It is a separate class from OpenMote because receiving installs a GPIO
 * interrupt, and you should opt into that rather than get it by surprise.
 *
 *     #include <OpenMoteIR.h>
 *     OpenMoteIR ir;
 *
 *     void setup() { ir.begin(); }
 *
 *     void loop() {
 *         if (ir.available()) {
 *             Serial.printf("addr 0x%02X cmd 0x%02X\n", ir.address(), ir.command());
 *             ir.resume();
 *         }
 *     }
 *
 * Hardware: IRM-56384 receiver on GPIO1 (active low), IR LED on GPIO41 through
 * an MMBT2222A. Range is a few metres with line of sight.
 *
 * Caveat: transmit timing is generated in software, so a busy Wi-Fi stack can
 * jitter it. For heavy-duty IR work (air conditioners, long protocols) use
 * IRremoteESP8266 instead — the pin numbers above are all it needs.
 */
#ifndef OPENMOTE_IR_H
#define OPENMOTE_IR_H

#include <Arduino.h>
#include "OpenMotePins.h"

#define OM_IR_RAW_BUFFER_SIZE  200   // edges captured per frame; NEC needs ~68

class OpenMoteIR {
public:
    OpenMoteIR();

    /**
     * Set up the IR pins.
     * @param enableReceive  false for transmit only (no interrupt installed).
     */
    bool begin(bool enableReceive = true);

    /** Release the receive interrupt and the transmit PWM channel. */
    void end();

    // -- Receiving --------------------------------------------------------

    /**
     * True once a complete frame has arrived and the air has gone quiet.
     * The capture buffer stays frozen until you call resume(), so you can take
     * your time reading it out.
     */
    bool available();

    /** Arm the receiver for the next frame. Call this after handling one. */
    void resume();

    /** True if the captured frame decoded as valid NEC (checksum included). */
    bool isNEC() const { return _necValid; }

    /** NEC device address from the last decoded frame. */
    uint8_t address() const { return _address; }

    /** NEC command byte from the last decoded frame. */
    uint8_t command() const { return _command; }

    /**
     * The raw 32 bits in transmission order — first bit received is bit 31.
     * Pass this straight back to sendNECRaw() to replay a captured button.
     */
    uint32_t code() const { return _code; }

    /** True if the frame was an NEC "button still held" repeat, not new data. */
    bool isRepeat() const { return _repeat; }

    /** How many raw mark/space durations were captured. */
    uint16_t rawLength() const { return _frameLen; }

    /**
     * The raw capture, in microseconds, alternating mark then space.
     * Valid only between available() and resume().
     */
    const uint16_t *rawBuffer() const { return (const uint16_t *)_buf; }

    /** Dump the raw capture to a stream, ready to paste into sendRaw(). */
    void printRaw(Print &out = Serial) const;

    // -- Transmitting -----------------------------------------------------

    /** Send a standard NEC frame (8-bit address + 8-bit command). */
    void sendNEC(uint8_t address, uint8_t command);

    /** Replay the exact 32 bits from code(). */
    void sendNECRaw(uint32_t code);

    /** Send the NEC "button held down" repeat frame. */
    void sendNECRepeat();

    /**
     * Send arbitrary timing. durations alternate mark, space, mark, ... in
     * microseconds, starting with a mark. A trailing space is optional.
     */
    void sendRaw(const uint16_t *durations, uint16_t count, uint16_t carrierKHz = 38);

    /** Carrier duty cycle in percent (default 33). Higher = brighter and hotter. */
    void setDutyCycle(uint8_t percent);

private:
    static void IRAM_ATTR isrTrampoline();
    void        IRAM_ATTR handleEdge();
    bool        decodeNEC();
    void        carrierOn();
    void        carrierOff();
    void        markMicros(uint32_t us);
    void        spaceMicros(uint32_t us);

    static OpenMoteIR *_instance;

    volatile uint16_t _buf[OM_IR_RAW_BUFFER_SIZE];
    volatile uint16_t _count    = 0;
    volatile uint32_t _lastEdge = 0;
    volatile bool     _running  = false;
    volatile bool     _frozen   = false;

    uint16_t _frameLen = 0;
    uint32_t _code     = 0;
    uint8_t  _address  = 0;
    uint8_t  _command  = 0;
    bool     _necValid = false;
    bool     _repeat   = false;

    bool     _rxEnabled  = false;
    uint8_t  _duty       = 33;
    uint16_t _carrierKHz = 38;
};

#endif // OPENMOTE_IR_H

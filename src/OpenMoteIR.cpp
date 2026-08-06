#include "OpenMoteIR.h"

// Core 2.x / 3.x LEDC shim — see the note in OpenMote.cpp.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define OM_IR_LEDC_ATTACH(pin, ch, freq, bits)  ledcAttach((pin), (freq), (bits))
  #define OM_IR_LEDC_WRITE(pin, ch, duty)         ledcWrite((pin), (duty))
  #define OM_IR_LEDC_DETACH(pin, ch)              ledcDetach((pin))
#else
  #define OM_IR_LEDC_ATTACH(pin, ch, freq, bits) \
      (ledcSetup((ch), (freq), (bits)), ledcAttachPin((pin), (ch)), true)
  #define OM_IR_LEDC_WRITE(pin, ch, duty)         ledcWrite((ch), (duty))
  #define OM_IR_LEDC_DETACH(pin, ch)              ledcDetachPin((pin))
#endif

#define OM_IR_LEDC_CHANNEL   1      // channel 0 belongs to the rumble motor
#define OM_IR_LEDC_BITS      8

// A frame is finished once the line has been idle this long.
#define OM_IR_GAP_US         15000

// NEC timing, in microseconds.
#define NEC_HDR_MARK         9000
#define NEC_HDR_SPACE        4500
#define NEC_RPT_SPACE        2250
#define NEC_BIT_MARK          560
#define NEC_ONE_SPACE        1690
#define NEC_ZERO_SPACE        560

// Everything is generated in software, so allow generous tolerance.
static inline bool matches(uint16_t measured, uint16_t expected) {
    uint32_t tol = (uint32_t)expected / 4 + 100;   // +/-25% plus a fixed margin
    return (measured + tol >= expected) && (measured <= expected + tol);
}

static inline uint8_t reverseByte(uint8_t b) {
    b = (uint8_t)((b & 0xF0) >> 4 | (b & 0x0F) << 4);
    b = (uint8_t)((b & 0xCC) >> 2 | (b & 0x33) << 2);
    b = (uint8_t)((b & 0xAA) >> 1 | (b & 0x55) << 1);
    return b;
}

OpenMoteIR *OpenMoteIR::_instance = nullptr;

OpenMoteIR::OpenMoteIR() {}

bool OpenMoteIR::begin(bool enableReceive) {
    _instance = this;

    // Transmitter: idle with the carrier off so the LED isn't sitting lit.
    pinMode(OM_PIN_IR_TX, OUTPUT);
    digitalWrite(OM_PIN_IR_TX, LOW);
    OM_IR_LEDC_ATTACH(OM_PIN_IR_TX, OM_IR_LEDC_CHANNEL,
                      _carrierKHz * 1000, OM_IR_LEDC_BITS);
    OM_IR_LEDC_WRITE(OM_PIN_IR_TX, OM_IR_LEDC_CHANNEL, 0);

    if (enableReceive) {
        pinMode(OM_PIN_IR_RX, INPUT);      // module has its own output driver
        _count = 0;
        _running = false;
        _frozen = false;
        attachInterrupt(digitalPinToInterrupt(OM_PIN_IR_RX), isrTrampoline, CHANGE);
        _rxEnabled = true;
    }
    return true;
}

void OpenMoteIR::end() {
    if (_rxEnabled) {
        detachInterrupt(digitalPinToInterrupt(OM_PIN_IR_RX));
        _rxEnabled = false;
    }
    OM_IR_LEDC_WRITE(OM_PIN_IR_TX, OM_IR_LEDC_CHANNEL, 0);
    OM_IR_LEDC_DETACH(OM_PIN_IR_TX, OM_IR_LEDC_CHANNEL);
    _instance = nullptr;
}

// ---------------------------------------------------------------------------
// Receive
// ---------------------------------------------------------------------------

// IRAM_ATTR lives on the declarations in the header; repeating it here makes
// GCC complain about conflicting section attributes.
void OpenMoteIR::isrTrampoline() {
    if (_instance) _instance->handleEdge();
}

void OpenMoteIR::handleEdge() {
    if (_frozen) return;                      // holding a frame for the sketch

    uint32_t now   = micros();
    int      level = digitalRead(OM_PIN_IR_RX);

    if (!_running) {
        // The receiver idles high; a frame opens with a falling edge.
        if (level != LOW) return;
        _running  = true;
        _count    = 0;
        _lastEdge = now;
        return;
    }

    uint32_t delta = now - _lastEdge;
    _lastEdge = now;

    uint16_t n = _count;                       // read/modify/write by hand:
    if (n < OM_IR_RAW_BUFFER_SIZE) {           // ++ on a volatile is deprecated
        _buf[n] = (delta > 65535) ? 65535 : (uint16_t)delta;
        _count  = (uint16_t)(n + 1);
    }
}

bool OpenMoteIR::available() {
    if (_frozen) return true;
    if (!_running || _count < 3) return false;

    // Still receiving? Wait for the line to go quiet.
    noInterrupts();
    uint32_t since = micros() - _lastEdge;
    interrupts();
    if (since < OM_IR_GAP_US) return false;

    _frozen   = true;
    _frameLen = _count;
    _necValid = decodeNEC();
    return true;
}

void OpenMoteIR::resume() {
    noInterrupts();
    _count    = 0;
    _running  = false;
    _frozen   = false;
    interrupts();
}

bool OpenMoteIR::decodeNEC() {
    _code = 0; _address = 0; _command = 0; _repeat = false;

    if (_frameLen < 3) return false;
    if (!matches(_buf[0], NEC_HDR_MARK)) return false;

    // Repeat frame: header mark, short space, one stop mark, done.
    if (matches(_buf[1], NEC_RPT_SPACE)) {
        _repeat = true;
        return true;
    }
    if (!matches(_buf[1], NEC_HDR_SPACE)) return false;

    // 32 bits, each a mark plus a space, then a trailing stop mark.
    if (_frameLen < 2 + 64) return false;

    uint32_t bits = 0;
    for (uint8_t i = 0; i < 32; i++) {
        uint16_t mark  = _buf[2 + i * 2];
        uint16_t space = _buf[3 + i * 2];
        if (!matches(mark, NEC_BIT_MARK)) return false;

        bits <<= 1;                                   // bit 31 = first received
        if (matches(space, NEC_ONE_SPACE))       bits |= 1;
        else if (!matches(space, NEC_ZERO_SPACE)) return false;
    }
    _code = bits;

    // On the wire each byte goes out least-significant bit first.
    uint8_t addr    = reverseByte((uint8_t)((bits >> 24) & 0xFF));
    uint8_t addrInv = reverseByte((uint8_t)((bits >> 16) & 0xFF));
    uint8_t cmd     = reverseByte((uint8_t)((bits >>  8) & 0xFF));
    uint8_t cmdInv  = reverseByte((uint8_t)( bits        & 0xFF));

    // The command checksum is required. The address one is not: extended NEC
    // remotes use those 16 bits as a wider address instead.
    if ((uint8_t)~cmd != cmdInv) return false;
    (void)addrInv;

    _address = addr;
    _command = cmd;
    return true;
}

void OpenMoteIR::printRaw(Print &out) const {
    out.printf("Raw (%u durations, us):\n", _frameLen);
    for (uint16_t i = 0; i < _frameLen; i++) {
        out.printf("%5u%s", _buf[i], (i % 8 == 7) ? ",\n" : ", ");
    }
    out.println();
}

// ---------------------------------------------------------------------------
// Transmit
// ---------------------------------------------------------------------------

void OpenMoteIR::setDutyCycle(uint8_t percent) {
    if (percent < 1)  percent = 1;
    if (percent > 50) percent = 50;   // beyond 50% the LED just gets hot
    _duty = percent;
}

void OpenMoteIR::carrierOn() {
    uint32_t duty = (255UL * _duty) / 100UL;
    OM_IR_LEDC_WRITE(OM_PIN_IR_TX, OM_IR_LEDC_CHANNEL, duty);
}

void OpenMoteIR::carrierOff() {
    OM_IR_LEDC_WRITE(OM_PIN_IR_TX, OM_IR_LEDC_CHANNEL, 0);
}

// delayMicroseconds gets imprecise over a few thousand us, so walk it in steps.
static inline void preciseDelay(uint32_t us) {
    while (us > 8000) { delayMicroseconds(8000); us -= 8000; }
    if (us) delayMicroseconds(us);
}

void OpenMoteIR::markMicros(uint32_t us)  { carrierOn();  preciseDelay(us); }
void OpenMoteIR::spaceMicros(uint32_t us) { carrierOff(); preciseDelay(us); }

void OpenMoteIR::sendRaw(const uint16_t *durations, uint16_t count, uint16_t carrierKHz) {
    if (carrierKHz != _carrierKHz) {
        _carrierKHz = carrierKHz;
        OM_IR_LEDC_ATTACH(OM_PIN_IR_TX, OM_IR_LEDC_CHANNEL,
                          _carrierKHz * 1000, OM_IR_LEDC_BITS);
    }
    for (uint16_t i = 0; i < count; i++) {
        if (i & 1) spaceMicros(durations[i]);
        else       markMicros(durations[i]);
    }
    carrierOff();
}

void OpenMoteIR::sendNECRaw(uint32_t code) {
    markMicros(NEC_HDR_MARK);
    spaceMicros(NEC_HDR_SPACE);
    for (int8_t i = 31; i >= 0; i--) {          // bit 31 first, matching decode
        markMicros(NEC_BIT_MARK);
        spaceMicros((code & (1UL << i)) ? NEC_ONE_SPACE : NEC_ZERO_SPACE);
    }
    markMicros(NEC_BIT_MARK);                   // stop mark
    carrierOff();
}

void OpenMoteIR::sendNEC(uint8_t address, uint8_t command) {
    // Reverse each byte so it leaves the LED least-significant bit first.
    uint32_t code = ((uint32_t)reverseByte(address)          << 24) |
                    ((uint32_t)reverseByte((uint8_t)~address) << 16) |
                    ((uint32_t)reverseByte(command)           <<  8) |
                    ((uint32_t)reverseByte((uint8_t)~command));
    sendNECRaw(code);
}

void OpenMoteIR::sendNECRepeat() {
    markMicros(NEC_HDR_MARK);
    spaceMicros(NEC_RPT_SPACE);
    markMicros(NEC_BIT_MARK);
    carrierOff();
}

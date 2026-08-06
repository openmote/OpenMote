/*
 * 07 — Audio: hold A to record, press B to play it back
 *
 *   A (hold)   record from the PDM microphone for as long as you hold it
 *   B          play the recording back through the speaker
 *   any button stop playback early
 *
 * The LEDs fill up as a bar graph while recording (four LEDs = buffer full) and
 * run a chase pattern during playback.
 *
 * ---------------------------------------------------------------------------
 * Why the I2S code is here and not in the library
 *
 * OpenMote.h covers buttons, LEDs, rumble, battery and the IMU, but not audio.
 * The speaker and microphone need ESP-IDF's I2S drivers, whose API has changed
 * shape between IDF versions — wrapping them in the library would make it break
 * every time the core moves. So the pin numbers come from OpenMotePins.h and
 * the driver calls live in the sketch, where you can see and adjust them.
 *
 * This uses the IDF 5.x drivers (driver/i2s_pdm.h, driver/i2s_std.h), which is
 * what esp32 core 3.x ships.
 *
 * Hardware:
 *   PDM mic (LMD4030T261-OA1)   CLK GPIO12, DATA GPIO11   -> I2S port 0
 *   Amp     (MAX98357A)         BCLK GPIO6, LRCLK GPIO5, DIN GPIO7 -> I2S port 1
 *   Amp shutdown/mode           GPIO21
 *
 * PDM is only available on I2S port 0 on the ESP32-S3, so the mic takes port 0
 * and the speaker gets port 1.
 *
 * Board settings: the standard OpenMote set. PSRAM must be OPI — the recording
 * buffer is 320 KB and lives there.
 * ---------------------------------------------------------------------------
 */
#include <OpenMote.h>

#include <driver/i2s_pdm.h>
#include <driver/i2s_std.h>

OpenMote mote;

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------
#define SAMPLE_RATE    16000     // plenty for voice, and keeps the buffer small
#define MAX_SECONDS    10
#define MAX_SAMPLES    (SAMPLE_RATE * MAX_SECONDS)
#define CHUNK_SAMPLES  512

// The PDM mic is quiet. Raise this if playback is faint, lower it if it
// distorts — the maths below clips rather than wrapping, so overdoing it
// sounds crunchy instead of exploding into noise.
#define PLAYBACK_GAIN  6

// ---------------------------------------------------------------------------

static i2s_chan_handle_t rxChan = nullptr;   // mic,     I2S port 0
static i2s_chan_handle_t txChan = nullptr;   // speaker, I2S port 1

static int16_t *recBuf   = nullptr;
static size_t   recLen   = 0;                // samples currently stored
static size_t   capacity = 0;                // samples the buffer can hold

static int16_t  chunk[CHUNK_SAMPLES];

// ---------------------------------------------------------------------------
// Audio setup
// ---------------------------------------------------------------------------
static void micBegin() {
    i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_new_channel(&chanCfg, nullptr, &rxChan);

    i2s_pdm_rx_config_t cfg = {
        .clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                   I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = (gpio_num_t)OM_PIN_MIC_CLK,
            .din = (gpio_num_t)OM_PIN_MIC_DATA,
            .invert_flags = { .clk_inv = false },
        },
    };
    // The mic's L/R select is strapped left on the OpenMote PCB.
    cfg.slot_cfg.slot_mask = I2S_PDM_SLOT_LEFT;

    i2s_channel_init_pdm_rx_mode(rxChan, &cfg);
}

static void speakerBegin() {
    i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    i2s_new_channel(&chanCfg, &txChan, nullptr);

    i2s_std_config_t cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)OM_PIN_I2S_BCLK,
            .ws   = (gpio_num_t)OM_PIN_I2S_LRCLK,
            .dout = (gpio_num_t)OM_PIN_I2S_DIN,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };

    i2s_channel_init_std_mode(txChan, &cfg);
}

// SD on the MAX98357A is an ANALOG mode pin, not a digital enable, and GPIO21
// reaches it through a 680 kOhm resistor (R17). That pulls very weakly, so the
// node needs a moment to settle — raise it, wait, then start streaming.
// See docs/PINOUT.md in the beta kit.
static void ampEnable(bool on) {
    digitalWrite(OM_PIN_AMP_SD, on ? HIGH : LOW);
    delay(50);
}

// ---------------------------------------------------------------------------
// Record
// ---------------------------------------------------------------------------
static void record() {
    Serial.println("recording... (release A to stop)");
    recLen = 0;

    i2s_channel_enable(rxChan);

    // Throw away the first chunk: the PDM filter needs a moment to settle and
    // the opening samples are usually a thump.
    size_t got = 0;
    i2s_channel_read(rxChan, chunk, sizeof(chunk), &got, 100 / portTICK_PERIOD_MS);

    // One-pole DC blocker. PDM mics carry a DC offset that makes the amp click
    // on every play, and eats headroom that should be going to the audio.
    int32_t prevIn = 0, prevOut = 0;

    while (mote.buttonDown(BTN_A) && recLen < capacity) {
        mote.update();

        if (i2s_channel_read(rxChan, chunk, sizeof(chunk), &got,
                             100 / portTICK_PERIOD_MS) != ESP_OK) {
            continue;
        }

        size_t n = got / sizeof(int16_t);
        if (n > capacity - recLen) n = capacity - recLen;

        for (size_t i = 0; i < n; i++) {
            int32_t x = chunk[i];
            int32_t y = x - prevIn + ((prevOut * 245) >> 8);   // ~0.957 pole
            prevIn  = x;
            prevOut = y;
            recBuf[recLen + i] = (int16_t)constrain(y, -32768, 32767);
        }
        recLen += n;

        mote.ledBar((uint8_t)((recLen * 4) / capacity + 1));
    }

    i2s_channel_disable(rxChan);
    mote.allLEDsOff();

    Serial.printf("recorded %.2f s (%u samples)%s\n",
                  recLen / (float)SAMPLE_RATE, (unsigned)recLen,
                  recLen >= capacity ? "  [buffer full]" : "");
}

// ---------------------------------------------------------------------------
// Play
// ---------------------------------------------------------------------------

// A *new* press on any button, as opposed to one that is merely still held.
// Playback starts from B's press edge, so B is still down when the loop below
// begins — checking buttonDown() here would stop playback instantly.
static bool anyButtonPressedEdge() {
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        if (mote.buttonPressed((OpenMoteButton)i)) return true;
    }
    return false;
}

static void play() {
    if (recLen == 0) {
        Serial.println("nothing recorded yet — hold A first");
        mote.setLED(4, true); delay(60); mote.setLED(4, false);
        return;
    }

    Serial.printf("playing %.2f s\n", recLen / (float)SAMPLE_RATE);

    ampEnable(true);
    i2s_channel_enable(txChan);

    size_t pos = 0;
    while (pos < recLen) {
        size_t n = recLen - pos;
        if (n > CHUNK_SAMPLES) n = CHUNK_SAMPLES;

        for (size_t i = 0; i < n; i++) {
            int32_t v = (int32_t)recBuf[pos + i] * PLAYBACK_GAIN;
            chunk[i] = (int16_t)constrain(v, -32768, 32767);
        }

        size_t written = 0;
        i2s_channel_write(txChan, chunk, n * sizeof(int16_t), &written,
                          200 / portTICK_PERIOD_MS);

        if (written == 0) {          // timed out with nothing consumed
            Serial.println("i2s write stalled — stopping");
            break;
        }

        // Advance by what the driver actually took. On a short write the tail
        // of this chunk is simply re-converted on the next pass, rather than
        // being skipped.
        pos += written / sizeof(int16_t);

        mote.setLEDs((uint8_t)(1 << ((pos / CHUNK_SAMPLES) % 4)));

        // Stop early on a new button press. Not buttonDown(): B is still held
        // from the press that started playback.
        mote.update();
        if (anyButtonPressedEdge()) {
            Serial.println("playback stopped");
            break;
        }
    }

    i2s_channel_disable(txChan);
    ampEnable(false);                 // amp off when idle, or it hisses
    mote.allLEDsOff();
}

// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(600);

    mote.begin(false);                // no IMU needed here

    pinMode(OM_PIN_AMP_SD, OUTPUT);
    ampEnable(false);

    // 10 s of 16-bit mono at 16 kHz is 320 KB — too big for internal RAM, so it
    // goes in PSRAM. If this returns null, PSRAM is set to Disabled or QSPI
    // instead of OPI; see the board settings notes.
    capacity = MAX_SAMPLES;
    recBuf   = (int16_t *)ps_malloc(capacity * sizeof(int16_t));

    if (!recBuf) {
        Serial.println("PSRAM allocation failed — check PSRAM is set to OPI.");
        Serial.println("Falling back to 1 s in internal RAM.");
        capacity = SAMPLE_RATE;
        recBuf   = (int16_t *)malloc(capacity * sizeof(int16_t));
    }
    if (!recBuf) {
        Serial.println("No memory for a recording buffer at all. Halting.");
        while (true) { mote.setLEDs(0b1111); delay(200); mote.allLEDsOff(); delay(200); }
    }

    micBegin();
    speakerBegin();

    Serial.println("\nOpenMote — audio record and playback");
    Serial.printf("Hold A to record (up to %.0f s), press B to play it back.\n",
                  capacity / (float)SAMPLE_RATE);
}

void loop() {
    mote.update();

    if (mote.buttonPressed(BTN_A)) record();
    if (mote.buttonPressed(BTN_B)) play();

    delay(5);
}

# OpenMote

Arduino library for the **OpenMote Gen 3.3** handheld controller — an ESP32-S3
board with twelve buttons, four player LEDs, a rumble motor, a 6-axis IMU,
infrared in and out, and a LiPo battery.

This repository is the library on its own. Documentation, demo firmware and
board definitions live in
[openmote-beta-kit](https://github.com/Gangwa-Labs/openmote-beta-kit).

---

## Install

**Arduino IDE** — **Code → Download ZIP**, then
**Sketch → Include Library → Add .ZIP Library…** and select the downloaded file.

You also need the ESP32 core: add
`https://espressif.github.io/arduino-esp32/package_esp32_index.json` under
*Preferences → Additional boards manager URLs*, then install **esp32 by
Espressif Systems** from Boards Manager.

**PlatformIO**

```ini
lib_deps = https://github.com/Gangwa-Labs/OpenMote.git
```

## Board settings

**Tools → Board → ESP32S3 Dev Module**, then:

```
PSRAM:            OPI PSRAM          <-- must be OPI, the N16R8 has Octal PSRAM
Flash Size:       16MB (128Mb)
Flash Mode:       QIO 80MHz          <-- NOT "OPI 80MHz", the board will not boot
USB CDC On Boot:  Enabled            <-- or you get no serial output at all
USB Mode:         Hardware CDC and JTAG
Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)
```

The full table, and why each setting matters, is in
[ARDUINO_IDE_SETUP.md](https://github.com/Gangwa-Labs/openmote-beta-kit/blob/main/docs/ARDUINO_IDE_SETUP.md).

## Hello world

```cpp
#include <OpenMote.h>

OpenMote mote;

void setup() {
    Serial.begin(115200);
    mote.begin();
}

void loop() {
    mote.update();                          // required, every loop

    if (mote.buttonPressed(BTN_A)) {
        mote.setLED(1, true);
        mote.rumble(200);                   // buzz for 200 ms
    }
    if (mote.buttonReleased(BTN_A)) {
        mote.setLED(1, false);
    }

    IMUData imu = mote.readIMU();
    if (imu.valid) {
        Serial.printf("%.2f %.2f %.2f g\n", imu.ax, imu.ay, imu.az);
    }
}
```

## What it covers

| | |
|---|---|
| **Buttons** | All 12, debounced, with press/release edges and hold timing |
| **LEDs** | The 4 player LEDs, individually or as a bar graph |
| **Rumble** | PWM strength control and non-blocking timed buzzes |
| **Battery** | Voltage and percentage, with the ADC2/Wi-Fi conflict handled |
| **IMU** | LSM6DS3TR-C accelerometer + gyroscope, pitch/roll, temperature |
| **Infrared** | NEC send and receive, plus raw timing capture and replay |
| **I2C** | Bus scanner and register access for the J15 expansion port |

**Audio is not included.** The speaker and microphone need ESP-IDF I2S drivers
whose API shifts between IDF versions, and wrapping them would make this library
fragile. All the pin definitions are here, and the
[demo firmware](https://github.com/Gangwa-Labs/openmote-beta-kit/tree/main/demos)
has working implementations to copy.

## Examples

**File → Examples → OpenMote**

| Sketch | Shows |
|---|---|
| `01_HelloOpenMote` | Board bring-up and serial output |
| `02_ButtonsAndLEDs` | Button edges, hold detection, LED control |
| `03_RumbleAndBattery` | PWM rumble and battery monitoring |
| `04_Motion` | IMU sampling, pitch and roll |
| `05_InfraredRemote` | Capture an NEC code and replay it |
| `06_BoardSelfTest` | Exercises every peripheral in turn |
| `demos/IRLearn` | Full learning universal remote, codes stored in NVS |

## Links

- [openmote-beta-kit](https://github.com/Gangwa-Labs/openmote-beta-kit) — pinout, component list, troubleshooting, demo firmware
- [API reference](https://github.com/Gangwa-Labs/openmote-beta-kit/blob/main/docs/API.md)
- [Pin map](https://github.com/Gangwa-Labs/openmote-beta-kit/blob/main/docs/PINOUT.md)

## License

MIT — see [LICENSE](LICENSE).

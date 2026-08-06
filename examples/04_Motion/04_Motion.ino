/*
 * 04 — Motion
 *
 * Streams the accelerometer and gyroscope, plus derived pitch, roll and
 * die temperature.
 *
 *   A     cycle what gets printed
 *   B     zero the gyroscope bias (hold the board still first)
 *
 * A gyroscope always has a small constant offset, so integrating it drifts.
 * Sampling that offset while the board is stationary and subtracting it is the
 * cheapest useful fix — that's what B does here.
 */
#include <OpenMote.h>

OpenMote mote;

enum Mode { MODE_ACCEL, MODE_GYRO, MODE_ANGLES, MODE_COUNT };
Mode mode = MODE_ACCEL;

const char *modeNames[] = {"accelerometer", "gyroscope", "angles"};

// Gyro bias, in degrees/second, subtracted from every reading.
float biasX = 0, biasY = 0, biasZ = 0;

void calibrateGyro() {
    Serial.println("\nHold the OpenMote completely still...");
    delay(1000);

    const uint16_t samples = 300;
    float sx = 0, sy = 0, sz = 0;
    uint16_t got = 0;

    for (uint16_t i = 0; i < samples; i++) {
        IMUData d = mote.readIMU();
        if (d.valid) { sx += d.gx; sy += d.gy; sz += d.gz; got++; }
        mote.ledBar(1 + (i / (samples / 4)) % 4);
        delay(5);
    }
    mote.allLEDsOff();

    if (got == 0) { Serial.println("IMU read failed."); return; }

    biasX = sx / got;
    biasY = sy / got;
    biasZ = sz / got;
    Serial.printf("Gyro bias: %+.2f %+.2f %+.2f deg/s\n", biasX, biasY, biasZ);
    mote.rumble(150);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    mote.begin();

    if (!mote.imuAvailable()) {
        Serial.println("IMU missing - check the I2C bus with mote.i2cScan().");
    }

    // Defaults are +/-2 g and 1000 deg/s. Widen the accel range if you are
    // logging impacts, or narrow the gyro range for finer slow rotation.
    mote.setAccelRange(ACCEL_4G);
    mote.setGyroRange(GYRO_1000DPS);

    Serial.println("A = change display, B = zero the gyro.");
}

void loop() {
    mote.update();

    if (mote.buttonPressed(BTN_A)) {
        mode = (Mode)((mode + 1) % MODE_COUNT);
        Serial.printf("\n--- %s ---\n", modeNames[mode]);
    }
    if (mote.buttonPressed(BTN_B)) calibrateGyro();

    static uint32_t last = 0;
    if (millis() - last >= 100) {
        last = millis();

        switch (mode) {
            case MODE_ACCEL: {
                IMUData d = mote.readIMU();
                if (d.valid) Serial.printf("accel  %+6.2f %+6.2f %+6.2f g\n",
                                           d.ax, d.ay, d.az);
                break;
            }
            case MODE_GYRO: {
                IMUData d = mote.readIMU();
                if (d.valid) Serial.printf("gyro   %+7.1f %+7.1f %+7.1f deg/s\n",
                                           d.gx - biasX, d.gy - biasY, d.gz - biasZ);
                break;
            }
            case MODE_ANGLES: {
                Serial.printf("pitch %+6.1f  roll %+6.1f  %.1f C\n",
                              mote.pitch(), mote.roll(), mote.readTemperature());
                break;
            }
            default: break;
        }
    }
}

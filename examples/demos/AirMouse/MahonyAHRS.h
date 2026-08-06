#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

#include <math.h>

// ============================================================
// Mahony 9-DOF AHRS  (accel + gyro + magnetometer)
// Adapted from Robert Mahony / Sebastian Madgwick's public-domain
// reference implementation (x-io.co.uk), with dt passed per update so
// the loop rate can jitter without biasing the filter.
//
// Why Mahony here: the proportional/integral feedback corrects gyro
// integration drift continuously - gravity pins pitch/roll, the magnetic
// vector pins yaw/heading - so the absolute aim point does not slowly
// wander the way a raw gyro integration would. The integral term also
// estimates and removes residual gyro bias (the dominant slow drift).
// ============================================================
class MahonyAHRS {
public:
    // Kp = how hard accel/mag pull the estimate toward truth (responsiveness
    // vs. noise rejection). Ki = gyro-bias learning rate. These defaults are a
    // good starting point for a handheld pointer; expose them for tuning.
    MahonyAHRS(float Kp = 1.0f, float Ki = 0.05f)
        : twoKp(2.0f * Kp), twoKi(2.0f * Ki),
          q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
          ibx(0.0f), iby(0.0f), ibz(0.0f) {}

    void setGains(float Kp, float Ki) { twoKp = 2.0f * Kp; twoKi = 2.0f * Ki; }

    // Gyro in rad/s, accel in any consistent unit (normalized internally),
    // mag in any consistent unit (normalized internally). dt in seconds.
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz, float dt) {
        // Fall back to 6-DOF (IMU-only) if the magnetometer reading is invalid,
        // so a flaky mag sample never injects a bogus heading correction.
        if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
            updateIMU(gx, gy, gz, ax, ay, az, dt);
            return;
        }

        float recipNorm;
        float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
        float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
        float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

        // Only run the correction if accel is usable (avoid NaN on free-fall).
        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

            // Reference direction of Earth's magnetic field
            float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            float hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            float bx = sqrtf(hx * hx + hy * hy);
            float bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

            // Estimated direction of gravity and magnetic field
            float halfvx = q1q3 - q0q2;
            float halfvy = q0q1 + q2q3;
            float halfvz = q0q0 - 0.5f + q3q3;
            float halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            float halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            float halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

            // Error is sum of cross product between estimated and measured fields
            float halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            float halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            float halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

            applyFeedback(gx, gy, gz, halfex, halfey, halfez, dt);
            return;
        }
        integrate(gx, gy, gz, dt);
    }

    // 6-DOF update (gravity only) - corrects pitch/roll, lets yaw free-run.
    void updateIMU(float gx, float gy, float gz,
                   float ax, float ay, float az, float dt) {
        float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2;
        float q1q1 = q1 * q1, q1q3 = q1 * q3;
        float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;
        (void)q0q0; (void)q1q1; (void)q2q2; (void)q3q3;

        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

            float halfvx = q1q3 - q0q2;
            float halfvy = q0q1 + q2q3;
            float halfvz = q0 * q0 - 0.5f + q3 * q3;

            float halfex = ay * halfvz - az * halfvy;
            float halfey = az * halfvx - ax * halfvz;
            float halfez = ax * halfvy - ay * halfvx;

            applyFeedback(gx, gy, gz, halfex, halfey, halfez, dt);
            return;
        }
        integrate(gx, gy, gz, dt);
    }

    // Euler angles (radians). yaw = heading (Z), pitch (Y), roll (X).
    float yaw()   const { return atan2f(2.0f * (q0 * q3 + q1 * q2),
                                        1.0f - 2.0f * (q2 * q2 + q3 * q3)); }
    float pitch() const { float t = 2.0f * (q0 * q2 - q3 * q1);
                          t = t > 1.0f ? 1.0f : (t < -1.0f ? -1.0f : t);
                          return asinf(t); }
    float roll()  const { return atan2f(2.0f * (q0 * q1 + q2 * q3),
                                        1.0f - 2.0f * (q1 * q1 + q2 * q2)); }

    // Rotate a body-frame (sensor-frame) vector into the world frame the filter
    // tracks (Z = up via gravity, heading via the magnetic vector). Used to map the
    // board's aim axis to a world pointing direction for the absolute cursor.
    void bodyToWorld(float vx, float vy, float vz,
                     float &ox, float &oy, float &oz) const {
        float w = q0, x = q1, y = q2, z = q3;
        ox = (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - w * z) * vy + 2 * (x * z + w * y) * vz;
        oy = 2 * (x * y + w * z) * vx + (1 - 2 * (x * x + z * z)) * vy + 2 * (y * z - w * x) * vz;
        oz = 2 * (x * z - w * y) * vx + 2 * (y * z + w * x) * vy + (1 - 2 * (x * x + y * y)) * vz;
    }

    void reset() { q0 = 1; q1 = q2 = q3 = 0; ibx = iby = ibz = 0; }

private:
    void applyFeedback(float gx, float gy, float gz,
                       float halfex, float halfey, float halfez, float dt) {
        if (twoKi > 0.0f) {
            ibx += twoKi * halfex * dt;
            iby += twoKi * halfey * dt;
            ibz += twoKi * halfez * dt;
            gx += ibx; gy += iby; gz += ibz;
        }
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
        integrate(gx, gy, gz, dt);
    }

    void integrate(float gx, float gy, float gz, float dt) {
        gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
        float qa = q0, qb = q1, qc = q2;
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);
        float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
    }

    static float invSqrt(float x) { return 1.0f / sqrtf(x); }

    float twoKp, twoKi;
    float q0, q1, q2, q3;       // orientation quaternion
    float ibx, iby, ibz;        // integral feedback (gyro-bias estimate)
};

#endif // MAHONY_AHRS_H

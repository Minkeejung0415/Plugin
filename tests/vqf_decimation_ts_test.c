/*
 * Regression test: VQF must integrate with each sensor's effective
 * (decimated) sample rate, not the hardware tick rate.
 *
 * -------------------------------------------------------------------------
 * THE BUG THIS TEST GUARDS AGAINST
 * -------------------------------------------------------------------------
 * The Red Pitaya FPGA ticks at g_stream_hw_hz (e.g. 1000 Hz).  Some IMU
 * sensors are physically read at a lower rate (e.g. 100 Hz) because they
 * cannot sample faster.  The firmware calls fusion_update_sensor() only on
 * the ticks when a real sensor reading is available.
 *
 * VQF integrates the gyro as:  angle_change = gyro_reading × Ts
 * where Ts is the time between consecutive calls.
 *
 * If Ts is set to the tick period (1/1000 s) but calls actually come at
 * 100 Hz (every 10th tick = every 0.01 s), each call integrates as if only
 * 1 ms passed when in reality 10 ms passed.  Result: the orientation spins
 * 10× too slowly — the "under-rotation" bug.
 *
 * FIX: set FusionSensorConfig.imu_sample_rate_hz = 100 for that sensor.
 * initialize_sensor_filter() then sets VQF's Ts = 1/100 = 0.01 s, which
 * is the correct per-call time step.
 *
 * -------------------------------------------------------------------------
 * WHAT THE TEST DOES
 * -------------------------------------------------------------------------
 * Simulates a sensor spinning at exactly 90 deg/s about the Z axis.
 *
 * Test A (decimated sensor):
 *   imu_sample_rate_hz = 10 Hz, 10 updates.
 *   10 updates × (1/10 s each) = 1 real second.
 *   Expected yaw ≈ 90 deg/s × 1 s = 90°.
 *
 * Test B (full-rate sensor):
 *   imu_sample_rate_hz = 0 (use module rate = 100 Hz), 100 updates.
 *   100 updates × (1/100 s each) = 1 real second.
 *   Expected yaw ≈ 90°.
 *
 * Both must come out near 90° to confirm the decimation fix works.
 * -------------------------------------------------------------------------
 *
 * Build and run (from repo root):
 *   cc -std=c99 -Wall -Wextra -I. -o /tmp/vqf_decim_test tests/vqf_decimation_ts_test.c sensor_fusion.c vqf.c -lm && /tmp/vqf_decim_test
 */

#include <math.h>
#include <stdio.h>

#include "sensor_fusion.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Simulate a sensor spinning at 90 deg/s about the Z axis for n_updates steps.
 * Returns the fused yaw in degrees so the test can check it against 90°.
 *
 * imu_sample_rate_hz: per-sensor effective rate to give VQF (0 = module rate).
 * n_updates:          how many times to call fusion_update_sensor.
 *
 * Raw values chosen to produce exactly 90 deg/s spin:
 *   acc = {0, 0, 16384}  →  0.0, 0.0, 9.807 m/s²  (gravity pointing +Z)
 *   gyr = {0, 0, 11790}  →  0.0, 0.0, 90.0 °/s    (11790 × 1/131 = 90.0)
 */
static double yaw_after_updates(float imu_sample_rate_hz, int n_updates)
{
    FusionSensorConfig cfg;
    fusion_get_default_sensor_config(FUSION_SENSOR_TYPE_GENERIC, false, &cfg);
    cfg.has_mag = false;
    cfg.imu_sample_rate_hz = imu_sample_rate_hz;  /* <-- the key field being tested */

    fusion_init(1, 100.0f);        /* module rate = 100 Hz (irrelevant when imu_sample_rate_hz set) */
    fusion_register_sensor_ex(0, &cfg);
    fusion_set_enabled(true);

    int16_t acc[3] = { 0, 0, 16384 };  /* 1 g along z at default ±2g scale (16384 LSB/g) */
    int16_t gyr[3] = { 0, 0, 11790 };  /* 90 deg/s at default 131 LSB/(deg/s): 131×90 = 11790 */
    int16_t q15[4];

    for (int i = 0; i < n_updates; i++) {
        fusion_update_sensor(0, acc, gyr, NULL, false, q15);
    }

    float q[4];
    fusion_get_quaternion_float(0, q);
    fusion_shutdown();

    /* Extract yaw from the quaternion.
     * For a pure Z-axis rotation: yaw = 2 × atan2(qz, qw).
     * q[0] = w, q[3] = z in our {w,x,y,z} ordering.                       */
    return 2.0 * atan2((double)q[3], (double)q[0]) * 180.0 / M_PI;
}

int main(void)
{
    int failures = 0;

    /* Sensor decimated 10:1 from a 100 Hz module rate: 10 updates span 1 s
       of real time, so a 90 deg/s rotation must fuse to ~90 deg of yaw. */
    double yaw_decimated = yaw_after_updates(10.0f, 10);
    if (fabs(yaw_decimated - 90.0) < 5.0) {
        printf("OK: decimated sensor yaw %.1f deg (expected ~90)\n", yaw_decimated);
    } else {
        printf("FAIL: decimated sensor yaw %.1f deg (expected ~90)\n", yaw_decimated);
        failures++;
    }

    /* imu_sample_rate_hz = 0 falls back to the module rate: 100 updates
       at 100 Hz span 1 s and must also fuse to ~90 deg. */
    double yaw_full_rate = yaw_after_updates(0.0f, 100);
    if (fabs(yaw_full_rate - 90.0) < 5.0) {
        printf("OK: full-rate sensor yaw %.1f deg (expected ~90)\n", yaw_full_rate);
    } else {
        printf("FAIL: full-rate sensor yaw %.1f deg (expected ~90)\n", yaw_full_rate);
        failures++;
    }

    if (failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    return 1;
}

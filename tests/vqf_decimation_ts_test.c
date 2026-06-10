/*
 * Regression test: VQF must integrate with each sensor's effective
 * (decimated) sample rate, not the hardware tick rate.
 *
 * run_stream() decimates sensors below g_stream_hw_hz via sample/hold,
 * so fusion_update_sensor() only runs every Nth tick. If VQF's Ts is the
 * tick period, gyro integration under-rotates by the decimation factor
 * (FusionSensorConfig.imu_sample_rate_hz fixes this).
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

/* Rotate at ~90 deg/s about z (gravity axis) and return fused yaw in degrees. */
static double yaw_after_updates(float imu_sample_rate_hz, int n_updates)
{
    FusionSensorConfig cfg;
    fusion_get_default_sensor_config(FUSION_SENSOR_TYPE_GENERIC, false, &cfg);
    cfg.has_mag = false;
    cfg.imu_sample_rate_hz = imu_sample_rate_hz;

    fusion_init(1, 100.0f);
    fusion_register_sensor_ex(0, &cfg);
    fusion_set_enabled(true);

    int16_t acc[3] = { 0, 0, 16384 };  /* 1 g along z at default +/-2g scale */
    int16_t gyr[3] = { 0, 0, 11790 };  /* 90 deg/s at default 131 LSB/(deg/s) */
    int16_t q15[4];

    for (int i = 0; i < n_updates; i++) {
        fusion_update_sensor(0, acc, gyr, NULL, false, q15);
    }

    float q[4];
    fusion_get_quaternion_float(0, q);
    fusion_shutdown();

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

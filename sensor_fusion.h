/*
 * multipleSensors_justin
 *
 * Red Pitaya-side IMU fusion helper built around VQF.
 *
 * This module maintains one fusion state per active IMU and converts raw
 * accelerometer, gyroscope, and magnetometer samples into fused orientation
 * quaternions. It does not perform hardware acquisition, socket transport,
 * packet formatting, Open Ephys plugin control, or visualization.
 *
 * Intended usage:
 * - RedPitaya_DAQ.c reads raw IMU samples
 * - RedPitaya_DAQ.c calls this module for fusion
 * - outgoing stream appends qw/qx/qy/qz channels
 */

#ifndef sensor_fusion_h
#define sensor_fusion_h

#include <stdbool.h>
#include <stdint.h>

#include "vqf.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FUSION_QUAT_Q15_SCALE 32767

typedef enum {
    FUSION_STATUS_VALID = 1u << 0,
    FUSION_STATUS_ENABLED = 1u << 1,
    FUSION_STATUS_REST_DETECTED = 1u << 2,
    FUSION_STATUS_MAG_DISTURBANCE = 1u << 3,
    FUSION_STATUS_MAGNETOMETER_USED = 1u << 4
} FusionStatusFlags;

typedef enum {
    FUSION_OK = 0,
    FUSION_ERR_NOT_INITIALIZED = -1,
    FUSION_ERR_INVALID_ARGUMENT = -2,
    FUSION_ERR_INVALID_SENSOR_INDEX = -3,
    FUSION_ERR_SENSOR_NOT_REGISTERED = -4,
    FUSION_ERR_DISABLED = -5,
    FUSION_ERR_NO_MEMORY = -6
} FusionResult;

typedef enum {
    FUSION_SENSOR_TYPE_GENERIC = 0,
    FUSION_SENSOR_TYPE_MPU6050,
    FUSION_SENSOR_TYPE_MPU9250,
    FUSION_SENSOR_TYPE_ICM20948,
    FUSION_SENSOR_TYPE_BNO055,
    FUSION_SENSOR_TYPE_CUSTOM
} FusionSensorType;

typedef struct {
    FusionSensorType sensor_type;
    bool has_mag;
    /* Accelerometer scale in m/s^2 per raw count. */
    float accel_mps2_per_lsb;
    /* Gyroscope scale in rad/s per raw count. */
    float gyro_rads_per_lsb;
    /* Magnetometer scale in physical field units per raw count. */
    float mag_units_per_lsb;
    /* Magnetometer sample rate used to configure VQF. */
    float mag_sample_rate_hz;
} FusionSensorConfig;

/*
 * Initializes the fusion module for a fixed number of sensor slots.
 * sample_rate_hz is the accel/gyro update rate in Hz.
 */
int fusion_init(int sensor_count, float sample_rate_hz);
void fusion_shutdown(void);

void fusion_set_enabled(bool enabled);
bool fusion_is_enabled(void);
void fusion_reset(void);
int fusion_reset_sensor(int sensor_index);

/*
 * Registers a sensor slot using generic default scales. For sensors with
 * device-specific scaling, prefer fusion_register_sensor_ex().
 */
int fusion_register_sensor(int sensor_index, bool has_mag);
int fusion_register_sensor_ex(int sensor_index, const FusionSensorConfig *config);

/*
 * Updates one sensor slot with raw integer samples.
 *
 * Inputs:
 * - raw_acc: accelerometer raw counts
 * - raw_gyr: gyroscope raw counts
 * - raw_mag: magnetometer raw counts, may be NULL for 6D mode
 * - mag_is_fresh: true only when raw_mag contains a new mag sample
 *
 * Output:
 * - quat_q15: quantized quaternion in qw, qx, qy, qz order using
 *   FUSION_QUAT_Q15_SCALE.
 */
int fusion_update_sensor(
    int sensor_index,
    const int16_t raw_acc[3],
    const int16_t raw_gyr[3],
    const int16_t raw_mag[3],
    bool mag_is_fresh,
    int16_t quat_q15[4]
);

int fusion_get_quaternion_q15(int sensor_index, int16_t quat_q15[4]);
int fusion_get_quaternion_float(int sensor_index, float quat_out[4]);
uint32_t fusion_get_status_flags(int sensor_index);

/* Fills a config struct with recommended per-device default scales. */
void fusion_get_default_sensor_config(
    FusionSensorType sensor_type,
    bool has_mag,
    FusionSensorConfig *config_out
);

#ifdef __cplusplus
}
#endif

#endif

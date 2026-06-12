/*
 * multipleSensors_justin
 *
 * Red Pitaya-side IMU fusion helper built around VQF.
 *
 * ============================================================================
 *  WHAT THIS MODULE DOES
 * ============================================================================
 *  This module is the manager layer sitting between the raw IMU hardware and
 *  the VQF math filter.  It:
 *    1. Allocates one VQF filter instance per physical sensor (fusion_init).
 *    2. Stores per-sensor scale factors so raw ADC counts become real units.
 *    3. On every sample: converts int16 counts → m/s² / rad/s → calls VQF.
 *    4. Encodes the resulting float quaternion into Q15 int16 format for the
 *       stream (multiply by 32767, clip, cast to int16).
 *    5. Exposes status flags so callers know if the sensor is valid, at rest,
 *       or seeing magnetic disturbance.
 *
 *  SCOPE: only math + memory.  No I2C, no sockets, no Open Ephys.
 *
 *  TYPICAL CALL SEQUENCE (once per sensor per sample tick):
 *    fusion_init(N_SENSORS, SAMPLE_RATE_HZ);
 *    fusion_register_sensor_ex(i, &config);   // once per sensor at startup
 *    fusion_set_enabled(true);
 *    // --- inside the data loop ---
 *    fusion_update_sensor(i, raw_acc, raw_gyr, raw_mag, mag_fresh, quat_q15);
 * ============================================================================
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

/* Q15 scale: floating-point quaternion components in [-1, +1] are multiplied
 * by 32767 and cast to int16.  To decode: divide raw int16 by 32767.0f.
 * Example: float 0.707 → int16 23169.  float -1.0 → int16 -32767. */
#define FUSION_QUAT_Q15_SCALE 32767

/* Bit-field flags returned by fusion_get_status_flags().
 * Test individual bits with: (flags & FUSION_STATUS_VALID) != 0           */
typedef enum {
    FUSION_STATUS_VALID        = 1u << 0,  /* Quaternion has been computed at least once */
    FUSION_STATUS_ENABLED      = 1u << 1,  /* Module is switched on (fusion_set_enabled) */
    FUSION_STATUS_REST_DETECTED    = 1u << 2,  /* Sensor has been still long enough for precise bias estimation */
    FUSION_STATUS_MAG_DISTURBANCE  = 1u << 3,  /* Magnetometer reading looks wrong (near metal/motor) */
    FUSION_STATUS_MAGNETOMETER_USED = 1u << 4  /* Magnetometer update was applied this sample */
} FusionStatusFlags;

/* Return codes for all fusion_* functions that can fail.
 * Check: if (result != FUSION_OK) { handle error }                        */
typedef enum {
    FUSION_OK = 0,
    FUSION_ERR_NOT_INITIALIZED    = -1,  /* fusion_init() was never called */
    FUSION_ERR_INVALID_ARGUMENT   = -2,  /* NULL pointer or out-of-range value */
    FUSION_ERR_INVALID_SENSOR_INDEX = -3, /* index < 0 or >= sensor_count */
    FUSION_ERR_SENSOR_NOT_REGISTERED = -4, /* slot exists but was never registered */
    FUSION_ERR_DISABLED           = -5,  /* fusion_set_enabled(false) was called */
    FUSION_ERR_NO_MEMORY          = -6   /* calloc failed (embedded OOM) */
} FusionResult;

/* Supported sensor chip models.  Each model has known default scale factors
 * stored in get_sensor_default_config() inside sensor_fusion.c.
 * Use GENERIC or CUSTOM when the chip is unknown or scales are measured.  */
typedef enum {
    FUSION_SENSOR_TYPE_GENERIC  = 0,  /* Unknown chip — generic MPU6050-like defaults */
    FUSION_SENSOR_TYPE_MPU6050,        /* InvenSense MPU-6050: 6-axis, no magnetometer */
    FUSION_SENSOR_TYPE_MPU9250,        /* InvenSense MPU-9250: 9-axis with AK8963 mag */
    FUSION_SENSOR_TYPE_ICM20948,       /* TDK ICM-20948: 9-axis with AK09916 mag */
    FUSION_SENSOR_TYPE_BNO055,         /* Bosch BNO055: absolute orientation sensor */
    FUSION_SENSOR_TYPE_CUSTOM          /* Caller supplies all scale factors manually */
} FusionSensorType;

/* Per-sensor configuration passed to fusion_register_sensor_ex().
 * All scale factors convert raw ADC counts to real physical units.         */
typedef struct {
    FusionSensorType sensor_type;

    /* Does this sensor slot have a working magnetometer?
     * If false, mag_* fields are ignored and VQF runs in 6D mode.         */
    bool has_mag;

    /* Multiply raw int16 accelerometer count by this → m/s².
     * Example for MPU6050 at ±2g: 9.80665 / 16384.0 ≈ 0.000598 m/s²/LSB */
    float accel_mps2_per_lsb;

    /* Multiply raw int16 gyroscope count by this → rad/s.
     * Example for MPU6050 at ±250°/s: π / (180 × 131) ≈ 1.332e-4 rad/s/LSB */
    float gyro_rads_per_lsb;

    /* Multiply raw int16 magnetometer count by this → any consistent unit.
     * VQF only needs relative field strength, so exact µT isn't mandatory. */
    float mag_units_per_lsb;

    /* How fast the magnetometer produces new samples (Hz).  Used to set
     * the VQF mag time step.  Common value: 100 Hz for AK09916.            */
    float mag_sample_rate_hz;

    /* Effective accel/gyro sample rate for THIS sensor (Hz).
     * Set to the actual per-sensor rate when the sensor is decimated below
     * the module's global sample rate (fusion_init second argument).
     * VQF uses 1/imu_sample_rate_hz as its time step, so if you lie here
     * the gyro integration will be wrong by a proportional factor.
     * 0 = use the module's global rate (no decimation).                    */
    float imu_sample_rate_hz;
} FusionSensorConfig;

/* ---- Lifecycle ---- */

/* Call once at program start before any other fusion_* functions.
 * sensor_count: how many physical IMU chips to manage (max from hardware).
 * sample_rate_hz: the rate at which the main data loop runs (Hz).
 *   Individual sensors may run slower; set imu_sample_rate_hz in their
 *   FusionSensorConfig to tell VQF the correct time step.
 * Returns FUSION_OK or FUSION_ERR_NO_MEMORY / FUSION_ERR_INVALID_ARGUMENT. */
int fusion_init(int sensor_count, float sample_rate_hz);

/* Free all memory and reset to blank state.  Safe to call even if not init. */
void fusion_shutdown(void);

/* ---- Global on/off switch ---- */

/* When disabled (false), fusion_update_sensor returns identity quaternions
 * and FUSION_ERR_DISABLED.  Useful during sensor reconfiguration.          */
void fusion_set_enabled(bool enabled);
bool fusion_is_enabled(void);

/* ---- Reset ---- */

/* Reset every active sensor's VQF filter to identity orientation.
 * Use this when the user asks to re-zero all sensors.                       */
void fusion_reset(void);

/* Reset one sensor's filter without disturbing others.                      */
int fusion_reset_sensor(int sensor_index);

/* ---- Registration (call once per sensor after fusion_init) ---- */

/* Quick registration using generic (MPU6050-like) defaults.  OK for testing
 * or when the exact chip model does not matter.                             */
int fusion_register_sensor(int sensor_index, bool has_mag);

/* Full registration with explicit scale factors.  Use this in production.
 * Populate config via fusion_get_default_sensor_config() then override
 * fields as needed (e.g. set imu_sample_rate_hz for a slow sensor).        */
int fusion_register_sensor_ex(int sensor_index, const FusionSensorConfig *config);

/* ---- Per-sample update (hot path, call inside the data loop) ---- */

/* Feed one sensor's raw readings, get back a fresh quaternion.
 *
 * sensor_index : 0 … (sensor_count-1) from fusion_init
 * raw_acc[3]   : raw int16 from the accelerometer ADC  (x, y, z)
 * raw_gyr[3]   : raw int16 from the gyroscope ADC      (x, y, z)
 * raw_mag[3]   : raw int16 from the magnetometer ADC   (x, y, z)
 *                pass NULL if no magnetometer or mag was not updated
 * mag_is_fresh : true ONLY when raw_mag contains a brand-new sample
 *                (magnetometers often run slower than accel/gyro;
 *                 set false for gyro/accel-only frames to avoid
 *                 feeding VQF the same stale mag value twice)
 * quat_q15[4]  : OUTPUT — quaternion in Q15 fixed-point {w, x, y, z}
 *                Decode: float_val = (float)quat_q15[i] / FUSION_QUAT_Q15_SCALE
 *
 * Returns FUSION_OK on success, or an error code (see FusionResult). */
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

#include "sensor_fusion.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FUSION_STANDARD_GRAVITY_MPS2 9.80665f

/*
 * Scaling assumptions inferred from the current RedPitaya_DAQ.c / axi_header.h:
 *
 * - MPU6050 accel/gyro appear to use default full-scale settings:
 *     accel: +/-2g   -> 16384 LSB/g
 *     gyro:  +/-250dps -> 131 LSB/(deg/s)
 *
 * - MPU9250 accel/gyro are not explicitly reconfigured in current init code,
 *   so the same default assumptions are used here:
 *     accel: +/-2g   -> 16384 LSB/g
 *     gyro:  +/-250dps -> 131 LSB/(deg/s)
 *   Magnetometer is configured for 16-bit continuous mode, so 0.15 uT/LSB
 *   is used as the physical conversion.
 *
 * - ICM20948 accel/gyro are likewise not explicitly reconfigured in the
 *   current init path, and axi_header.h defines matching sensitivities:
 *     accel: +/-2g   -> 16384 LSB/g
 *     gyro:  +/-250dps -> 131 LSB/(deg/s)
 *   The AK09916 magnetometer is configured for continuous 100 Hz mode.
 *
 * - BNO055 support follows existing repo constants used elsewhere:
 *     accel: 100 LSB/(m/s^2)
 *     gyro:  16 LSB/(deg/s)
 *     mag:   16 LSB/uT
 *
 * If DAQ later programs different full-scale ranges, these defaults should
 * be overridden via fusion_register_sensor_ex().
 */

/* Internal state for one physical IMU sensor slot. */
typedef struct {
    bool active;               /* true once fusion_register_sensor_ex() was called for this slot */
    bool has_mag;              /* mirrors config.has_mag; used to choose 6D vs 9D VQF output */
    FusionSensorConfig config; /* scale factors + sample rate copied from registration call */
    VQF vqf;                   /* one complete VQF filter instance for this sensor */
    float last_quat[4];        /* most recent float quaternion {w,x,y,z} cached here */
    uint32_t last_status_flags;/* bitmask of FusionStatusFlags updated after each call */
} FusionSensorState;

/* Singleton that owns all sensor slots.  Only one fusion module per program. */
typedef struct {
    bool initialized;          /* true after fusion_init(); guards all other calls */
    bool enabled;              /* global on/off; false → identity quaternions returned */
    int sensor_count;          /* number of slots allocated (argument to fusion_init) */
    float sample_rate_hz;      /* default rate used when imu_sample_rate_hz == 0 */
    FusionSensorState *sensors; /* heap-allocated array of sensor_count slots */
} FusionModuleState;

static FusionModuleState g_fusion;

/* Convert degrees to radians.  Used when setting up VQF scale factors that
 * are specified in degrees (e.g. 1/131 deg/s → rad/s).                     */
static float deg_to_rad(float degrees)
{
    return degrees * ((float)M_PI / 180.0f);
}

/* Simple clamp — keeps a value within [min_value, max_value].              */
static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/* Write an identity quaternion {1, 0, 0, 0} — represents "no rotation".
 * Used to initialise last_quat before any real data arrives.               */
static void set_identity_quaternion(float quat[4])
{
    quat[0] = 1.0f;  /* w = 1: the real component */
    quat[1] = 0.0f;  /* x = 0 */
    quat[2] = 0.0f;  /* y = 0 */
    quat[3] = 0.0f;  /* z = 0: imaginary components are all zero → no rotation */
}

/* Write the identity quaternion in Q15 format.
 * w = 32767 (≈ 1.0), x = y = z = 0.                                       */
static void copy_identity_quaternion_q15(int16_t quat_q15[4])
{
    quat_q15[0] = FUSION_QUAT_Q15_SCALE;  /* w: max positive = +1.0 */
    quat_q15[1] = 0;
    quat_q15[2] = 0;
    quat_q15[3] = 0;
}

/* Convert one float in [-1, +1] to a signed Q15 int16.
 * Steps: scale by 32767, round to nearest integer, clip to valid int16 range.
 * Example: float  0.707 → scaled 23168.8 → rounded 23169 → (int16) 23169
 *          float -1.0   → scaled -32767.0 → rounded -32767 → (int16) -32767 */
static int16_t float_to_q15(float value)
{
    const float scaled  = value * (float)FUSION_QUAT_Q15_SCALE;
    /* Round toward nearest: add +0.5 for positive, -0.5 for negative. */
    const float rounded = scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f;
    /* Clip: int16 holds -32768 … 32767; float_to_q15(-1.0) must not overflow. */
    const float clipped = clampf(rounded, -32768.0f, 32767.0f);
    return (int16_t)clipped;
}

/* Convert all four quaternion components to Q15 at once. */
static void quat_float_to_q15(const float quat[4], int16_t quat_q15[4])
{
    quat_q15[0] = float_to_q15(quat[0]);  /* w */
    quat_q15[1] = float_to_q15(quat[1]);  /* x */
    quat_q15[2] = float_to_q15(quat[2]);  /* y */
    quat_q15[3] = float_to_q15(quat[3]);  /* z */
}

static bool is_valid_sensor_index(int sensor_index)
{
    return g_fusion.initialized &&
           sensor_index >= 0 &&
           sensor_index < g_fusion.sensor_count;
}

/* Set up (or re-set) the VQF filter for one sensor.
 *
 * The critical decision here is the time step (Ts = 1/rate).  VQF uses Ts
 * to integrate the gyroscope:  angle_change = gyro_reading × Ts.
 * If Ts is wrong, the output orientation will rotate at the wrong speed.
 *
 * When the hardware FPGA tick rate is 1000 Hz but a specific sensor is only
 * read every 10th tick (100 Hz), imu_sample_rate_hz should be 100.  Setting
 * gyr_ts = 1/100 = 0.01 s means VQF integrates as if each sample spans 10 ms,
 * which is correct.  Using the module rate (1/1000 = 0.001 s) would cause the
 * angle to grow 10× too fast — the "decimation bug" this guard prevents.    */
static void initialize_sensor_filter(FusionSensorState *sensor)
{
    /* Use the per-sensor rate if specified; otherwise fall back to the module rate. */
    float imu_rate_hz = sensor->config.imu_sample_rate_hz;
    if (imu_rate_hz <= 0.0f) {
        imu_rate_hz = g_fusion.sample_rate_hz;
    }

    const float gyr_ts = 1.0f / imu_rate_hz;  /* seconds per gyro sample */
    const float acc_ts = gyr_ts;               /* accel arrives at the same rate */

    float mag_rate_hz = sensor->config.mag_sample_rate_hz;
    if (mag_rate_hz <= 0.0f) {
        mag_rate_hz = g_fusion.sample_rate_hz;
    }

    /* Initialize VQF with gyro_Ts, accel_Ts, mag_Ts.
     * VQF pre-computes filter coefficients from these at init time.        */
    vqf_init(&sensor->vqf, gyr_ts, acc_ts, 1.0f / mag_rate_hz);

    /* Start at identity orientation (no rotation).  The filter will converge
     * to the true orientation after a few seconds of normal movement.      */
    set_identity_quaternion(sensor->last_quat);
    sensor->last_status_flags = FUSION_STATUS_ENABLED;
}

static void get_sensor_default_config(
    FusionSensorType sensor_type,
    bool has_mag,
    FusionSensorConfig *config_out
)
{
    FusionSensorConfig config;

    config.sensor_type = sensor_type;
    config.has_mag = has_mag;
    config.accel_mps2_per_lsb = FUSION_STANDARD_GRAVITY_MPS2 / 16384.0f;
    config.gyro_rads_per_lsb = deg_to_rad(1.0f / 131.0f);
    config.mag_units_per_lsb = 0.15f;
    config.mag_sample_rate_hz = 100.0f;
    config.imu_sample_rate_hz = 0.0f;

    switch (sensor_type) {
    case FUSION_SENSOR_TYPE_MPU6050:
        config.has_mag = false;
        config.mag_units_per_lsb = 0.0f;
        config.mag_sample_rate_hz = 0.0f;
        break;

    case FUSION_SENSOR_TYPE_MPU9250:
        config.has_mag = has_mag;
        config.accel_mps2_per_lsb = FUSION_STANDARD_GRAVITY_MPS2 / 16384.0f;
        config.gyro_rads_per_lsb = deg_to_rad(1.0f / 131.0f);
        config.mag_units_per_lsb = 0.15f / 16.0f;
        config.mag_sample_rate_hz = has_mag ? 100.0f : 0.0f;
        break;

    case FUSION_SENSOR_TYPE_ICM20948:
        config.has_mag = has_mag;
        config.accel_mps2_per_lsb = FUSION_STANDARD_GRAVITY_MPS2 / 16384.0f;
        config.gyro_rads_per_lsb = deg_to_rad(1.0f / 131.0f);
        config.mag_units_per_lsb = 0.15f;
        config.mag_sample_rate_hz = has_mag ? 100.0f : 0.0f;
        break;

    case FUSION_SENSOR_TYPE_BNO055:
        config.has_mag = has_mag;
        config.accel_mps2_per_lsb = 1.0f / 100.0f;
        config.gyro_rads_per_lsb = deg_to_rad(1.0f / 16.0f);
        config.mag_units_per_lsb = 1.0f / 16.0f;
        config.mag_sample_rate_hz = has_mag ? 100.0f : 0.0f;
        break;

    case FUSION_SENSOR_TYPE_GENERIC:
    case FUSION_SENSOR_TYPE_CUSTOM:
    default:
        break;
    }

    *config_out = config;
}

/* Multiply each raw ADC integer by the per-axis scale factor to get physical
 * units (m/s² for accel, rad/s for gyro, µT for mag).
 * The cast chain (int16→float→scale→vqf_real_t) avoids integer overflow
 * while keeping precision compatible with VQF's internal type.             */
static void convert_vector_to_physical(
    const int16_t raw[3],
    float scale,
    vqf_real_t out[3]
)
{
    out[0] = (vqf_real_t)((float)raw[0] * scale);
    out[1] = (vqf_real_t)((float)raw[1] * scale);
    out[2] = (vqf_real_t)((float)raw[2] * scale);
}

/* Read the latest quaternion out of VQF and store it in last_quat.
 * Also refresh the status flags so callers can check sensor health.
 *
 * Selects 9D (with magnetometer heading) when has_mag is true, otherwise
 * 6D (tilt-corrected, no north reference).                                 */
static void refresh_last_quaternion(FusionSensorState *sensor)
{
    vqf_real_t quat[4];

    /* Choose the best available output mode. */
    if (sensor->has_mag) {
        vqf_get_quat_9d(&sensor->vqf, quat);  /* full orientation, uses compass */
    } else {
        vqf_get_quat_6d(&sensor->vqf, quat);  /* tilt-corrected, no heading */
    }

    /* Downcast from vqf_real_t (double by default) to float for storage.
     * VQF guarantees the output is already a unit quaternion, so no
     * re-normalization is needed before the Q15 encoding step.             */
    sensor->last_quat[0] = (float)quat[0];  /* w */
    sensor->last_quat[1] = (float)quat[1];  /* x */
    sensor->last_quat[2] = (float)quat[2];  /* y */
    sensor->last_quat[3] = (float)quat[3];  /* z */

    /* Start with "valid and enabled", then add any warning bits. */
    sensor->last_status_flags = FUSION_STATUS_VALID | FUSION_STATUS_ENABLED;

    /* Sensor has been still for long enough that bias is well estimated. */
    if (vqf_get_rest_detected(&sensor->vqf)) {
        sensor->last_status_flags |= FUSION_STATUS_REST_DETECTED;
    }

    /* A metal object / motor is distorting the magnetic field.
     * The heading component of the quaternion may be unreliable.           */
    if (vqf_get_mag_dist_detected(&sensor->vqf)) {
        sensor->last_status_flags |= FUSION_STATUS_MAG_DISTURBANCE;
    }
}

int fusion_init(int sensor_count, float sample_rate_hz)
{
    if (sensor_count <= 0 || sample_rate_hz <= 0.0f) {
        return FUSION_ERR_INVALID_ARGUMENT;
    }

    fusion_shutdown();

    g_fusion.sensors = (FusionSensorState *)calloc((size_t)sensor_count, sizeof(FusionSensorState));
    if (g_fusion.sensors == NULL) {
        return FUSION_ERR_NO_MEMORY;
    }

    g_fusion.initialized = true;
    g_fusion.enabled = false;
    g_fusion.sensor_count = sensor_count;
    g_fusion.sample_rate_hz = sample_rate_hz;
    return FUSION_OK;
}

void fusion_shutdown(void)
{
    free(g_fusion.sensors);
    memset(&g_fusion, 0, sizeof(g_fusion));
}

void fusion_set_enabled(bool enabled)
{
    if (!g_fusion.initialized) return;
    g_fusion.enabled = enabled;
}

bool fusion_is_enabled(void)
{
    return g_fusion.initialized && g_fusion.enabled;
}

void fusion_reset(void)
{
    int i;

    if (!g_fusion.initialized) return;

    for (i = 0; i < g_fusion.sensor_count; ++i) {
        FusionSensorState *sensor = &g_fusion.sensors[i];
        if (!sensor->active) continue;
        initialize_sensor_filter(sensor);
    }
}

int fusion_reset_sensor(int sensor_index)
{
    FusionSensorState *sensor;

    if (!g_fusion.initialized) return FUSION_ERR_NOT_INITIALIZED;
    if (!is_valid_sensor_index(sensor_index)) return FUSION_ERR_INVALID_SENSOR_INDEX;

    sensor = &g_fusion.sensors[sensor_index];
    if (!sensor->active) return FUSION_ERR_SENSOR_NOT_REGISTERED;

    initialize_sensor_filter(sensor);
    return FUSION_OK;
}

void fusion_get_default_sensor_config(
    FusionSensorType sensor_type,
    bool has_mag,
    FusionSensorConfig *config_out
)
{
    if (config_out == NULL) return;
    get_sensor_default_config(sensor_type, has_mag, config_out);
}

int fusion_register_sensor(int sensor_index, bool has_mag)
{
    FusionSensorConfig config;

    get_sensor_default_config(FUSION_SENSOR_TYPE_GENERIC, has_mag, &config);
    return fusion_register_sensor_ex(sensor_index, &config);
}

int fusion_register_sensor_ex(int sensor_index, const FusionSensorConfig *config)
{
    FusionSensorState *sensor;

    if (!g_fusion.initialized) return FUSION_ERR_NOT_INITIALIZED;
    if (config == NULL) return FUSION_ERR_INVALID_ARGUMENT;
    if (!is_valid_sensor_index(sensor_index)) return FUSION_ERR_INVALID_SENSOR_INDEX;
    if (config->accel_mps2_per_lsb <= 0.0f || config->gyro_rads_per_lsb <= 0.0f) {
        return FUSION_ERR_INVALID_ARGUMENT;
    }
    if (config->has_mag && config->mag_units_per_lsb <= 0.0f) {
        return FUSION_ERR_INVALID_ARGUMENT;
    }

    sensor = &g_fusion.sensors[sensor_index];
    memset(sensor, 0, sizeof(*sensor));
    sensor->active = true;
    sensor->has_mag = config->has_mag;
    sensor->config = *config;
    initialize_sensor_filter(sensor);
    return FUSION_OK;
}

/* ---- Hot path: called once per sensor per hardware tick ---- */
int fusion_update_sensor(
    int sensor_index,
    const int16_t raw_acc[3],
    const int16_t raw_gyr[3],
    const int16_t raw_mag[3],
    bool mag_is_fresh,
    int16_t quat_q15[4]
)
{
    FusionSensorState *sensor;
    vqf_real_t acc[3];
    vqf_real_t gyr[3];

    /* Guard: refuse to run if the module is not ready. */
    if (!g_fusion.initialized) return FUSION_ERR_NOT_INITIALIZED;
    if (!is_valid_sensor_index(sensor_index)) return FUSION_ERR_INVALID_SENSOR_INDEX;
    if (raw_acc == NULL || raw_gyr == NULL || quat_q15 == NULL) {
        return FUSION_ERR_INVALID_ARGUMENT;
    }

    sensor = &g_fusion.sensors[sensor_index];

    /* Slot exists but was never registered — return identity so the stream
     * still has well-defined data instead of garbage.                       */
    if (!sensor->active) {
        copy_identity_quaternion_q15(quat_q15);
        return FUSION_ERR_SENSOR_NOT_REGISTERED;
    }

    /* Global kill switch (e.g. during reconfiguration).  Caller still gets
     * a valid identity quaternion so downstream consumers don't crash.      */
    if (!g_fusion.enabled) {
        copy_identity_quaternion_q15(quat_q15);
        sensor->last_status_flags = 0;
        return FUSION_ERR_DISABLED;
    }

    /* --- Step 1: Convert raw ADC integers → physical units --- */
    convert_vector_to_physical(raw_acc, sensor->config.accel_mps2_per_lsb, acc);
    convert_vector_to_physical(raw_gyr, sensor->config.gyro_rads_per_lsb,  gyr);

    /* --- Step 2: 6D VQF update (gyro + accel) --- */
    /* vqf_update runs: gyro strapdown integration + accel tilt correction + bias estimation. */
    vqf_update(&sensor->vqf, gyr, acc);

    /* --- Step 3: Optionally apply magnetometer correction (9D) --- */
    /* Only run when:  (a) this sensor has a mag chip, AND
     *                 (b) the mag reading is from a new sample this tick, AND
     *                 (c) the caller provided non-NULL mag data.
     * Condition (b) prevents feeding the same stale mag value twice when the
     * mag runs slower than the accel/gyro (e.g. 100 Hz mag, 500 Hz accel).  */
    if (sensor->has_mag && mag_is_fresh && raw_mag != NULL) {
        vqf_real_t mag[3];
        convert_vector_to_physical(raw_mag, sensor->config.mag_units_per_lsb, mag);
        vqf_update_mag(&sensor->vqf, mag);
    }

    /* --- Step 4: Read back the updated quaternion and status --- */
    refresh_last_quaternion(sensor);

    /* Tag the status if mag was actually used this frame. */
    if (sensor->has_mag && mag_is_fresh && raw_mag != NULL) {
        sensor->last_status_flags |= FUSION_STATUS_MAGNETOMETER_USED;
    }

    /* --- Step 5: Encode float quaternion → Q15 int16 for the TCP stream --- */
    quat_float_to_q15(sensor->last_quat, quat_q15);
    return FUSION_OK;
}

int fusion_get_quaternion_q15(int sensor_index, int16_t quat_q15[4])
{
    if (!g_fusion.initialized) return FUSION_ERR_NOT_INITIALIZED;
    if (!is_valid_sensor_index(sensor_index)) return FUSION_ERR_INVALID_SENSOR_INDEX;
    if (quat_q15 == NULL) return FUSION_ERR_INVALID_ARGUMENT;
    if (!g_fusion.sensors[sensor_index].active) return FUSION_ERR_SENSOR_NOT_REGISTERED;

    quat_float_to_q15(g_fusion.sensors[sensor_index].last_quat, quat_q15);
    return FUSION_OK;
}

int fusion_get_quaternion_float(int sensor_index, float quat_out[4])
{
    if (!g_fusion.initialized) return FUSION_ERR_NOT_INITIALIZED;
    if (!is_valid_sensor_index(sensor_index)) return FUSION_ERR_INVALID_SENSOR_INDEX;
    if (quat_out == NULL) return FUSION_ERR_INVALID_ARGUMENT;
    if (!g_fusion.sensors[sensor_index].active) return FUSION_ERR_SENSOR_NOT_REGISTERED;

    memcpy(quat_out, g_fusion.sensors[sensor_index].last_quat, 4u * sizeof(float));
    return FUSION_OK;
}

uint32_t fusion_get_status_flags(int sensor_index)
{
    if (!is_valid_sensor_index(sensor_index)) return 0u;
    if (!g_fusion.sensors[sensor_index].active) return 0u;
    return g_fusion.sensors[sensor_index].last_status_flags;
}

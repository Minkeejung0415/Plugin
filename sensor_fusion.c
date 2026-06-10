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

typedef struct {
    bool active;
    bool has_mag;
    FusionSensorConfig config;
    VQF vqf;
    float last_quat[4];
    uint32_t last_status_flags;
} FusionSensorState;

typedef struct {
    bool initialized;
    bool enabled;
    int sensor_count;
    float sample_rate_hz;
    FusionSensorState *sensors;
} FusionModuleState;

static FusionModuleState g_fusion;

static float deg_to_rad(float degrees)
{
    return degrees * ((float)M_PI / 180.0f);
}

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void set_identity_quaternion(float quat[4])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}

static void copy_identity_quaternion_q15(int16_t quat_q15[4])
{
    quat_q15[0] = FUSION_QUAT_Q15_SCALE;
    quat_q15[1] = 0;
    quat_q15[2] = 0;
    quat_q15[3] = 0;
}

static int16_t float_to_q15(float value)
{
    const float scaled = value * (float)FUSION_QUAT_Q15_SCALE;
    const float rounded = scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f;
    const float clipped = clampf(rounded, -32768.0f, 32767.0f);
    return (int16_t)clipped;
}

static void quat_float_to_q15(const float quat[4], int16_t quat_q15[4])
{
    quat_q15[0] = float_to_q15(quat[0]);
    quat_q15[1] = float_to_q15(quat[1]);
    quat_q15[2] = float_to_q15(quat[2]);
    quat_q15[3] = float_to_q15(quat[3]);
}

static void normalize_quaternion(float quat[4])
{
    const float norm = sqrtf(quat[0] * quat[0] +
                             quat[1] * quat[1] +
                             quat[2] * quat[2] +
                             quat[3] * quat[3]);

    if (norm <= 0.0f) {
        set_identity_quaternion(quat);
        return;
    }

    quat[0] /= norm;
    quat[1] /= norm;
    quat[2] /= norm;
    quat[3] /= norm;
}

static bool is_valid_sensor_index(int sensor_index)
{
    return g_fusion.initialized &&
           sensor_index >= 0 &&
           sensor_index < g_fusion.sensor_count;
}

static void initialize_sensor_filter(FusionSensorState *sensor)
{
    float imu_rate_hz = sensor->config.imu_sample_rate_hz;
    if (imu_rate_hz <= 0.0f) {
        imu_rate_hz = g_fusion.sample_rate_hz;
    }

    const float gyr_ts = 1.0f / imu_rate_hz;
    const float acc_ts = gyr_ts;
    float mag_rate_hz = sensor->config.mag_sample_rate_hz;

    if (mag_rate_hz <= 0.0f) {
        mag_rate_hz = g_fusion.sample_rate_hz;
    }

    vqf_init(&sensor->vqf, gyr_ts, acc_ts, 1.0f / mag_rate_hz);
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

static void refresh_last_quaternion(FusionSensorState *sensor)
{
    vqf_real_t quat[4];

    if (sensor->has_mag) {
        vqf_get_quat_9d(&sensor->vqf, quat);
    } else {
        vqf_get_quat_6d(&sensor->vqf, quat);
    }

    sensor->last_quat[0] = (float)quat[0];
    sensor->last_quat[1] = (float)quat[1];
    sensor->last_quat[2] = (float)quat[2];
    sensor->last_quat[3] = (float)quat[3];
    normalize_quaternion(sensor->last_quat);

    sensor->last_status_flags = FUSION_STATUS_VALID | FUSION_STATUS_ENABLED;

    if (vqf_get_rest_detected(&sensor->vqf)) {
        sensor->last_status_flags |= FUSION_STATUS_REST_DETECTED;
    }

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

    if (!g_fusion.initialized) return FUSION_ERR_NOT_INITIALIZED;
    if (!is_valid_sensor_index(sensor_index)) return FUSION_ERR_INVALID_SENSOR_INDEX;
    if (raw_acc == NULL || raw_gyr == NULL || quat_q15 == NULL) {
        return FUSION_ERR_INVALID_ARGUMENT;
    }

    sensor = &g_fusion.sensors[sensor_index];
    if (!sensor->active) {
        copy_identity_quaternion_q15(quat_q15);
        return FUSION_ERR_SENSOR_NOT_REGISTERED;
    }

    if (!g_fusion.enabled) {
        copy_identity_quaternion_q15(quat_q15);
        sensor->last_status_flags = 0;
        return FUSION_ERR_DISABLED;
    }

    convert_vector_to_physical(raw_acc, sensor->config.accel_mps2_per_lsb, acc);
    convert_vector_to_physical(raw_gyr, sensor->config.gyro_rads_per_lsb, gyr);
    vqf_update(&sensor->vqf, gyr, acc);

    if (sensor->has_mag && mag_is_fresh && raw_mag != NULL) {
        vqf_real_t mag[3];
        convert_vector_to_physical(raw_mag, sensor->config.mag_units_per_lsb, mag);
        vqf_update_mag(&sensor->vqf, mag);
    }

    refresh_last_quaternion(sensor);
    if (sensor->has_mag && mag_is_fresh && raw_mag != NULL) {
        sensor->last_status_flags |= FUSION_STATUS_MAGNETOMETER_USED;
    }

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

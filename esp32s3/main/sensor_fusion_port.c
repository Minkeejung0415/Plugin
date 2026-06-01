/* Thin wrapper: calls the existing VQF C implementation from the repo root.
   The CMakeLists.txt adds the repo-level vqf.c/sensor_fusion.c as a
   component so we can reuse the exact same filter tuning.              */
#include <string.h>
#include "esp_log.h"
#include "../../../sensor_fusion.h"   /* repo-root sensor_fusion.h */
#include "include/imu_icm20948.h"

static const char *TAG = "fusion_port";

static SensorFusionState s_state;
static bool              s_init_done = false;

void fusion_port_init(uint16_t sample_rate_hz)
{
    sensor_fusion_init(&s_state, sample_rate_hz);
    s_init_done = true;
    ESP_LOGI(TAG, "VQF sensor fusion init at %d Hz", sample_rate_hz);
}

/* Returns quaternion (qw qx qy qz) in Q15 int16 format, same as RedPitaya */
void fusion_port_update(const icm20948_data_t *d,
                        int16_t quat_out[4], float quat_float_out[4])
{
    if (!s_init_done) {
        memset(quat_out, 0, 4 * sizeof(int16_t));
        return;
    }
    sensor_fusion_update(&s_state,
                         d->ax, d->ay, d->az,
                         d->gx, d->gy, d->gz,
                         d->mx, d->my, d->mz);

    float qw, qx, qy, qz;
    sensor_fusion_get_quaternion(&s_state, &qw, &qx, &qy, &qz);

    if (quat_float_out) {
        quat_float_out[0] = qw;
        quat_float_out[1] = qx;
        quat_float_out[2] = qy;
        quat_float_out[3] = qz;
    }

    /* Q15 scaling, same as RedPitaya */
    quat_out[0] = (int16_t)(qw * 32767.0f);
    quat_out[1] = (int16_t)(qx * 32767.0f);
    quat_out[2] = (int16_t)(qy * 32767.0f);
    quat_out[3] = (int16_t)(qz * 32767.0f);
}

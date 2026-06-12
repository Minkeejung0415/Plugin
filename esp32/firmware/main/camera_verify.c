#include "camera_verify.h"

#include "esp_log.h"

static const char *TAG = "camera_verify";

static int16_t s_motion_score;

void camera_verify_init(void)
{
    /*
     * Production: esp32-camera on XIAO Sense (OV3660 DVP).
     * TEVM-AR0234 MIPI requires ESP32-P4 — see docs/camera-feasibility.md
     */
    ESP_LOGI(TAG, "Camera verify stub — enable CONFIG_CAMERA_DVP for Sense board");
    s_motion_score = 0;
}

int16_t camera_verify_motion_score(void)
{
    return s_motion_score;
}

int16_t camera_verify_action_flag(int16_t imu_motion_metric)
{
    const int16_t threshold = 400;
    if (imu_motion_metric > threshold && s_motion_score > threshold) {
        return 1;
    }
    if (imu_motion_metric < 50 && s_motion_score < 50) {
        return 0;
    }
    return -1;
}

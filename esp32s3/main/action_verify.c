/* Action verification: captures a camera frame whenever the IMU detects
   a deliberate motion event (acceleration spike), then saves the JPEG to
   SD card and flags it in the UDP stream so the host can cross-reference
   the image against the quaternion trajectory.                           */
#include <math.h>
#include <stdlib.h>
#include "esp_log.h"
#include "include/config.h"
#include "include/camera_ar0234.h"
#include "include/sdcard_log.h"
#include "include/network_stream.h"

static const char *TAG = "action_verify";

static int   s_hold_count = 0;
static bool  s_triggered  = false;

/* Call once per sample.  Returns true if a verify frame was captured. */
bool action_verify_update(float ax, float ay, float az,
                          uint32_t seq, float timestamp_s)
{
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    /* Remove gravity component (≈9.8 m/s²) — only dynamic acceleration */
    float dyn = fabsf(mag - 9.80665f);

    if (dyn > VERIFY_ACCEL_THRESH_G * 9.80665f) {
        s_hold_count++;
    } else {
        s_hold_count = 0;
        s_triggered  = false;
    }

    /* Fire once per sustained motion event (debounce) */
    if (s_hold_count >= VERIFY_HOLD_SAMPLES && !s_triggered) {
        s_triggered = true;

        if (!camera_frame_ready()) {
            ESP_LOGW(TAG, "Motion event seq=%lu but no camera frame ready",
                     (unsigned long)seq);
            return false;
        }

        uint8_t *jpg_buf = NULL;
        size_t   jpg_len = 0;
        if (camera_capture_jpeg(&jpg_buf, &jpg_len) != ESP_OK) {
            ESP_LOGE(TAG, "Capture failed at seq=%lu", (unsigned long)seq);
            return false;
        }

        ESP_LOGI(TAG, "Action frame: seq=%lu dyn=%.2f m/s² size=%zu bytes",
                 (unsigned long)seq, (double)dyn, jpg_len);

        if (network_stream_recording())
            sdcard_write_jpeg(jpg_buf, jpg_len, seq, timestamp_s);

        free(jpg_buf);
        return true;
    }
    return false;
}

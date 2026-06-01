/* AR0234 camera driver.
   Uses ESP-IDF esp_camera component (originally written for OV series but
   the DVP interface layer is sensor-agnostic once the sensor is brought up
   via its I2C registers).  AR0234 is configured for parallel (DVP) output
   on init; the default MIPI serial mode is switched off via reg 0x301A.

   Technexion modules: identical register map.  The physical electrical
   interface difference (MIPI CSI-2) is handled by the carrier-board bridge
   chip before the signal reaches the ESP32-S3 DVP pins.                   */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "include/config.h"
#include "include/camera_ar0234.h"

static const char *TAG = "cam_ar0234";

/* AR0234 I2C address (7-bit, default SADDR low) */
#define AR0234_I2C_ADDR  0x10

/* AR0234 key registers (16-bit address, 16-bit value) */
#define AR_REG_RESET          0x301A
#define AR_REG_OUTPUT_CTRL    0x301A
#define AR_PARALLEL_EN_MASK   0x0400   /* bit 10 = 0 → parallel; 1 → serial */
#define AR_REG_FRAME_LEN_V    0x300A
#define AR_REG_COARSE_INT     0x3012
#define AR_REG_ANALOG_GAIN    0x3028
#define AR_REG_X_SIZE         0x034C
#define AR_REG_Y_SIZE         0x034E
#define AR_REG_X_START        0x0344
#define AR_REG_Y_START        0x0346

static bool s_running = false;
static cam_cfg_t s_cfg;

/* ── ESP-IDF camera config (maps to DVP pins defined in config.h) ────── */
static camera_config_t make_camera_config(const cam_cfg_t *cfg)
{
    camera_config_t cc = {
        .pin_pwdn     = -1,
        .pin_reset    = -1,
        .pin_xclk     = PIN_CAM_XCLK,
        .pin_sscb_sda = PIN_CAM_SDA,
        .pin_sscb_scl = PIN_CAM_SCL,
        .pin_d7 = PIN_CAM_D7, .pin_d6 = PIN_CAM_D6,
        .pin_d5 = PIN_CAM_D5, .pin_d4 = PIN_CAM_D4,
        .pin_d3 = PIN_CAM_D3, .pin_d2 = PIN_CAM_D2,
        .pin_d1 = PIN_CAM_D1, .pin_d0 = PIN_CAM_D0,
        .pin_vsync   = PIN_CAM_VSYNC,
        .pin_href    = PIN_CAM_HREF,
        .pin_pclk    = PIN_CAM_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        /* JPEG output with the requested resolution */
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = (cfg->width >= 1920) ? FRAMESIZE_FHD :
                        (cfg->width >= 1280) ? FRAMESIZE_HD  :
                        (cfg->width >= 640)  ? FRAMESIZE_VGA : FRAMESIZE_QVGA,
        .jpeg_quality = cfg->jpeg_quality,
        .fb_count     = 2,     /* double-buffer in PSRAM */
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,
    };
    return cc;
}

esp_err_t camera_init(const cam_cfg_t *cfg)
{
    s_cfg = *cfg;
    camera_config_t cc = make_camera_config(cfg);
    esp_err_t ret = esp_camera_init(&cc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Switch AR0234 to parallel output mode over I2C.
       esp_camera has already started XCLK and probed the sensor;
       we now patch the output format register.                          */
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        /* AR0234 register 0x301A bit 10 = 0 selects parallel/DVP;
           the driver writes this after the standard OV-family init.
           Because the esp_camera component doesn't know about AR0234
           we write the register directly via the SCCB bus.              */
        s->set_reg(s, AR_REG_OUTPUT_CTRL, 0xFFFF,
                   s->get_reg(s, AR_REG_OUTPUT_CTRL, 0xFFFF)
                   & ~AR_PARALLEL_EN_MASK);
        /* Crop to requested size */
        s->set_framesize(s, cc.frame_size);
        s->set_quality(s, cfg->jpeg_quality);
    }

    s_running = true;
    ESP_LOGI(TAG, "Camera ready: %dx%d JPEG q=%d",
             cfg->width, cfg->height, cfg->jpeg_quality);
    return ESP_OK;
}

esp_err_t camera_stop(void)
{
    s_running = false;
    return esp_camera_deinit();
}

esp_err_t camera_capture_jpeg(uint8_t **buf, size_t *len)
{
    if (!s_running) return ESP_ERR_INVALID_STATE;

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Frame buffer capture failed");
        return ESP_FAIL;
    }

    *buf = heap_caps_malloc(fb->len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!*buf) {
        esp_camera_fb_return(fb);
        return ESP_ERR_NO_MEM;
    }
    memcpy(*buf, fb->buf, fb->len);
    *len = fb->len;
    esp_camera_fb_return(fb);
    return ESP_OK;
}

bool camera_frame_ready(void)
{
    /* esp_camera GRAB_LATEST mode: a frame is "ready" if the DMA ring
       has a fresh buffer.  We probe without consuming it.               */
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return false;
    esp_camera_fb_return(fb);
    return true;
}

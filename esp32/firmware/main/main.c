#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "camera_verify.h"
#include "dio_input.h"
#include "espnow_sync.h"
#include "icm20948.h"
#include "open_ephys_stream.h"
#include "sd_logger.h"

static const char *TAG = "main";

#define WIFI_SSID CONFIG_ESP_WIFI_SSID
#define WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define TCP_PORT 5000

#ifndef CONFIG_ESP_WIFI_SSID
#define CONFIG_ESP_WIFI_SSID "STEP_ESP32"
#define CONFIG_ESP_WIFI_PASSWORD "changeme"
#endif

#ifndef CONFIG_STEP_NODE_MASTER
#define CONFIG_STEP_NODE_MASTER 1
#endif

#ifndef CONFIG_STEP_ENABLE_ESPNOW
#define CONFIG_STEP_ENABLE_ESPNOW 0
#endif

static void wifi_init_sta(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wcfg = {0};
    strncpy((char *)wcfg.sta.ssid, CONFIG_ESP_WIFI_SSID, sizeof(wcfg.sta.ssid) - 1);
    strncpy((char *)wcfg.sta.password, CONFIG_ESP_WIFI_PASSWORD, sizeof(wcfg.sta.password) - 1);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wcfg);
    esp_wifi_start();
    esp_wifi_connect();
}

static void acquisition_task(void *arg)
{
    (void)arg;
    TickType_t period = pdMS_TO_TICKS(1000 / OE_STREAM_SAMPLE_HZ);
    uint32_t seq = 0;

    while (1) {
        oe_sample_t sample = {0};
        sample.seq = seq++;
        sample.timestamp_us = esp_timer_get_time();

        icm20948_read_scaled(sample.ch);
        dio_input_update();
        sample.ch[6] = dio_input_read_channel();
        sample.ch[7] = camera_verify_motion_score();

        int16_t imu_motion = (int16_t)(abs(sample.ch[0]) + abs(sample.ch[1]));
        int16_t verify = camera_verify_action_flag(imu_motion);
        if (verify >= 0) {
            sample.ch[7] = (int16_t)((sample.ch[7] & 0xFFFE) | (verify & 1));
        }

#if CONFIG_STEP_ENABLE_ESPNOW
        espnow_sync_on_local_frame(sample.seq, sample.timestamp_us);
#endif
        open_ephys_stream_set_sample(&sample);
        sd_logger_append(&sample);

        vTaskDelay(period);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 STEP node starting");
    nvs_flash_init();
    wifi_init_sta();

    open_ephys_stream_init();
    icm20948_init();
    dio_input_init(1);
    camera_verify_init();
    sd_logger_init();
#if CONFIG_STEP_ENABLE_ESPNOW
    espnow_sync_init(CONFIG_STEP_NODE_MASTER != 0);
    ESP_LOGI(TAG, "ESP-NOW enabled (multi-node)");
#else
    ESP_LOGI(TAG, "ESP-NOW disabled — single-node mode");
#endif

    open_ephys_stream_start_server(TCP_PORT);
    xTaskCreate(acquisition_task, "acquire", 4096, NULL, 6, NULL);

    ESP_LOGI(TAG, "Ready — TCP %d, %d Hz, %d ch", TCP_PORT, OE_STREAM_SAMPLE_HZ, OE_STREAM_NUM_CHANNELS);
}

/* ESP32-S3 IMU acquisition node — main entry point.
   Mirrors the Red Pitaya server behaviour:
     • ICM-20948 IMU via SPI
     • TEVM-AR0234 / Technexion camera via DVP
     • ESPNow multi-node sync
     • SD card logging
     • TCP:5000 + UDP:55001 streaming to Open Ephys (same protocol)
     • UDP:5005 quaternion stream to OpenSim bridge                     */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "include/config.h"
#include "include/imu_icm20948.h"
#include "include/camera_ar0234.h"
#include "include/espnow_sync.h"
#include "include/sdcard_log.h"
#include "include/network_stream.h"
#include "sensor_fusion_port.c"   /* single-TU port; include directly */
#include "action_verify.c"
#include "dio.c"

static const char *TAG = "main";

/* ── WiFi credentials — override via NVS provisioning or idf.py menuconfig */
#ifndef CONFIG_WIFI_SSID
#define CONFIG_WIFI_SSID     "your_ssid"
#define CONFIG_WIFI_PASSWORD "your_password"
#endif

/* ── Node identity — stored in NVS; defaults to master (node 0) ─────── */
static uint8_t  s_node_id  = 0;
static node_role_t s_role  = ROLE_MASTER;

/* ── WiFi ──────────────────────────────────────────────────────────── */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected — retrying");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&e->ip_info.ip));
    }
}

static void wifi_init_sta(void)
{
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler, NULL));
    wifi_config_t wc = {
        .sta = {
            .ssid     = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_connect();
}

/* ── ESPNow slave data callback (runs on master) ───────────────────── */
static void on_slave_data(const espnow_data_pkt_t *pkt)
{
    /* Re-pack the slave frame into the same channel layout and push
       it through the network_stream / sdcard pipeline.
       For multi-node experiments: channel block = node_id * CHANNELS_PER_IMU */
    int16_t channels[CHANNELS_PER_IMU + 1];
    memcpy(channels, pkt->raw, sizeof(pkt->raw));
    channels[CHANNELS_PER_IMU] = pkt->dio_state;

    static uint32_t slave_seq[ESPNOW_MAX_NODES] = {0};
    uint32_t seq = slave_seq[pkt->node_id]++;
    float ts = (float)(pkt->timestamp_us) * 1e-6f;

    network_stream_push_sample(channels, CHANNELS_PER_IMU + 1, seq);
    if (network_stream_recording())
        sdcard_write_sample(channels, CHANNELS_PER_IMU + 1, seq, ts);
}

/* ── Main acquisition task ─────────────────────────────────────────── */
static void acquisition_task(void *arg)
{
    uint32_t seq       = 0;
    int64_t  start_us  = esp_timer_get_time();
    uint16_t last_rate = DEFAULT_SAMPLE_RATE_HZ;

    fusion_port_init(last_rate);

    int64_t next_tick_us = esp_timer_get_time();

    while (1) {
        uint16_t rate = network_stream_get_rate();
        if (rate != last_rate) {
            fusion_port_init(rate);
            last_rate = rate;
        }

        int64_t period_us = 1000000LL / rate;
        int64_t now       = esp_timer_get_time();
        int64_t sleep_us  = next_tick_us - now;
        if (sleep_us > 100)
            esp_rom_delay_us((uint32_t)sleep_us);
        next_tick_us += period_us;

        /* ── Read IMU ───────────────────────────────── */
        icm20948_data_t imu;
        icm20948_read(&imu);

        /* ── Sensor fusion ──────────────────────────── */
        int16_t quat_q15[4];
        float   quat_f[4];
        if (network_stream_fusion_enabled())
            fusion_port_update(&imu, quat_q15, quat_f);
        else
            memset(quat_q15, 0, sizeof(quat_q15));

        /* ── DIO ─────────────────────────────────────── */
        uint8_t dio = dio_read_state();

        /* ── Pack channel frame (Q15 int16) ─────────── */
        int16_t ch[CHANNELS_PER_IMU + 1];
        float a_sc = 32767.0f / (2.0f * 9.80665f);   /* ±2g range */
        float g_sc = 32767.0f / (250.0f * 0.01745f); /* ±250°/s range */
        float m_sc = 32767.0f / 4912.0f;              /* ±4912 µT */
        ch[0] = (int16_t)(imu.ax * a_sc);
        ch[1] = (int16_t)(imu.ay * a_sc);
        ch[2] = (int16_t)(imu.az * a_sc);
        ch[3] = (int16_t)(imu.gx * g_sc);
        ch[4] = (int16_t)(imu.gy * g_sc);
        ch[5] = (int16_t)(imu.gz * g_sc);
        ch[6] = (int16_t)(imu.mx * m_sc);
        ch[7] = (int16_t)(imu.my * m_sc);
        ch[8] = (int16_t)(imu.mz * m_sc);
        ch[9]  = quat_q15[0];
        ch[10] = quat_q15[1];
        ch[11] = quat_q15[2];
        ch[12] = quat_q15[3];
        ch[13] = (int16_t)dio;

        /* ── Stream & log ───────────────────────────── */
        float ts = (float)(esp_timer_get_time() - start_us) * 1e-6f;
        network_stream_push_sample(ch, CHANNELS_PER_IMU + 1, seq);
        network_stream_push_quats(quat_f, 1, ts);

        if (network_stream_recording())
            sdcard_write_sample(ch, CHANNELS_PER_IMU + 1, seq, ts);

        /* ── Action verification ─────────────────────── */
        action_verify_update(imu.ax, imu.ay, imu.az, seq, ts);

        /* Flush SD every 1000 samples (≈10 s at 100 Hz) */
        if (seq % 1000 == 0)
            sdcard_sync();

        seq++;
    }
}

/* ── app_main ──────────────────────────────────────────────────────── */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Load node role & ID from NVS */
    nvs_handle_t nvs;
    if (nvs_open("config", NVS_READONLY, &nvs) == ESP_OK) {
        uint8_t role_nv = 0, id_nv = 0;
        nvs_get_u8(nvs, "role",    &role_nv);
        nvs_get_u8(nvs, "node_id", &id_nv);
        s_role    = (node_role_t)role_nv;
        s_node_id = id_nv;
        nvs_close(nvs);
    }
    ESP_LOGI(TAG, "Node %d  role=%s", s_node_id,
             s_role == ROLE_MASTER ? "MASTER" : "SLAVE");

    /* ── Hardware init ────────────────────────────────────────────── */
    icm20948_cfg_t imu_cfg = {
        .accel_range = 0,   /* ±2g */
        .gyro_range  = 0,   /* ±250°/s */
        .odr_hz      = DEFAULT_SAMPLE_RATE_HZ,
    };
    ESP_ERROR_CHECK(icm20948_init(&imu_cfg));
    icm20948_calibrate_gyro();

    ESP_ERROR_CHECK(sdcard_init());
    ESP_ERROR_CHECK(dio_init());

    cam_cfg_t cam_cfg = {
        .width        = VERIFY_FRAME_WIDTH,
        .height       = VERIFY_FRAME_HEIGHT,
        .jpeg_quality = VERIFY_JPEG_QUALITY,
        .continuous   = true,
    };
    if (camera_init(&cam_cfg) != ESP_OK)
        ESP_LOGW(TAG, "Camera unavailable — action verify disabled");

    /* ── WiFi + ESPNow (master needs WiFi STA; slaves only ESPNow) ── */
    if (s_role == ROLE_MASTER) {
        wifi_init_sta();
        /* Allow WiFi to connect before starting TCP server */
        vTaskDelay(pdMS_TO_TICKS(3000));
        network_stream_init();
    } else {
        /* Slaves: start WiFi in STA mode but don't connect — needed for
           ESPNow to work on the same channel as the master's AP/STA.   */
        esp_netif_create_default_wifi_sta();
        wifi_init_config_t wc = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&wc));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
    }

    espnow_sync_init(s_node_id, s_role == ROLE_MASTER);
    if (s_role == ROLE_MASTER)
        espnow_set_data_callback(on_slave_data);

    /* ── Acquisition task (pinned to core 1; WiFi runs on core 0) ── */
    xTaskCreatePinnedToCore(acquisition_task, "acq", 8192, NULL,
                            configMAX_PRIORITIES - 1, NULL, 1);

    ESP_LOGI(TAG, "Startup complete");
}

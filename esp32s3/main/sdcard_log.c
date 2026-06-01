/* SD card logging — binary and CSV files byte-compatible with Red Pitaya. */
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_log.h"
#include "include/config.h"
#include "include/sdcard_log.h"

static const char *TAG = "sdcard";

static sdmmc_card_t *s_card = NULL;
static FILE         *s_bin  = NULL;
static FILE         *s_csv  = NULL;
static FILE         *s_jpg_index = NULL;
static char          s_base[64];
static SemaphoreHandle_t s_mutex;
static uint32_t      s_sample_count = 0;

esp_err_t sdcard_init(void)
{
    s_mutex = xSemaphoreCreateMutex();

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk   = PIN_SD_CLK;
    slot.cmd   = PIN_SD_CMD;
    slot.d0    = PIN_SD_D0;
    slot.d1    = PIN_SD_D1;
    slot.d2    = PIN_SD_D2;
    slot.d3    = PIN_SD_D3;
    slot.width = 4;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_vfs_fat_sdmmc_mount_config_t mc = {
        .format_if_mount_failed = false,
        .max_files              = SD_MAX_FILES,
        .allocation_unit_size   = 64 * 1024,
    };

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot,
                                             &mc, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(ret));
        return ret;
    }
    sdmmc_card_print_info(stdout, s_card);
    ESP_LOGI(TAG, "SD card mounted at %s", SD_MOUNT_POINT);
    return ESP_OK;
}

esp_err_t sdcard_open_session(char *base_path_out, size_t out_len)
{
    /* Session filename: /sdcard/session_<epoch>.bin */
    struct timeval tv; gettimeofday(&tv, NULL);
    snprintf(s_base, sizeof(s_base), SD_MOUNT_POINT "/session_%lld",
             (long long)tv.tv_sec);
    if (base_path_out)
        snprintf(base_path_out, out_len, "%s", s_base);

    char path[80];
    snprintf(path, sizeof(path), "%s.bin", s_base);
    s_bin = fopen(path, "wb");

    snprintf(path, sizeof(path), "%s.csv", s_base);
    s_csv = fopen(path, "w");
    if (s_csv) fprintf(s_csv, "seq,time_s,channels\n");

    snprintf(path, sizeof(path), "%s_frames.txt", s_base);
    s_jpg_index = fopen(path, "w");

    s_sample_count = 0;
    ESP_LOGI(TAG, "Session opened: %s", s_base);
    return (s_bin && s_csv) ? ESP_OK : ESP_FAIL;
}

esp_err_t sdcard_close_session(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_bin) { fclose(s_bin); s_bin = NULL; }
    if (s_csv) { fclose(s_csv); s_csv = NULL; }
    if (s_jpg_index) { fclose(s_jpg_index); s_jpg_index = NULL; }
    ESP_LOGI(TAG, "Session closed: %lu samples", (unsigned long)s_sample_count);
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t sdcard_write_sample(const int16_t *channels, uint16_t nch,
                               uint32_t seq, float timestamp_s)
{
    if (!s_bin && !s_csv) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    if (s_bin)
        fwrite(channels, sizeof(int16_t), nch, s_bin);

    if (s_csv) {
        fprintf(s_csv, "%lu,%.6f", (unsigned long)seq, timestamp_s);
        for (int i = 0; i < nch; i++) fprintf(s_csv, ",%d", channels[i]);
        fputc('\n', s_csv);
    }

    s_sample_count++;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t sdcard_write_jpeg(const uint8_t *buf, size_t len,
                             uint32_t seq, float timestamp_s)
{
    char path[80];
    snprintf(path, sizeof(path), "%s_frame_%06lu.jpg", s_base, (unsigned long)seq);
    FILE *f = fopen(path, "wb");
    if (!f) return ESP_FAIL;
    fwrite(buf, 1, len, f);
    fclose(f);

    if (s_jpg_index)
        fprintf(s_jpg_index, "%lu,%.6f,%s\n",
                (unsigned long)seq, timestamp_s, path);
    return ESP_OK;
}

esp_err_t sdcard_sync(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_bin) fflush(s_bin);
    if (s_csv) fflush(s_csv);
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

#include "sd_logger.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "sd_logger";

#ifndef CONFIG_STEP_ENABLE_SD_LOGGER
#define CONFIG_STEP_ENABLE_SD_LOGGER 1
#endif

#ifndef CONFIG_STEP_SD_LOG_QUEUE_DEPTH
#define CONFIG_STEP_SD_LOG_QUEUE_DEPTH 256
#endif

#ifndef CONFIG_STEP_SD_LOG_FLUSH_INTERVAL_MS
#define CONFIG_STEP_SD_LOG_FLUSH_INTERVAL_MS 1000
#endif

#ifndef CONFIG_STEP_SD_LOG_PATH
#define CONFIG_STEP_SD_LOG_PATH "/sdcard/step_session.bin"
#endif

#define SD_LOG_MAGIC 0x31504453u
#define SD_LOG_VERSION 1u
#define SD_LOG_CHANNEL_LAYOUT_ID 1u
#define SD_LOG_MAX_PATH 128

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t header_size;
    uint16_t record_size;
    uint16_t sample_rate_hz;
    uint16_t channel_count;
    uint16_t channel_layout_id;
    uint8_t sd_logging_enabled;
    uint8_t stream_enabled;
    uint8_t filter_enabled;
    uint8_t reserved;
    int64_t start_timestamp_us;
} sd_session_header_t;

typedef enum {
    SD_QUEUE_SAMPLE,
    SD_QUEUE_START,
    SD_QUEUE_STOP,
} sd_queue_item_type_t;

typedef struct {
    sd_queue_item_type_t type;
    oe_sample_t sample;
    char path[SD_LOG_MAX_PATH];
} sd_queue_item_t;

static FILE *s_fp;
static QueueHandle_t s_queue;
static sd_logger_stats_t s_stats;
static portMUX_TYPE s_stats_mux = portMUX_INITIALIZER_UNLOCKED;

static void stats_update_max_u32(uint32_t *field, uint32_t value)
{
    if (value > *field) {
        *field = value;
    }
}

static void write_stats_snapshot(uint64_t saved_delta, uint64_t error_delta, uint32_t write_us)
{
    taskENTER_CRITICAL(&s_stats_mux);
    s_stats.saved_samples += saved_delta;
    s_stats.sd_write_errors += error_delta;
    stats_update_max_u32(&s_stats.max_sd_write_us, write_us);
    taskEXIT_CRITICAL(&s_stats_mux);
}

static void set_file_open(bool file_open)
{
    taskENTER_CRITICAL(&s_stats_mux);
    s_stats.file_open = file_open;
    taskEXIT_CRITICAL(&s_stats_mux);
}

static void write_session_header(FILE *fp)
{
    sd_session_header_t header = {
        .magic = SD_LOG_MAGIC,
        .version = SD_LOG_VERSION,
        .header_size = sizeof(sd_session_header_t),
        .record_size = sizeof(oe_sample_t),
        .sample_rate_hz = OE_STREAM_SAMPLE_HZ,
        .channel_count = OE_STREAM_NUM_CHANNELS,
        .channel_layout_id = SD_LOG_CHANNEL_LAYOUT_ID,
        .sd_logging_enabled = CONFIG_STEP_ENABLE_SD_LOGGER ? 1 : 0,
        .stream_enabled = 1,
        .filter_enabled = 0,
        .reserved = 0,
        .start_timestamp_us = esp_timer_get_time(),
    };

    int64_t start_us = esp_timer_get_time();
    size_t written = fwrite(&header, sizeof(header), 1, fp);
    int64_t write_us = esp_timer_get_time() - start_us;
    write_stats_snapshot(0, written == 1 ? 0 : 1, (uint32_t)write_us);
}

static void close_log_file(void)
{
    if (!s_fp) {
        set_file_open(false);
        return;
    }

    int64_t start_us = esp_timer_get_time();
    int flush_ok = fflush(s_fp);
    int64_t flush_us = esp_timer_get_time() - start_us;
    write_stats_snapshot(0, flush_ok == 0 ? 0 : 1, (uint32_t)flush_us);
    fclose(s_fp);
    s_fp = NULL;
    set_file_open(false);
    ESP_LOGI(TAG, "SD recording stopped");
}

static void open_log_file(const char *path)
{
    close_log_file();

    s_fp = fopen(path, "wb");
    if (!s_fp) {
        set_file_open(false);
        taskENTER_CRITICAL(&s_stats_mux);
        s_stats.sd_write_errors++;
        taskEXIT_CRITICAL(&s_stats_mux);
        ESP_LOGW(TAG, "SD log open failed at %s; recording disabled", path);
        return;
    }

    set_file_open(true);
    write_session_header(s_fp);
    ESP_LOGI(TAG, "SD recording started path=%s", path);
}

static void sd_writer_task(void *arg)
{
    (void)arg;
    TickType_t last_flush = xTaskGetTickCount();
    const TickType_t flush_interval = pdMS_TO_TICKS(CONFIG_STEP_SD_LOG_FLUSH_INTERVAL_MS);

    while (1) {
        sd_queue_item_t item;
        if (xQueueReceive(s_queue, &item, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (item.type == SD_QUEUE_START) {
                open_log_file(item.path[0] != '\0' ? item.path : CONFIG_STEP_SD_LOG_PATH);
                last_flush = xTaskGetTickCount();
                continue;
            }

            if (item.type == SD_QUEUE_STOP) {
                close_log_file();
                last_flush = xTaskGetTickCount();
                continue;
            }

            if (!s_fp) {
                continue;
            }

            int64_t start_us = esp_timer_get_time();
            size_t written = fwrite(&item.sample, sizeof(item.sample), 1, s_fp);
            int64_t write_us = esp_timer_get_time() - start_us;
            write_stats_snapshot(written == 1 ? 1 : 0, written == 1 ? 0 : 1, (uint32_t)write_us);
        }

        TickType_t now = xTaskGetTickCount();
        if (s_fp && (now - last_flush) >= flush_interval) {
            int64_t start_us = esp_timer_get_time();
            int flush_ok = fflush(s_fp);
            int64_t flush_us = esp_timer_get_time() - start_us;
            write_stats_snapshot(0, flush_ok == 0 ? 0 : 1, (uint32_t)flush_us);
            last_flush = now;
        }
    }
}

void sd_logger_init(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.configured_enabled = CONFIG_STEP_ENABLE_SD_LOGGER != 0;
    s_stats.queue_capacity = CONFIG_STEP_SD_LOG_QUEUE_DEPTH;

    if (!CONFIG_STEP_ENABLE_SD_LOGGER) {
        ESP_LOGI(TAG, "SD logging disabled by config");
        return;
    }

    s_queue = xQueueCreate(CONFIG_STEP_SD_LOG_QUEUE_DEPTH, sizeof(sd_queue_item_t));
    if (!s_queue) {
        ESP_LOGE(TAG, "SD queue allocation failed; logging disabled");
        return;
    }

    xTaskCreate(sd_writer_task, "sd_writer", 4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "SD logger ready path=%s queue_depth=%d flush_ms=%d trigger=plugin RECORD ON/OFF",
             CONFIG_STEP_SD_LOG_PATH,
             CONFIG_STEP_SD_LOG_QUEUE_DEPTH,
             CONFIG_STEP_SD_LOG_FLUSH_INTERVAL_MS);
}

bool sd_logger_start(const char *path)
{
    if (!s_stats.configured_enabled || !s_queue) {
        return false;
    }

    sd_queue_item_t item = {
        .type = SD_QUEUE_START,
    };

    const char *target_path = (path && path[0] != '\0') ? path : CONFIG_STEP_SD_LOG_PATH;
    strncpy(item.path, target_path, sizeof(item.path) - 1);

    BaseType_t ok = xQueueSend(s_queue, &item, pdMS_TO_TICKS(100));
    if (ok == pdTRUE) {
        taskENTER_CRITICAL(&s_stats_mux);
        s_stats.recording_requested = true;
        taskEXIT_CRITICAL(&s_stats_mux);
    }
    return ok == pdTRUE;
}

bool sd_logger_stop(void)
{
    if (!s_stats.configured_enabled || !s_queue) {
        return false;
    }

    sd_queue_item_t item = {
        .type = SD_QUEUE_STOP,
    };

    BaseType_t ok = xQueueSend(s_queue, &item, pdMS_TO_TICKS(100));
    if (ok == pdTRUE) {
        taskENTER_CRITICAL(&s_stats_mux);
        s_stats.recording_requested = false;
        taskEXIT_CRITICAL(&s_stats_mux);
    }
    return ok == pdTRUE;
}

bool sd_logger_append(const oe_sample_t *sample)
{
    if (!s_stats.configured_enabled || !s_stats.recording_requested || !s_stats.file_open || !s_queue) {
        return false;
    }

    sd_queue_item_t item = {
        .type = SD_QUEUE_SAMPLE,
        .sample = *sample,
    };

    int64_t start_us = esp_timer_get_time();
    BaseType_t ok = xQueueSend(s_queue, &item, 0);
    uint32_t enqueue_us = (uint32_t)(esp_timer_get_time() - start_us);
    UBaseType_t depth = uxQueueMessagesWaiting(s_queue);

    taskENTER_CRITICAL(&s_stats_mux);
    stats_update_max_u32(&s_stats.max_sd_enqueue_us, enqueue_us);
    stats_update_max_u32(&s_stats.max_sd_queue_depth, (uint32_t)depth);
    if (ok == pdTRUE) {
        s_stats.enqueued_samples++;
    } else {
        s_stats.sd_queue_drops++;
    }
    taskEXIT_CRITICAL(&s_stats_mux);

    return ok == pdTRUE;
}

void sd_logger_note_acquisition(uint32_t loop_us, uint32_t period_us)
{
    taskENTER_CRITICAL(&s_stats_mux);
    s_stats.generated_samples++;
    stats_update_max_u32(&s_stats.max_acq_loop_us, loop_us);
    if (loop_us > period_us) {
        s_stats.acq_loop_overruns++;
    }
    taskEXIT_CRITICAL(&s_stats_mux);
}

void sd_logger_get_stats(sd_logger_stats_t *out)
{
    if (!out) {
        return;
    }

    taskENTER_CRITICAL(&s_stats_mux);
    *out = s_stats;
    taskEXIT_CRITICAL(&s_stats_mux);
}

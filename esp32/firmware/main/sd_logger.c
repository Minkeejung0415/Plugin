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
#define REC_RECONNECT_GRACE_MS 90000u

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
    char reason[32];
} sd_queue_item_t;

static FILE *s_fp;
static QueueHandle_t s_queue;
static sd_logger_stats_t s_stats;
static portMUX_TYPE s_stats_mux = portMUX_INITIALIZER_UNLOCKED;
static char s_active_path[SD_LOG_MAX_PATH];
static int64_t s_grace_deadline_us = 0;

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

static uint32_t checksum_file(const char *path, uint64_t *size_out)
{
    uint8_t buf[256];
    uint32_t crc = 0;
    uint64_t total = 0;
    FILE *fp = fopen(path, "rb");
    if (!fp) {
        if (size_out) {
            *size_out = 0;
        }
        return 0;
    }

    while (1) {
        size_t n = fread(buf, 1, sizeof(buf), fp);
        if (n > 0) {
            crc = crc32_update(crc, buf, n);
            total += n;
        }
        if (n < sizeof(buf)) {
            break;
        }
    }
    fclose(fp);
    if (size_out) {
        *size_out = total;
    }
    return crc;
}

static void set_text(char *dst, size_t dst_len, const char *src)
{
    if (dst_len == 0) {
        return;
    }
    strncpy(dst, src ? src : "", dst_len - 1);
    dst[dst_len - 1] = '\0';
}

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
    s_stats.sd_ready = file_open || s_stats.sd_ready;
    taskEXIT_CRITICAL(&s_stats_mux);
}

static void set_recording_state(const char *state)
{
    taskENTER_CRITICAL(&s_stats_mux);
    set_text(s_stats.recording_state, sizeof(s_stats.recording_state), state);
    taskEXIT_CRITICAL(&s_stats_mux);
}

static void set_transfer_state(const char *state)
{
    taskENTER_CRITICAL(&s_stats_mux);
    set_text(s_stats.transfer_state, sizeof(s_stats.transfer_state), state);
    taskEXIT_CRITICAL(&s_stats_mux);
}

static void set_last_error(const char *error)
{
    taskENTER_CRITICAL(&s_stats_mux);
    set_text(s_stats.last_error, sizeof(s_stats.last_error), error ? error : "none");
    taskEXIT_CRITICAL(&s_stats_mux);
}

static void finalize_metadata_locked(const char *reason)
{
    uint64_t file_size = 0;
    uint32_t checksum = s_active_path[0] ? checksum_file(s_active_path, &file_size) : 0;

    taskENTER_CRITICAL(&s_stats_mux);
    set_text(s_stats.recording_state, sizeof(s_stats.recording_state), "finalized");
    set_text(s_stats.transfer_state, sizeof(s_stats.transfer_state), "none");
    set_text(s_stats.finalization_reason, sizeof(s_stats.finalization_reason),
             reason && reason[0] ? reason : "manual_stop");
    s_stats.file_byte_size = file_size;
    s_stats.file_checksum = checksum;
    s_stats.grace_ms_remaining = 0;
    s_stats.recording_requested = false;
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

static void close_log_file(const char *reason)
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
    finalize_metadata_locked(reason);
    ESP_LOGI(TAG, "SD recording stopped");
}

static void open_log_file(const char *path)
{
    close_log_file("manual_stop");

    s_fp = fopen(path, "wb");
    if (!s_fp) {
        set_file_open(false);
        taskENTER_CRITICAL(&s_stats_mux);
        s_stats.sd_write_errors++;
        set_text(s_stats.recording_state, sizeof(s_stats.recording_state), "failed");
        set_text(s_stats.last_error, sizeof(s_stats.last_error), "open_failed");
        taskEXIT_CRITICAL(&s_stats_mux);
        ESP_LOGW(TAG, "SD log open failed at %s; recording disabled", path);
        return;
    }

    set_text(s_active_path, sizeof(s_active_path), path);
    set_file_open(true);
    set_recording_state("recording");
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
                set_recording_state("finalizing");
                close_log_file(item.reason[0] ? item.reason : "manual_stop");
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
    set_text(s_stats.recording_state, sizeof(s_stats.recording_state), "idle");
    set_text(s_stats.transfer_state, sizeof(s_stats.transfer_state), "none");
    set_text(s_stats.finalization_reason, sizeof(s_stats.finalization_reason), "none");
    set_text(s_stats.last_error, sizeof(s_stats.last_error), "none");

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
    return sd_logger_start_session(NULL, path);
}

bool sd_logger_start_session(const char *requested_session, const char *path)
{
    if (!s_stats.configured_enabled || !s_queue) {
        set_last_error("unsupported");
        return false;
    }

    sd_queue_item_t item = {
        .type = SD_QUEUE_START,
    };

    char new_session_id[sizeof(s_stats.session_id)];
    uint64_t sid = (uint64_t)esp_timer_get_time();
    if (requested_session && requested_session[0]) {
        set_text(new_session_id, sizeof(new_session_id), requested_session);
    } else {
        snprintf(new_session_id, sizeof(new_session_id), "%016llx", (unsigned long long)sid);
    }

    if (path && path[0] != '\0') {
        strncpy(item.path, path, sizeof(item.path) - 1);
    } else {
        snprintf(item.path, sizeof(item.path), "/sdcard/step_%s.bin", new_session_id);
    }

    BaseType_t ok = xQueueSend(s_queue, &item, pdMS_TO_TICKS(100));
    if (ok == pdTRUE) {
        taskENTER_CRITICAL(&s_stats_mux);
        set_text(s_stats.session_id, sizeof(s_stats.session_id), new_session_id);
        snprintf(s_stats.sd_path_token, sizeof(s_stats.sd_path_token), "sd:%s", s_stats.session_id);
        set_text(s_stats.recording_state, sizeof(s_stats.recording_state), "starting");
        set_text(s_stats.transfer_state, sizeof(s_stats.transfer_state), "none");
        set_text(s_stats.finalization_reason, sizeof(s_stats.finalization_reason), "none");
        set_text(s_stats.last_error, sizeof(s_stats.last_error), "none");
        s_stats.recording_requested = true;
        s_stats.file_byte_size = 0;
        s_stats.file_checksum = 0;
        s_stats.saved_samples = 0;
        s_stats.enqueued_samples = 0;
        s_stats.sd_queue_drops = 0;
        s_stats.sd_write_errors = 0;
        s_stats.max_sd_write_us = 0;
        s_stats.grace_ms_remaining = 0;
        taskEXIT_CRITICAL(&s_stats_mux);
    }
    return ok == pdTRUE;
}

bool sd_logger_stop(void)
{
    return sd_logger_stop_with_reason("manual_stop");
}

bool sd_logger_stop_with_reason(const char *reason)
{
    if (!s_stats.configured_enabled || !s_queue) {
        set_last_error("unsupported");
        return false;
    }

    sd_queue_item_t item = {
        .type = SD_QUEUE_STOP,
    };
    set_text(item.reason, sizeof(item.reason), reason && reason[0] ? reason : "manual_stop");

    BaseType_t ok = xQueueSend(s_queue, &item, pdMS_TO_TICKS(100));
    if (ok == pdTRUE) {
        taskENTER_CRITICAL(&s_stats_mux);
        set_text(s_stats.recording_state, sizeof(s_stats.recording_state), "finalizing");
        set_text(s_stats.finalization_reason, sizeof(s_stats.finalization_reason), item.reason);
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
    if (strcmp(s_stats.recording_state, "host-disconnected-grace") == 0 && s_grace_deadline_us > 0) {
        int64_t remain_us = s_grace_deadline_us - esp_timer_get_time();
        s_stats.grace_ms_remaining = remain_us > 0 ? (uint32_t)(remain_us / 1000) : 0;
    }
    *out = s_stats;
    taskEXIT_CRITICAL(&s_stats_mux);
}

void sd_logger_mark_host_disconnected(void)
{
    taskENTER_CRITICAL(&s_stats_mux);
    if (s_stats.recording_requested && s_stats.file_open &&
        strcmp(s_stats.recording_state, "recording") == 0) {
        set_text(s_stats.recording_state, sizeof(s_stats.recording_state), "host-disconnected-grace");
        s_grace_deadline_us = esp_timer_get_time() + (int64_t)REC_RECONNECT_GRACE_MS * 1000;
        s_stats.grace_ms_remaining = REC_RECONNECT_GRACE_MS;
    }
    taskEXIT_CRITICAL(&s_stats_mux);
}

void sd_logger_mark_host_connected(void)
{
    taskENTER_CRITICAL(&s_stats_mux);
    if (strcmp(s_stats.recording_state, "host-disconnected-grace") == 0) {
        set_text(s_stats.recording_state, sizeof(s_stats.recording_state), "recording");
        s_grace_deadline_us = 0;
        s_stats.grace_ms_remaining = 0;
    }
    taskEXIT_CRITICAL(&s_stats_mux);
}

void sd_logger_poll(void)
{
    bool timed_out = false;
    taskENTER_CRITICAL(&s_stats_mux);
    timed_out = strcmp(s_stats.recording_state, "host-disconnected-grace") == 0 &&
                s_grace_deadline_us > 0 &&
                esp_timer_get_time() >= s_grace_deadline_us;
    taskEXIT_CRITICAL(&s_stats_mux);
    if (timed_out) {
        sd_logger_stop_with_reason("disconnect_timeout");
    }
}

bool sd_logger_get_session(const char *session_id, sd_logger_session_t *out)
{
    if (!out) {
        return false;
    }

    taskENTER_CRITICAL(&s_stats_mux);
    bool latest = !session_id || strcmp(session_id, "latest_finalized") == 0;
    bool match = latest || strcmp(session_id, s_stats.session_id) == 0;
    bool finalized = strcmp(s_stats.recording_state, "finalized") == 0 ||
                     (strcmp(s_stats.recording_state, "failed") == 0 && s_stats.file_byte_size > 0);
    if (match && finalized) {
        set_text(out->session_id, sizeof(out->session_id), s_stats.session_id);
        set_text(out->sd_path_token, sizeof(out->sd_path_token), s_stats.sd_path_token);
        set_text(out->finalization_reason, sizeof(out->finalization_reason), s_stats.finalization_reason);
        out->file_byte_size = s_stats.file_byte_size;
        out->file_checksum = s_stats.file_checksum;
        out->sample_count = s_stats.saved_samples;
    }
    taskEXIT_CRITICAL(&s_stats_mux);
    return match && finalized;
}

int sd_logger_read_chunk(const char *session_id, uint64_t offset, uint8_t *buf, size_t len, size_t *out_len)
{
    sd_logger_session_t session;
    if (out_len) {
        *out_len = 0;
    }
    if (!sd_logger_get_session(session_id, &session)) {
        return -1;
    }
    if (offset > session.file_byte_size) {
        set_last_error("offset_out_of_range");
        return -2;
    }
    if (!buf || len == 0) {
        return 0;
    }
    if (offset + len > session.file_byte_size) {
        len = (size_t)(session.file_byte_size - offset);
    }

    FILE *fp = fopen(s_active_path, "rb");
    if (!fp) {
        set_last_error("sd_error");
        return -3;
    }
    if (fseek(fp, (long)offset, SEEK_SET) != 0) {
        fclose(fp);
        set_last_error("offset_out_of_range");
        return -2;
    }
    size_t n = fread(buf, 1, len, fp);
    fclose(fp);
    if (out_len) {
        *out_len = n;
    }
    return 0;
}

bool sd_logger_transfer_begin(const char *session_id)
{
    sd_logger_session_t session;
    if (!sd_logger_get_session(session_id, &session)) {
        set_last_error("not_finalized");
        return false;
    }
    set_transfer_state("chunking");
    return true;
}

bool sd_logger_transfer_complete(const char *session_id, uint64_t file_size, uint32_t file_checksum)
{
    sd_logger_session_t session;
    if (!sd_logger_get_session(session_id, &session)) {
        set_last_error("not_active");
        return false;
    }
    if (session.file_byte_size != file_size) {
        set_last_error("size_mismatch");
        set_transfer_state("failed");
        return false;
    }
    if (session.file_checksum != file_checksum) {
        set_last_error("checksum_mismatch");
        set_transfer_state("failed");
        return false;
    }
    set_transfer_state("complete");
    return true;
}

bool sd_logger_transfer_abort(const char *session_id)
{
    sd_logger_session_t session;
    if (!sd_logger_get_session(session_id, &session)) {
        set_last_error("not_active");
        return false;
    }
    set_transfer_state("aborted");
    return true;
}

bool sd_logger_clear(const char *scope)
{
    if (scope && strcmp(scope, "errors") == 0) {
        set_last_error("none");
        return true;
    }
    if (scope && strcmp(scope, "transfer") == 0) {
        set_transfer_state("none");
        return true;
    }
    if (scope && strcmp(scope, "last_finalized") == 0) {
        taskENTER_CRITICAL(&s_stats_mux);
        if (s_stats.recording_requested || s_stats.file_open) {
            taskEXIT_CRITICAL(&s_stats_mux);
            set_last_error("busy_recording");
            return false;
        }
        s_stats.session_id[0] = '\0';
        s_stats.sd_path_token[0] = '\0';
        s_stats.file_byte_size = 0;
        s_stats.file_checksum = 0;
        set_text(s_stats.finalization_reason, sizeof(s_stats.finalization_reason), "none");
        set_text(s_stats.recording_state, sizeof(s_stats.recording_state), "idle");
        taskEXIT_CRITICAL(&s_stats_mux);
        return true;
    }
    set_last_error("invalid_scope");
    return false;
}

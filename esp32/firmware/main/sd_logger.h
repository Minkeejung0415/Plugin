#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "open_ephys_stream.h"

typedef struct {
    bool configured_enabled;
    bool recording_requested;
    bool file_open;
    bool sd_ready;
    uint64_t generated_samples;
    uint64_t enqueued_samples;
    uint64_t saved_samples;
    uint64_t sd_queue_drops;
    uint64_t sd_write_errors;
    uint64_t acq_loop_overruns;
    uint32_t max_acq_loop_us;
    uint32_t max_sd_enqueue_us;
    uint32_t max_sd_write_us;
    uint32_t max_sd_queue_depth;
    uint32_t queue_capacity;
    char recording_state[32];
    char transfer_state[16];
    char session_id[33];
    char sd_path_token[64];
    char finalization_reason[32];
    char last_error[32];
    uint64_t file_byte_size;
    uint32_t file_checksum;
    uint32_t grace_ms_remaining;
} sd_logger_stats_t;

#define SD_LOGGER_SESSION_ID_MAX 32
#define SD_LOGGER_PATH_TOKEN_MAX 64
#define SD_LOGGER_MAX_CHUNK 1024

typedef struct {
    char session_id[SD_LOGGER_SESSION_ID_MAX + 1];
    char sd_path_token[SD_LOGGER_PATH_TOKEN_MAX];
    char finalization_reason[32];
    uint64_t file_byte_size;
    uint32_t file_checksum;
    uint64_t sample_count;
} sd_logger_session_t;

void sd_logger_init(void);
bool sd_logger_start(const char *path);
bool sd_logger_start_session(const char *requested_session, const char *path);
bool sd_logger_stop(void);
bool sd_logger_stop_with_reason(const char *reason);
bool sd_logger_append(const oe_sample_t *sample);
void sd_logger_note_acquisition(uint32_t loop_us, uint32_t period_us);
void sd_logger_get_stats(sd_logger_stats_t *out);
void sd_logger_mark_host_disconnected(void);
void sd_logger_mark_host_connected(void);
void sd_logger_poll(void);
bool sd_logger_get_session(const char *session_id, sd_logger_session_t *out);
int sd_logger_read_chunk(const char *session_id, uint64_t offset, uint8_t *buf, size_t len, size_t *out_len);
bool sd_logger_transfer_begin(const char *session_id);
bool sd_logger_transfer_complete(const char *session_id, uint64_t file_size, uint32_t file_checksum);
bool sd_logger_transfer_abort(const char *session_id);
bool sd_logger_clear(const char *scope);

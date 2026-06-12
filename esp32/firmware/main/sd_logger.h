#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "open_ephys_stream.h"

typedef struct {
    bool configured_enabled;
    bool recording_requested;
    bool file_open;
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
} sd_logger_stats_t;

void sd_logger_init(void);
bool sd_logger_start(const char *path);
bool sd_logger_stop(void);
bool sd_logger_append(const oe_sample_t *sample);
void sd_logger_note_acquisition(uint32_t loop_us, uint32_t period_us);
void sd_logger_get_stats(sd_logger_stats_t *out);

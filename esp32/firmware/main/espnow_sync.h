#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t master_seq;
    int64_t master_time_us;
    int32_t clock_offset_us;
} sync_state_t;

void espnow_sync_init(bool is_master);
void espnow_sync_get_state(sync_state_t *out);
void espnow_sync_on_local_frame(uint32_t seq, int64_t time_us);

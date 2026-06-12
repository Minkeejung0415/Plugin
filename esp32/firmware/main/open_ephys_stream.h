#pragma once

#include <stdint.h>
#include <stdbool.h>

#define OE_STREAM_NUM_CHANNELS 8
#define OE_STREAM_SAMPLES_PER_CHANNEL 1
#define OE_STREAM_SAMPLE_HZ 100

typedef struct {
    int64_t timestamp_us;
    uint32_t seq;
    int16_t ch[OE_STREAM_NUM_CHANNELS];
} oe_sample_t;

void open_ephys_stream_init(void);
void open_ephys_stream_set_sample(const oe_sample_t *sample);
void open_ephys_stream_start_server(uint16_t port);

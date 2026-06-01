#pragma once
#include <stdint.h>
#include "esp_err.h"

/* Protocol-compatible with RedPitaya_justin.c so Open Ephys plugin
   requires zero changes.  Same TCP:5000 control channel,
   same UDP:55001 data channel, same UDP quaternion channel.            */

typedef struct {
    uint16_t total_channels;
    uint16_t sample_rate_hz;
} stream_cfg_t;

esp_err_t network_stream_init(void);

/* Called by acquisition task each sample period */
void      network_stream_push_sample(const int16_t *channels, uint16_t nch,
                                     uint32_t seq);

/* Called by sensor fusion when new quaternion set is ready */
void      network_stream_push_quats(const float *quats, uint8_t n_sensors,
                                    float timestamp_s);

/* Query live config (rate, filter state, etc.) */
uint16_t  network_stream_get_rate(void);
bool      network_stream_fusion_enabled(void);
bool      network_stream_recording(void);

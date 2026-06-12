#pragma once

#include "open_ephys_stream.h"

void sd_logger_init(void);
void sd_logger_append(const oe_sample_t *sample);

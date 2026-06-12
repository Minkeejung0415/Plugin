#pragma once

#include <stdbool.h>
#include "open_ephys_stream.h"

#define ICM20948_I2C_ADDR 0x69

bool icm20948_init(void);
bool icm20948_read_scaled(int16_t out[6]);

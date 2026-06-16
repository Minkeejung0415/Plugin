#pragma once

#include <stdbool.h>
#include "open_ephys_stream.h"

#define ICM20948_SPI_SCK_GPIO 7
#define ICM20948_SPI_MISO_GPIO 8
#define ICM20948_SPI_MOSI_GPIO 9
#define ICM20948_SPI_CS_GPIO 44

bool icm20948_init(void);
bool icm20948_read_scaled(int16_t out[6]);

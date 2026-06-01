#pragma once
#include <stdint.h>
#include "esp_err.h"

/* Binary log format is byte-compatible with RedPitaya's .bin file so
   the same offline analysis toolchain applies.                          */

esp_err_t sdcard_init(void);

/* Open a new session file pair: <base>.bin and <base>.csv */
esp_err_t sdcard_open_session(char *base_path_out, size_t out_len);
esp_err_t sdcard_close_session(void);

/* Write one sample row.  channels[] in Q15 int16, same layout as UDP. */
esp_err_t sdcard_write_sample(const int16_t *channels, uint16_t nch,
                               uint32_t seq, float timestamp_s);

/* Write an action-verify JPEG frame with its IMU timestamp */
esp_err_t sdcard_write_jpeg(const uint8_t *buf, size_t len,
                             uint32_t seq, float timestamp_s);

/* Flush pending writes (call before power-down) */
esp_err_t sdcard_sync(void);

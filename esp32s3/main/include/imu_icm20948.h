#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float ax, ay, az;   /* m/s² */
    float gx, gy, gz;   /* rad/s */
    float mx, my, mz;   /* µT */
    int64_t timestamp_us;
} icm20948_data_t;

typedef struct {
    uint8_t accel_range;  /* 0=±2g 1=±4g 2=±8g 3=±16g */
    uint8_t gyro_range;   /* 0=±250°/s 1=±500 2=±1000 3=±2000 */
    uint16_t odr_hz;
} icm20948_cfg_t;

esp_err_t icm20948_init(const icm20948_cfg_t *cfg);
esp_err_t icm20948_read(icm20948_data_t *out);
esp_err_t icm20948_set_accel_range(uint8_t range);
esp_err_t icm20948_set_gyro_range(uint8_t range);
esp_err_t icm20948_calibrate_gyro(void);  /* collect bias at rest */

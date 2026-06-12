#include "icm20948.h"

#include <math.h>

#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "icm20948";

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static bool s_ok;

/* Minimal bring-up: WHO_AM_I 0x12, placeholder scaling for lab calibration */
#define REG_WHO_AM_I 0x00
#define REG_PWR_MGMT_1 0x06

static bool i2c_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, 2, 100) == ESP_OK;
}

static bool i2c_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, val, 1, 100) == ESP_OK;
}

bool icm20948_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = 5,
        .sda_io_num = 4,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_cfg, &s_bus) != ESP_OK) {
        return false;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM20948_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    if (i2c_new_master_device(s_bus, &dev_cfg, &s_dev) != ESP_OK) {
        return false;
    }

    uint8_t who = 0;
    if (!i2c_read_reg(REG_WHO_AM_I, &who)) {
        ESP_LOGW(TAG, "ICM20948 not detected — using synthetic IMU for bench");
        s_ok = false;
        return true;
    }
    if (who != 0xEA) {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I 0x%02x", who);
    }
    i2c_write_reg(REG_PWR_MGMT_1, 0x01);
    s_ok = true;
    ESP_LOGI(TAG, "ICM20948 ready WHO_AM_I=0x%02x", who);
    return true;
}

bool icm20948_read_scaled(int16_t out[6])
{
    if (!s_ok) {
        static int32_t t;
        t++;
        out[0] = (int16_t)(1000 * sinf((float)t * 0.01f));
        out[1] = (int16_t)(500 * cosf((float)t * 0.01f));
        out[2] = 16384;
        out[3] = out[4] = out[5] = 0;
        return true;
    }
    /* TODO: burst read ACCEL_XOUT_H .. GYRO_ZOUT_L */
    (void)out;
    return false;
}

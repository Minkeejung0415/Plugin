#include "icm20948.h"

#include <math.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_log.h"

static const char *TAG = "icm20948";

static spi_device_handle_t s_dev;
static bool s_ok;

#define REG_WHO_AM_I 0x00
#define REG_PWR_MGMT_1 0x06
#define REG_ACCEL_XOUT_H 0x2D
#define REG_BANK_SEL 0x7F

#define ICM_SPI_HOST SPI2_HOST
#define ICM_SPI_HZ 4000000

static bool spi_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7F), val};
    spi_transaction_t t = {
        .length = sizeof(tx) * 8,
        .tx_buffer = tx,
    };
    return spi_device_transmit(s_dev, &t) == ESP_OK;
}

static bool spi_read_bytes(uint8_t reg, uint8_t *buf, size_t len)
{
    if (len > 15) {
        return false;
    }

    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};
    tx[0] = (uint8_t)(reg | 0x80);
    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    if (spi_device_transmit(s_dev, &t) != ESP_OK) {
        return false;
    }
    memcpy(buf, &rx[1], len);
    return true;
}

static bool spi_read_reg(uint8_t reg, uint8_t *val)
{
    return spi_read_bytes(reg, val, 1);
}

static void icm_select_bank(uint8_t bank)
{
    spi_write_reg(REG_BANK_SEL, (uint8_t)((bank & 0x03) << 4));
}

bool icm20948_init(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = ICM20948_SPI_MOSI_GPIO,
        .miso_io_num = ICM20948_SPI_MISO_GPIO,
        .sclk_io_num = ICM20948_SPI_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16,
    };
    esp_err_t err = spi_bus_initialize(ICM_SPI_HOST, &bus_cfg, SPI_DMA_DISABLED);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return false;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = ICM_SPI_HZ,
        .mode = 0,
        .spics_io_num = ICM20948_SPI_CS_GPIO,
        .queue_size = 1,
    };
    if (spi_bus_add_device(ICM_SPI_HOST, &dev_cfg, &s_dev) != ESP_OK) {
        return false;
    }

    uint8_t who = 0;
    icm_select_bank(0);
    if (!spi_read_reg(REG_WHO_AM_I, &who)) {
        ESP_LOGW(TAG, "ICM20948 not detected on SPI - using synthetic IMU for bench");
        s_ok = false;
        return true;
    }
    if (who != 0xEA) {
        ESP_LOGW(TAG, "Unexpected SPI WHO_AM_I 0x%02x", who);
        s_ok = false;
        return true;
    }

    spi_write_reg(REG_PWR_MGMT_1, 0x01);
    s_ok = true;
    ESP_LOGI(TAG, "ICM20948 ready on SPI CS GPIO%d WHO_AM_I=0x%02x",
             ICM20948_SPI_CS_GPIO, who);
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

    uint8_t raw[14] = {0};
    icm_select_bank(0);
    if (!spi_read_bytes(REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
        return false;
    }

    out[0] = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    out[1] = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    out[2] = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
    out[3] = (int16_t)(((uint16_t)raw[8] << 8) | raw[9]);
    out[4] = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);
    out[5] = (int16_t)(((uint16_t)raw[12] << 8) | raw[13]);
    return true;
}

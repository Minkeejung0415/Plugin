/* ICM-20948 driver over SPI (host = SPI2).
   Bank-switch + AK09916 magnetometer access via passthrough mode.      */
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "include/config.h"
#include "include/imu_icm20948.h"

static const char *TAG = "icm20948";

/* ── Register map (bank 0 unless noted) ──────────────────────────────── */
#define REG_WHO_AM_I        0x00
#define REG_USER_CTRL       0x03
#define REG_PWR_MGMT_1      0x06
#define REG_PWR_MGMT_2      0x07
#define REG_INT_PIN_CFG     0x0F
#define REG_ACCEL_XOUT_H    0x2D
#define REG_GYRO_XOUT_H     0x33
#define REG_BANK_SEL        0x7F
/* Bank 2 */
#define REG_GYRO_SMPLRT_DIV 0x00   /* bank 2 */
#define REG_GYRO_CONFIG_1   0x01
#define REG_ACCEL_SMPLRT_DIV_1 0x10
#define REG_ACCEL_SMPLRT_DIV_2 0x11
#define REG_ACCEL_CONFIG    0x14
/* Bank 3 – I2C master for AK09916 */
#define REG_I2C_MST_CTRL    0x01   /* bank 3 */
#define REG_I2C_SLV0_ADDR   0x03
#define REG_I2C_SLV0_REG    0x04
#define REG_I2C_SLV0_CTRL   0x05
/* Bank 0 – EXT_SLV_SENS_DATA */
#define REG_EXT_SLV_DATA_00 0x3B
/* AK09916 */
#define AK09916_ADDR        0x0C
#define AK_REG_ST1          0x10
#define AK_REG_HXL          0x11
#define AK_REG_CNTL2        0x31
#define ICM_WHO_AM_I_VAL    0xEA

/* ── Sensitivity constants ──────────────────────────────────────────── */
static const float k_accel_scale[4] = {
    2.0f/32768.f, 4.0f/32768.f, 8.0f/32768.f, 16.0f/32768.f
};
static const float k_gyro_scale[4] = {
    250.0f/32768.f, 500.0f/32768.f, 1000.0f/32768.f, 2000.0f/32768.f
};
#define G_MPS2   9.80665f
#define DEG2RAD  (M_PI/180.0f)
#define MAG_SCALE_UT  (4912.0f/32752.0f)   /* AK09916 ±4912 µT */

static spi_device_handle_t s_spi;
static icm20948_cfg_t      s_cfg;
static float               s_gyro_bias[3] = {0};

/* ── SPI helpers ─────────────────────────────────────────────────────── */
static void spi_write_reg(uint8_t reg, uint8_t val)
{
    spi_transaction_t t = {
        .length = 16,
        .tx_data = {reg & 0x7F, val},
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    spi_device_polling_transmit(s_spi, &t);
}

static uint8_t spi_read_reg(uint8_t reg)
{
    spi_transaction_t t = {
        .length = 16,
        .tx_data = {reg | 0x80, 0x00},
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    spi_device_polling_transmit(s_spi, &t);
    return t.rx_data[1];
}

static void spi_read_burst(uint8_t reg, uint8_t *buf, int len)
{
    uint8_t tx[len + 1];
    uint8_t rx[len + 1];
    memset(tx, 0, sizeof(tx));
    tx[0] = reg | 0x80;
    spi_transaction_t t = {
        .length   = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_polling_transmit(s_spi, &t);
    memcpy(buf, &rx[1], len);
}

static void bank_sel(uint8_t bank)
{
    spi_write_reg(REG_BANK_SEL, (bank & 0x03) << 4);
}

/* ── AK09916 helpers (via I2C master on ICM-20948) ──────────────────── */
static void ak_write(uint8_t reg, uint8_t val)
{
    bank_sel(3);
    spi_write_reg(REG_I2C_SLV0_ADDR, AK09916_ADDR);
    spi_write_reg(REG_I2C_SLV0_REG,  reg);
    /* write single byte via SLV0 */
    spi_write_reg(0x06, val);   /* I2C_SLV0_DO */
    spi_write_reg(REG_I2C_SLV0_CTRL, 0x81);
    bank_sel(0);
    vTaskDelay(pdMS_TO_TICKS(1));
}

static void ak_setup_read(uint8_t reg, uint8_t len)
{
    bank_sel(3);
    spi_write_reg(REG_I2C_SLV0_ADDR, AK09916_ADDR | 0x80);
    spi_write_reg(REG_I2C_SLV0_REG,  reg);
    spi_write_reg(REG_I2C_SLV0_CTRL, 0x80 | len);
    bank_sel(0);
}

/* ── Init ────────────────────────────────────────────────────────────── */
esp_err_t icm20948_init(const icm20948_cfg_t *cfg)
{
    s_cfg = *cfg;

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_IMU_MOSI,
        .miso_io_num = PIN_IMU_MISO,
        .sclk_io_num = PIN_IMU_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 7000000,
        .mode           = 3,
        .spics_io_num   = PIN_IMU_CS,
        .queue_size     = 4,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi));

    /* Reset */
    bank_sel(0);
    spi_write_reg(REG_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));
    spi_write_reg(REG_PWR_MGMT_1, 0x01);   /* auto-select clk */
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t who = spi_read_reg(REG_WHO_AM_I);
    if (who != ICM_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I = 0x%02X (expected 0xEA)", who);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "ICM-20948 found");

    /* Enable accel + gyro */
    spi_write_reg(REG_PWR_MGMT_2, 0x00);

    /* Accel config */
    bank_sel(2);
    uint16_t a_div = (1125 / cfg->odr_hz) - 1;
    spi_write_reg(REG_ACCEL_SMPLRT_DIV_1, (a_div >> 8) & 0x0F);
    spi_write_reg(REG_ACCEL_SMPLRT_DIV_2, a_div & 0xFF);
    spi_write_reg(REG_ACCEL_CONFIG, (cfg->accel_range << 1) | 0x01);  /* DLPF on */

    /* Gyro config */
    uint8_t g_div = (1100 / cfg->odr_hz) - 1;
    spi_write_reg(REG_GYRO_SMPLRT_DIV, g_div);
    spi_write_reg(REG_GYRO_CONFIG_1, (cfg->gyro_range << 1) | 0x01);

    /* I2C master for AK09916 */
    bank_sel(0);
    spi_write_reg(REG_USER_CTRL, 0x20);   /* I2C_MST_EN */
    bank_sel(3);
    spi_write_reg(REG_I2C_MST_CTRL, 0x17);  /* 400 kHz */
    bank_sel(0);

    /* Reset and configure AK09916 */
    ak_write(AK_REG_CNTL2, 0x01);   /* soft reset */
    vTaskDelay(pdMS_TO_TICKS(10));
    ak_write(AK_REG_CNTL2, 0x08);   /* continuous mode 2: 100 Hz */

    /* Set AK to auto-read into EXT_SLV_SENS_DATA registers */
    ak_setup_read(AK_REG_ST1, 9);   /* ST1 + HXL…HZH + ST2 */

    ESP_LOGI(TAG, "ICM-20948 configured: accel range %d, gyro range %d, %d Hz",
             cfg->accel_range, cfg->gyro_range, cfg->odr_hz);
    return ESP_OK;
}

/* ── Read ────────────────────────────────────────────────────────────── */
esp_err_t icm20948_read(icm20948_data_t *out)
{
    bank_sel(0);
    uint8_t buf[12];
    spi_read_burst(REG_ACCEL_XOUT_H, buf, 12);

    int16_t ax_r = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t ay_r = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t az_r = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t gx_r = (int16_t)((buf[6]  << 8) | buf[7]);
    int16_t gy_r = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gz_r = (int16_t)((buf[10] << 8) | buf[11]);

    float a_sc = k_accel_scale[s_cfg.accel_range] * G_MPS2;
    float g_sc = k_gyro_scale[s_cfg.gyro_range]   * DEG2RAD;

    out->ax = ax_r * a_sc;
    out->ay = ay_r * a_sc;
    out->az = az_r * a_sc;
    out->gx = gx_r * g_sc - s_gyro_bias[0];
    out->gy = gy_r * g_sc - s_gyro_bias[1];
    out->gz = gz_r * g_sc - s_gyro_bias[2];

    /* Magnetometer from EXT_SLV_SENS_DATA (9 bytes: ST1 HXL HXH … ST2) */
    uint8_t mag[9];
    spi_read_burst(REG_EXT_SLV_DATA_00, mag, 9);
    if (mag[0] & 0x01) {   /* DRDY */
        int16_t mx_r = (int16_t)((mag[2] << 8) | mag[1]);
        int16_t my_r = (int16_t)((mag[4] << 8) | mag[3]);
        int16_t mz_r = (int16_t)((mag[6] << 8) | mag[5]);
        out->mx = mx_r * MAG_SCALE_UT;
        out->my = my_r * MAG_SCALE_UT;
        out->mz = mz_r * MAG_SCALE_UT;
    }

    out->timestamp_us = esp_timer_get_time();
    return ESP_OK;
}

/* ── Range setters ───────────────────────────────────────────────────── */
esp_err_t icm20948_set_accel_range(uint8_t range)
{
    if (range > 3) return ESP_ERR_INVALID_ARG;
    s_cfg.accel_range = range;
    bank_sel(2);
    uint8_t v = spi_read_reg(REG_ACCEL_CONFIG);
    v = (v & ~0x06) | (range << 1);
    spi_write_reg(REG_ACCEL_CONFIG, v);
    bank_sel(0);
    return ESP_OK;
}

esp_err_t icm20948_set_gyro_range(uint8_t range)
{
    if (range > 3) return ESP_ERR_INVALID_ARG;
    s_cfg.gyro_range = range;
    bank_sel(2);
    uint8_t v = spi_read_reg(REG_GYRO_CONFIG_1);
    v = (v & ~0x06) | (range << 1);
    spi_write_reg(REG_GYRO_CONFIG_1, v);
    bank_sel(0);
    return ESP_OK;
}

/* ── Gyro bias calibration (200 samples at rest) ────────────────────── */
esp_err_t icm20948_calibrate_gyro(void)
{
    ESP_LOGI(TAG, "Calibrating gyro bias — keep sensor still...");
    double bx = 0, by = 0, bz = 0;
    const int N = 200;
    for (int i = 0; i < N; i++) {
        icm20948_data_t d;
        icm20948_read(&d);
        /* read before bias subtraction: temporarily zero bias */
        bx += d.gx + s_gyro_bias[0];
        by += d.gy + s_gyro_bias[1];
        bz += d.gz + s_gyro_bias[2];
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    s_gyro_bias[0] = (float)(bx / N);
    s_gyro_bias[1] = (float)(by / N);
    s_gyro_bias[2] = (float)(bz / N);
    ESP_LOGI(TAG, "Gyro bias: %.4f %.4f %.4f rad/s",
             s_gyro_bias[0], s_gyro_bias[1], s_gyro_bias[2]);
    return ESP_OK;
}

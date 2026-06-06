/*
 * imu_registers.h — Consolidated register map for all IMU types used in this project.
 *
 * Centralises hardware constants that were previously scattered across
 * RedPitaya_justin.c and axi_header.h.  Include this header instead of
 * re-declaring register addresses in each translation unit.
 *
 * Supported devices
 * -----------------
 *   MPU6050  (I²C, InvenSense RM-MPU-6000A)
 *   MPU9250  (I²C / SPI, InvenSense PS-MPU-9250A)
 *   ICM20948 (SPI, TDK InvenSense DS-000189)
 *     + AK09916 auxiliary magnetometer (accessed via ICM20948 I²C master)
 *   BNO055   (I²C, Bosch BST-BNO055-DS000)
 *
 * Sensitivity defaults (full-scale = factory default at power-on)
 * ---------------------------------------------------------------
 *   MPU6050 / MPU9250  accel  ±2 g    → 16384 LSB/g
 *   MPU6050 / MPU9250  gyro   ±250 °/s → 131.072 LSB/(°/s)
 *   MPU9250            mag    AK8963 16-bit → 0.15 µT/LSB
 *   ICM20948           accel  ±2 g    → 16384 LSB/g
 *   ICM20948           gyro   ±250 °/s → 131.072 LSB/(°/s)
 *   ICM20948/AK09916   mag    16-bit  → 0.15 µT/LSB
 *   BNO055             accel  100 LSB/(m/s²)
 *   BNO055             gyro   16 LSB/(°/s)
 *   BNO055             mag    16 LSB/µT
 */

#ifndef IMU_REGISTERS_H
#define IMU_REGISTERS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * MPU-6050 / MPU-6500 register map
 * ========================================================================= */

/* I²C addresses (AD0 pin selects between the two) */
#define MPU6050_I2C_ADDR_LOW    0x68u  /* AD0 = GND */
#define MPU6050_I2C_ADDR_HIGH   0x69u  /* AD0 = VCC */

/* Configuration registers */
#define MPU6050_REG_SMPLRT_DIV      0x19u  /* Sample Rate = Gyro_Rate / (1 + SMPLRT_DIV) */
#define MPU6050_REG_CONFIG          0x1Au  /* DLPF and FSYNC config */
#define MPU6050_REG_GYRO_CONFIG     0x1Bu  /* Gyro full-scale select */
#define MPU6050_REG_ACCEL_CONFIG    0x1Cu  /* Accel full-scale select */

/* Gyro full-scale field (bits [4:3] of GYRO_CONFIG) */
#define MPU6050_GYRO_FS_250DPS      0x00u  /* ±250  °/s  – default */
#define MPU6050_GYRO_FS_500DPS      0x08u  /* ±500  °/s */
#define MPU6050_GYRO_FS_1000DPS     0x10u  /* ±1000 °/s */
#define MPU6050_GYRO_FS_2000DPS     0x18u  /* ±2000 °/s */

/* Accel full-scale field (bits [4:3] of ACCEL_CONFIG) */
#define MPU6050_ACCEL_FS_2G         0x00u  /* ±2 g  – default */
#define MPU6050_ACCEL_FS_4G         0x08u  /* ±4 g */
#define MPU6050_ACCEL_FS_8G         0x10u  /* ±8 g */
#define MPU6050_ACCEL_FS_16G        0x18u  /* ±16 g */

/* Sensitivity scales (LSB per physical unit) at factory default full-scale */
#define MPU6050_ACCEL_LSB_PER_G     16384.0f  /* ±2 g  range */
#define MPU6050_GYRO_LSB_PER_DPS    131.072f  /* ±250 °/s range */

/* Data output registers (burst-read 14 bytes from ACCEL_XOUT_H) */
#define MPU6050_REG_ACCEL_XOUT_H    0x3Bu
#define MPU6050_REG_ACCEL_XOUT_L    0x3Cu
#define MPU6050_REG_ACCEL_YOUT_H    0x3Du
#define MPU6050_REG_ACCEL_YOUT_L    0x3Eu
#define MPU6050_REG_ACCEL_ZOUT_H    0x3Fu
#define MPU6050_REG_ACCEL_ZOUT_L    0x40u
#define MPU6050_REG_TEMP_OUT_H      0x41u
#define MPU6050_REG_TEMP_OUT_L      0x42u
#define MPU6050_REG_GYRO_XOUT_H     0x43u
#define MPU6050_REG_GYRO_XOUT_L     0x44u
#define MPU6050_REG_GYRO_YOUT_H     0x45u
#define MPU6050_REG_GYRO_YOUT_L     0x46u
#define MPU6050_REG_GYRO_ZOUT_H     0x47u
#define MPU6050_REG_GYRO_ZOUT_L     0x48u

/* Power management */
#define MPU6050_REG_PWR_MGMT_1      0x6Bu
#define MPU6050_REG_PWR_MGMT_2      0x6Cu

/* Bit fields for PWR_MGMT_1 */
#define MPU6050_PWR1_DEVICE_RESET   0x80u
#define MPU6050_PWR1_SLEEP          0x40u
#define MPU6050_PWR1_CLKSEL_XGYRO   0x01u  /* PLL with X-axis gyro reference */

/* Who-am-I */
#define MPU6050_REG_WHO_AM_I        0x75u
#define MPU6050_WHO_AM_I_VALUE      0x68u

/* Temperature conversion: Temp_degC = (raw / 340.0) + 36.53 */
#define MPU6050_TEMP_SENSITIVITY    340.0f
#define MPU6050_TEMP_OFFSET         36.53f


/* =========================================================================
 * MPU-9250 register map (accel/gyro identical to MPU-6050; adds AK8963 mag)
 * ========================================================================= */

#define MPU9250_I2C_ADDR_LOW        0x68u
#define MPU9250_I2C_ADDR_HIGH       0x69u

/* Shared with MPU-6050 */
#define MPU9250_REG_SMPLRT_DIV      MPU6050_REG_SMPLRT_DIV
#define MPU9250_REG_CONFIG          MPU6050_REG_CONFIG
#define MPU9250_REG_GYRO_CONFIG     MPU6050_REG_GYRO_CONFIG
#define MPU9250_REG_ACCEL_CONFIG    MPU6050_REG_ACCEL_CONFIG
#define MPU9250_REG_ACCEL_XOUT_H    MPU6050_REG_ACCEL_XOUT_H
#define MPU9250_REG_PWR_MGMT_1      MPU6050_REG_PWR_MGMT_1
#define MPU9250_REG_WHO_AM_I        MPU6050_REG_WHO_AM_I
#define MPU9250_WHO_AM_I_VALUE      0x71u  /* differs from MPU-6050 */
#define MPU9250_WHO_AM_I_VALUE_ALT  0x73u  /* MPU-9255 variant */

/* MPU-9250 I²C master control (for AK8963 pass-through) */
#define MPU9250_REG_I2C_MST_CTRL    0x24u
#define MPU9250_REG_I2C_SLV0_ADDR   0x25u
#define MPU9250_REG_I2C_SLV0_REG    0x26u
#define MPU9250_REG_I2C_SLV0_CTRL   0x27u
#define MPU9250_REG_EXT_SENS_DATA_00 0x49u
#define MPU9250_REG_USER_CTRL       0x6Au
#define MPU9250_REG_INT_PIN_CFG     0x37u  /* Set BYPASS_EN to access AK8963 directly */

/* Sensitivity (same defaults as MPU-6050) */
#define MPU9250_ACCEL_LSB_PER_G     MPU6050_ACCEL_LSB_PER_G
#define MPU9250_GYRO_LSB_PER_DPS    MPU6050_GYRO_LSB_PER_DPS

/* AK8963 magnetometer (auxiliary I²C slave inside MPU-9250 package) */
#define AK8963_I2C_ADDR             0x0Cu
#define AK8963_REG_WIA              0x00u  /* Who-am-I; expected value 0x48 */
#define AK8963_WHO_AM_I_VALUE       0x48u
#define AK8963_REG_INFO             0x01u
#define AK8963_REG_ST1              0x02u  /* Status 1: DRDY bit */
#define AK8963_REG_HXL              0x03u  /* Mag X low byte (burst-read 7 bytes ST1..ST2) */
#define AK8963_REG_HXH              0x04u
#define AK8963_REG_HYL              0x05u
#define AK8963_REG_HYH              0x06u
#define AK8963_REG_HZL              0x07u
#define AK8963_REG_HZH              0x08u
#define AK8963_REG_ST2              0x09u  /* Status 2: HOFL overflow bit */
#define AK8963_REG_CNTL1            0x0Au  /* Mode control */
#define AK8963_REG_CNTL2            0x0Bu  /* Soft reset */
#define AK8963_REG_ASAX             0x10u  /* Sensitivity adjustment X (factory ROM) */
#define AK8963_REG_ASAY             0x11u
#define AK8963_REG_ASAZ             0x12u

/* AK8963 CNTL1 mode values */
#define AK8963_MODE_POWER_DOWN      0x00u
#define AK8963_MODE_SINGLE          0x01u
#define AK8963_MODE_CONT_8HZ        0x02u
#define AK8963_MODE_CONT_100HZ      0x06u
#define AK8963_MODE_14BIT           0x00u  /* Output bit: 0 = 14-bit */
#define AK8963_MODE_16BIT           0x10u  /* Output bit: 1 = 16-bit */

/* AK8963 sensitivity: 16-bit mode = 0.15 µT/LSB */
#define AK8963_MAG_UT_PER_LSB_16BIT 0.15f


/* =========================================================================
 * ICM-20948 register map (SPI; bank-switched register space)
 * ========================================================================= */

/* Bank selection register — present in every bank */
#define ICM20948_REG_BANK_SEL           0x7Fu

/* Bank indices */
#define ICM20948_BANK_0                 0x00u
#define ICM20948_BANK_1                 0x01u
#define ICM20948_BANK_2                 0x02u
#define ICM20948_BANK_3                 0x03u

/* Bank 0 */
#define ICM20948_B0_WHO_AM_I            0x00u
#define ICM20948_B0_WHO_AM_I_VALUE      0xEAu
#define ICM20948_B0_USER_CTRL           0x03u
#define ICM20948_B0_LP_CONFIG           0x05u
#define ICM20948_B0_PWR_MGMT_1         0x06u
#define ICM20948_B0_PWR_MGMT_2         0x07u
#define ICM20948_B0_INT_PIN_CFG        0x0Fu
#define ICM20948_B0_INT_ENABLE          0x10u
#define ICM20948_B0_INT_ENABLE_1        0x11u
#define ICM20948_B0_ACCEL_XOUT_H        0x2Du  /* burst-read 12 bytes → ax ay az gx gy gz */
#define ICM20948_B0_ACCEL_XOUT_L        0x2Eu
#define ICM20948_B0_ACCEL_YOUT_H        0x2Fu
#define ICM20948_B0_ACCEL_YOUT_L        0x30u
#define ICM20948_B0_ACCEL_ZOUT_H        0x31u
#define ICM20948_B0_ACCEL_ZOUT_L        0x32u
#define ICM20948_B0_GYRO_XOUT_H         0x33u
#define ICM20948_B0_GYRO_XOUT_L         0x34u
#define ICM20948_B0_GYRO_YOUT_H         0x35u
#define ICM20948_B0_GYRO_YOUT_L         0x36u
#define ICM20948_B0_GYRO_ZOUT_H         0x37u
#define ICM20948_B0_GYRO_ZOUT_L         0x38u
#define ICM20948_B0_TEMP_OUT_H          0x39u
#define ICM20948_B0_TEMP_OUT_L          0x3Au
#define ICM20948_B0_EXT_SENS_DATA_00    0x3Bu  /* First byte of I²C master read-back */

/* Bank 0 – USER_CTRL bit fields */
#define ICM20948_USER_CTRL_I2C_MST_EN   0x20u  /* Enable I²C master for AK09916 */
#define ICM20948_USER_CTRL_I2C_IF_DIS   0x10u  /* Disable I²C (SPI-only mode) */
#define ICM20948_USER_CTRL_DMP_RST      0x08u
#define ICM20948_USER_CTRL_SRAM_RST     0x04u
#define ICM20948_USER_CTRL_I2C_MST_RST  0x02u

/* Bank 0 – PWR_MGMT_1 bit fields */
#define ICM20948_PWR1_DEVICE_RESET      0x80u
#define ICM20948_PWR1_SLEEP             0x40u
#define ICM20948_PWR1_LP_EN             0x20u
#define ICM20948_PWR1_CLKSEL_AUTO       0x01u  /* Auto-select best available clock */

/* Bank 2 – accel / gyro ODR and full-scale */
#define ICM20948_B2_GYRO_SMPLRT_DIV     0x00u
#define ICM20948_B2_GYRO_CONFIG_1       0x01u
#define ICM20948_B2_GYRO_CONFIG_2       0x02u
#define ICM20948_B2_ACCEL_SMPLRT_DIV_1  0x10u
#define ICM20948_B2_ACCEL_SMPLRT_DIV_2  0x11u
#define ICM20948_B2_ACCEL_CONFIG        0x14u

/* Gyro full-scale field (bits [2:1] of GYRO_CONFIG_1) */
#define ICM20948_GYRO_FS_250DPS         0x00u
#define ICM20948_GYRO_FS_500DPS         0x02u
#define ICM20948_GYRO_FS_1000DPS        0x04u
#define ICM20948_GYRO_FS_2000DPS        0x06u

/* Accel full-scale field (bits [2:1] of ACCEL_CONFIG) */
#define ICM20948_ACCEL_FS_2G            0x00u
#define ICM20948_ACCEL_FS_4G            0x02u
#define ICM20948_ACCEL_FS_8G            0x04u
#define ICM20948_ACCEL_FS_16G           0x06u

/* Sensitivity at factory defaults */
#define ICM20948_ACCEL_LSB_PER_G        16384.0f  /* ±2 g */
#define ICM20948_GYRO_LSB_PER_DPS       131.072f  /* ±250 °/s */

/* Bank 3 – I²C master (for AK09916) */
#define ICM20948_B3_I2C_MST_CTRL        0x01u
#define ICM20948_B3_I2C_MST_DELAY_CTRL  0x02u
#define ICM20948_B3_I2C_SLV0_ADDR       0x03u
#define ICM20948_B3_I2C_SLV0_REG        0x04u
#define ICM20948_B3_I2C_SLV0_CTRL       0x05u
#define ICM20948_B3_I2C_SLV0_DO         0x06u

/* AK09916 magnetometer (auxiliary I²C slave inside ICM-20948 package) */
#define ICM20948_AK09916_I2C_ADDR       0x0Cu
#define AK09916_REG_WIA2                0x01u  /* Who-am-I byte 2; expected 0x09 */
#define AK09916_WHO_AM_I_VALUE          0x09u
#define AK09916_REG_ST1                 0x10u  /* DRDY flag */
#define AK09916_REG_HXL                 0x11u  /* Burst-read 8 bytes HXL..ST2 */
#define AK09916_REG_HXH                 0x12u
#define AK09916_REG_HYL                 0x13u
#define AK09916_REG_HYH                 0x14u
#define AK09916_REG_HZL                 0x15u
#define AK09916_REG_HZH                 0x16u
#define AK09916_REG_TMPS                0x17u  /* Dummy byte — must be read before ST2 */
#define AK09916_REG_ST2                 0x18u  /* HOFL overflow flag */
#define AK09916_REG_CNTL2               0x31u  /* Mode control */
#define AK09916_REG_CNTL3               0x32u  /* Soft reset */

/* AK09916 CNTL2 mode values */
#define AK09916_MODE_POWER_DOWN         0x00u
#define AK09916_MODE_SINGLE             0x01u
#define AK09916_MODE_CONT_10HZ          0x02u
#define AK09916_MODE_CONT_20HZ          0x04u
#define AK09916_MODE_CONT_50HZ          0x06u
#define AK09916_MODE_CONT_100HZ         0x08u

/* AK09916 sensitivity: always 16-bit = 0.15 µT/LSB */
#define AK09916_MAG_UT_PER_LSB          0.15f


/* =========================================================================
 * BNO055 register map (I²C, Bosch)
 * ========================================================================= */

#define BNO055_I2C_ADDR_LOW             0x28u  /* COM3 = GND */
#define BNO055_I2C_ADDR_HIGH            0x29u  /* COM3 = VCC */

/* Page 0 registers */
#define BNO055_REG_CHIP_ID              0x00u
#define BNO055_CHIP_ID_VALUE            0xA0u
#define BNO055_REG_ACC_ID               0x01u
#define BNO055_REG_MAG_ID               0x02u
#define BNO055_REG_GYR_ID               0x03u
#define BNO055_REG_OPR_MODE             0x3Du
#define BNO055_REG_PWR_MODE             0x3Eu
#define BNO055_REG_SYS_TRIGGER          0x3Fu

/* Operating mode values (OPR_MODE) */
#define BNO055_OPR_MODE_CONFIG          0x00u  /* Must be in CONFIG to change settings */
#define BNO055_OPR_MODE_ACCONLY         0x01u
#define BNO055_OPR_MODE_MAGONLY         0x02u
#define BNO055_OPR_MODE_GYROONLY        0x03u
#define BNO055_OPR_MODE_ACCMAG          0x04u
#define BNO055_OPR_MODE_ACCGYRO         0x05u
#define BNO055_OPR_MODE_MAGGYRO         0x06u
#define BNO055_OPR_MODE_AMG             0x07u  /* Accel + Mag + Gyro, no fusion */
#define BNO055_OPR_MODE_IMU             0x08u  /* Fusion: accel + gyro */
#define BNO055_OPR_MODE_NDOF            0x0Cu  /* Full 9-DOF fusion */

/* Data output registers */
#define BNO055_REG_ACC_DATA_X_LSB       0x08u  /* Burst: 6 bytes accel */
#define BNO055_REG_MAG_DATA_X_LSB       0x0Eu  /* Burst: 6 bytes mag   */
#define BNO055_REG_GYR_DATA_X_LSB       0x14u  /* Burst: 6 bytes gyro  */
#define BNO055_REG_EUL_DATA_X_LSB       0x1Au  /* Burst: 6 bytes Euler */
#define BNO055_REG_QUA_DATA_W_LSB       0x20u  /* Burst: 8 bytes quat  */
#define BNO055_REG_LIA_DATA_X_LSB       0x28u  /* Linear accel (gravity removed) */
#define BNO055_REG_GRV_DATA_X_LSB       0x2Eu  /* Gravity vector */
#define BNO055_REG_TEMP                 0x34u

/* Sensitivity — AMG mode, default full-scale */
#define BNO055_ACCEL_LSB_PER_MPS2       100.0f   /* 100 LSB/(m/s²) */
#define BNO055_GYRO_LSB_PER_DPS         16.0f    /* 16 LSB/(°/s)   */
#define BNO055_MAG_LSB_PER_UT           16.0f    /* 16 LSB/µT      */


/* =========================================================================
 * Xilinx AXI IIC core register offsets
 * (memory-mapped at the physical base address supplied to mmap)
 * ========================================================================= */

#define XIIC_RESETR_OFFSET              0x40u
#define XIIC_CR_REG_OFFSET              0x100u
#define XIIC_SR_REG_OFFSET              0x104u
#define XIIC_TX_FIFO_REG_OFFSET         0x108u
#define XIIC_RX_FIFO_REG_OFFSET         0x10Cu
#define XIIC_DRR_REG_OFFSET             0x10Cu  /* Alias for RX FIFO drain register */
#define XIIC_RFD_REG_OFFSET             0x120u  /* RX FIFO depth register */

/* CR bit fields */
#define XIIC_CR_ENABLE_DEVICE_MASK      0x01u
#define XIIC_CR_TX_FIFO_RESET_MASK      0x02u
#define XIIC_CR_MSMS_MASK               0x04u  /* Master/slave mode select */
#define XIIC_CR_TX_MASK                 0x08u  /* Transmit enable */
#define XIIC_CR_TXAK_MASK               0x10u  /* Transmit ACK */

/* SR bit fields */
#define XIIC_SR_ABGC_MASK               0x01u  /* Addressed by general call */
#define XIIC_SR_AAS_MASK                0x02u  /* Addressed as slave */
#define XIIC_SR_BUS_BUSY_MASK           0x04u
#define XIIC_SR_SRW_MASK                0x08u  /* Slave read/write */
#define XIIC_SR_TX_FIFO_FULL_MASK       0x10u
#define XIIC_SR_RX_FIFO_FULL_MASK       0x20u
#define XIIC_SR_RX_FIFO_EMPTY_MASK      0x40u
#define XIIC_SR_TX_FIFO_EMPTY_MASK      0x80u

/* Software reset value */
#define XIIC_RESET_MASK                 0x0Au

#ifdef __cplusplus
}
#endif

#endif /* IMU_REGISTERS_H */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "rp.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include "vqf.h"

/*######### IIC #########*/

#define AXI_IIC1_ADDRESS 0x41600000 // Adress of axi_iic_0
#define IIC_CR_OFFSET 0x100
#define IIC_SR_OFFSET 0x104
#define IIC_TX_FIFO_OFFSET 0x108
#define IIC_RX_FIFO_OFFSET 0x10C
#define IIC_RX_FIFO_PIRQ_OFFSET 0x120
#define RANGE 64000

// axi_iic status register bits
#define SR_BUS_BUSY (1 << 2)
#define SR_RX_FIFO_EMPTY (1 << 6)
#define SR_TX_FIFO_FULL (1 << 4)
#define SR_TX_FIFO_EMPTY (1 << 7)

//sensor specific constants:

//MPU6050
#define mpu_addr 0x68           // I2C address when AD0 = LOW
#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_WHO_AM_I 0x68
#define power_management 0x6B
#define mpu_ACC 0x3B
#define mpu_CONFIG 0x1A
#define USER_CTRL 0x6A
#define INT_PIN_CFG 0x37

//MPU9265/9250 family (same register layout as MPU6050; current wiring uses AD0 = HIGH)
#define MPU9265_ADDR 0x69
#define MPU9265_WHO_AM_I_REG 0x75
#define MPU9265_WHO_AM_I_9265 0x71
#define MPU9265_WHO_AM_I_6500 0x70  // MPU6500 die found in some MPU9265 modules

//BNO055
#define bno055_addr 0x28           // 00101000 << 1 => 01010000
#define BNO055_OPR_MODE 0x3D
#define BNO055_ACC 0x08
#define BNO055_PWR_MODE 0x3E
#define BNO055_PAGE_ID 0x07



/*######### SPI #########*/

#define AXI_SPI1_ADDRESS 0x41E00000

// (relevant) axi_spi control register bits
#define SPI_CR_MASTER_TX_INHIBIT (1 << 8)
#define SPI_CR_MANUAL_SS_ENABLE (1 << 7)
#define SPI_CR_RX_FIFO_RESET (1 << 6)
#define SPI_CR_TX_FIFO_RESET (1 << 5)
#define SPI_CR_MASTER_MODE (1 << 2)
#define SPI_CR_ENABLE (1 << 1)
#define SPI_CR_LOOPBACK_MODE (1)

// axi_spi control register (relevant) bits
#define SPI_SR_TX_FULL (1 << 3)
#define SPI_SR_TX_EMPTY (1 << 2)
#define SPI_SR_RX_FULL (1 << 1)
#define SPI_SR_RX_EMPTY (1)

//sensor specific constants:

//ICM20948 (on SPI; ICM_ADDR kept only for reference / legacy I2C init)
#define ICM_ADDR 0x68
#define ICM_WHO_AM_I_REG 0x00
#define ICM_WHO_AM_I_20948 0xEA
#define ICM_WHO_AM_I_20649 0xE1
#define ICM_READ_BIT (1 << 7)
#define REG_BANK_SEL 0x7F
#define ICM_PWR_MGMT_1 0x06
#define ICM_PWR_MGMT_2 0x07
#define ICM_USER_CTRL 0x03
#define ICM_INT_PIN_CFG 0x0F
#define ICM_ACCEL_OUT 0x2D
#define ICM_ACC ICM_ACCEL_OUT


#define g 9.80665
#define ACC_SENSITIVITY_ICM20948 16384
#define ACCEL_SENSITIVITY 16384
#define ACC_SENSITIVITY_ICM20649 8192
#define MAG_SENSITIVITY 16
#define GYRO_SENSITIVITY 131
#define GYRO_SENSITIVITY_ICM20948 131
#define GYRO_SENSITIVITY_ICM20649 65.5



/*######### Timer #########*/

#define TIMER_GPIO_ADDRESS 0x41200000



/*######### Sine wave #########*/

#define SINE_GPIO_ADDRESS 0x41200000



//######### IIC FUNCTIONS ##########

void axi_iic_initialize(void *axi_iic_map) {
    volatile uint32_t *iic_cr = (volatile uint32_t *)(axi_iic_map + IIC_CR_OFFSET);
    volatile uint32_t *iic_sr = (volatile uint32_t *)(axi_iic_map + IIC_SR_OFFSET);
    volatile uint32_t *iic_rx_fifo_pirq = (volatile uint32_t *)(axi_iic_map + IIC_RX_FIFO_PIRQ_OFFSET);
    const uint32_t timeout_iters = 1000000;
    uint32_t guard = 0;
    *iic_rx_fifo_pirq = 0x0F; // Initialize the RX FIFO PIRQ register
    
    for (guard = 0; (*iic_sr & SR_BUS_BUSY) && guard < timeout_iters; guard++);
    if (*iic_sr & SR_BUS_BUSY) {
        printf("axi_iic_initialize: timeout waiting BUS_BUSY clear, SR=0x%08X\n", *iic_sr);
        fflush(stdout);
        return;
    }
    for (guard = 0; ((*iic_sr & SR_RX_FIFO_EMPTY) == 0) && guard < timeout_iters; guard++);
    if ((*iic_sr & SR_RX_FIFO_EMPTY) == 0) {
        printf("axi_iic_initialize: timeout waiting RX_FIFO_EMPTY set, SR=0x%08X\n", *iic_sr);
        fflush(stdout);
        return;
    }
    for (guard = 0; ((*iic_sr & SR_TX_FIFO_EMPTY) == 0) && guard < timeout_iters; guard++);
    if ((*iic_sr & SR_TX_FIFO_EMPTY) == 0) {
        printf("axi_iic_initialize: timeout waiting TX_FIFO_EMPTY set, SR=0x%08X\n", *iic_sr);
        fflush(stdout);
        return;
    }

    *iic_cr = 0x02; // Reset control register
    usleep(5); // Wait for a short time to ensure the reset is complete
    *iic_cr = 0x01; // enable control register

    return;

}


void axi_iic_write_byte(void *axi_iic_map, uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    volatile uint32_t *iic_sr = (volatile uint32_t *)(axi_iic_map + IIC_SR_OFFSET);
    volatile uint32_t *iic_tx_fifo = (volatile uint32_t *)(axi_iic_map + IIC_TX_FIFO_OFFSET);

    while (*iic_sr & SR_BUS_BUSY); // Wait until the bus is free
    while ((*iic_sr & SR_RX_FIFO_EMPTY) == 0);
    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0); // Ensure FIFO registers are empty

    *iic_tx_fifo = 0x100 | (device_addr << 1) | 0; // Write device address with write bit
    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0);
    
    *iic_tx_fifo = reg_addr; // Write register address
    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0);
    
    *iic_tx_fifo = 0x200 | data; // Write data + stop bit
    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0);

    return;
}

void axi_iic_write2_bytes(void *axi_iic_map, uint8_t device_addr, uint8_t reg_addr, uint8_t data1, uint8_t data2) {
    volatile uint32_t *iic_sr = (volatile uint32_t *)(axi_iic_map + IIC_SR_OFFSET);
    volatile uint32_t *iic_tx_fifo = (volatile uint32_t *)(axi_iic_map + IIC_TX_FIFO_OFFSET);

    while (*iic_sr & SR_BUS_BUSY); // Wait until the bus is free
    while ((*iic_sr & SR_RX_FIFO_EMPTY) == 0);
    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0); // Ensure FIFO registers are empty

    *iic_tx_fifo = 0x100 | (device_addr << 1) | 0; // Write device address with write bit
    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0);
    
    *iic_tx_fifo = reg_addr; // Write register address
    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0);
    
    *iic_tx_fifo = data1; // Write first data byte
    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0);
    
    *iic_tx_fifo = 0x200 | data2; // Write second data byte + stop bit
    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0);

    return;
}

void axi_iic_write_n_bytes(void *axi_iic_map, uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t data_length) {
    volatile uint32_t *iic_sr = (volatile uint32_t *)(axi_iic_map + IIC_SR_OFFSET);
    volatile uint32_t *iic_tx_fifo = (volatile uint32_t *)(axi_iic_map + IIC_TX_FIFO_OFFSET);

    while (*iic_sr & SR_BUS_BUSY); // Wait until the bus is free
    while ((*iic_sr & SR_RX_FIFO_EMPTY) == 0);
    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0); // Ensure FIFO registers are empty

    *iic_tx_fifo = 0x100 | (device_addr << 1) | 0; // Write device address with write bit

    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0);
    *iic_tx_fifo = reg_addr; // Write register address

    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0);

    for (size_t i = 0; i < data_length-1; i++) {
        *iic_tx_fifo = data[i]; // Write data byte
        while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0);
    }
    
    *iic_tx_fifo = 0x200 | data[data_length-1];  // Write last data byte + stop bit
    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0);

    return;
}

void axi_iic_read_n_bytes(void *axi_iic_map, uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t data_length) {
    volatile uint32_t *iic_sr = (volatile uint32_t *)(axi_iic_map + IIC_SR_OFFSET);
    volatile uint32_t *iic_tx_fifo = (volatile uint32_t *)(axi_iic_map + IIC_TX_FIFO_OFFSET);
    volatile uint32_t *iic_rx_fifo = (volatile uint32_t *)(axi_iic_map + IIC_RX_FIFO_OFFSET);

    while (*iic_sr & SR_BUS_BUSY); // Wait until the bus is free
    while ((*iic_sr & SR_RX_FIFO_EMPTY) == 0);
    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0); // Ensure FIFO registers are empty

    *iic_tx_fifo = 0x100 | (device_addr << 1) | 0; // Write device address with write bit

    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0); 
    *iic_tx_fifo = reg_addr; // Write register address

    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0); 
    *iic_tx_fifo = 0x100 | (device_addr << 1) | 1; // Repeated start + read bit

    while ((*iic_sr & SR_TX_FIFO_EMPTY) == 0);
    *iic_tx_fifo = 0x0200 | data_length; // Set the number of bytes to read + stop bit

    for (size_t i = 0; i < data_length; i++) {
        while ((*iic_sr & SR_RX_FIFO_EMPTY) != 0); // Wait for data to be available
        data[i] = *iic_rx_fifo; // Read data byte
    }

    return;
}

static inline int axi_iic_wait_while_set(volatile uint32_t* reg, uint32_t mask, uint32_t timeout_iters) {
    while ((*reg & mask) != 0) {
        if (timeout_iters-- == 0) return -1;
    }
    return 0;
}

static inline int axi_iic_wait_while_clear(volatile uint32_t* reg, uint32_t mask, uint32_t timeout_iters) {
    while ((*reg & mask) == 0) {
        if (timeout_iters-- == 0) return -1;
    }
    return 0;
}

static inline int axi_iic_drain_rx_fifo_timeout(volatile uint32_t* iic_sr, volatile uint32_t* iic_rx_fifo,
                                                uint32_t timeout_iters) {
    while ((*iic_sr & SR_RX_FIFO_EMPTY) == 0) {
        (void)*iic_rx_fifo;
        if (timeout_iters-- == 0) return -1;
    }
    return 0;
}

static inline int axi_iic_write_byte_timeout(void *axi_iic_map, uint8_t device_addr, uint8_t reg_addr, uint8_t data,
                                             uint32_t timeout_iters) {
    volatile uint32_t *iic_sr = (volatile uint32_t *)(axi_iic_map + IIC_SR_OFFSET);
    volatile uint32_t *iic_tx_fifo = (volatile uint32_t *)(axi_iic_map + IIC_TX_FIFO_OFFSET);
    volatile uint32_t *iic_rx_fifo = (volatile uint32_t *)(axi_iic_map + IIC_RX_FIFO_OFFSET);

    if (axi_iic_wait_while_set(iic_sr, SR_BUS_BUSY, timeout_iters) < 0) return -1;
    if (axi_iic_drain_rx_fifo_timeout(iic_sr, iic_rx_fifo, timeout_iters) < 0) return -1;
    if (axi_iic_wait_while_clear(iic_sr, SR_TX_FIFO_EMPTY, timeout_iters) < 0) return -1;

    *iic_tx_fifo = 0x100 | (device_addr << 1) | 0;
    if (axi_iic_wait_while_clear(iic_sr, SR_TX_FIFO_EMPTY, timeout_iters) < 0) return -1;

    *iic_tx_fifo = reg_addr;
    if (axi_iic_wait_while_clear(iic_sr, SR_TX_FIFO_EMPTY, timeout_iters) < 0) return -1;

    *iic_tx_fifo = 0x200 | data;
    if (axi_iic_wait_while_clear(iic_sr, SR_TX_FIFO_EMPTY, timeout_iters) < 0) return -1;

    return 0;
}

static inline int axi_iic_read_n_bytes_timeout(void *axi_iic_map, uint8_t device_addr, uint8_t reg_addr,
                                               uint8_t *data, size_t data_length, uint32_t timeout_iters) {
    volatile uint32_t *iic_sr = (volatile uint32_t *)(axi_iic_map + IIC_SR_OFFSET);
    volatile uint32_t *iic_tx_fifo = (volatile uint32_t *)(axi_iic_map + IIC_TX_FIFO_OFFSET);
    volatile uint32_t *iic_rx_fifo = (volatile uint32_t *)(axi_iic_map + IIC_RX_FIFO_OFFSET);

    if (data == NULL || data_length == 0) return -1;

    if (axi_iic_wait_while_set(iic_sr, SR_BUS_BUSY, timeout_iters) < 0) return -1;
    if (axi_iic_drain_rx_fifo_timeout(iic_sr, iic_rx_fifo, timeout_iters) < 0) return -1;
    if (axi_iic_wait_while_clear(iic_sr, SR_TX_FIFO_EMPTY, timeout_iters) < 0) return -1;

    *iic_tx_fifo = 0x100 | (device_addr << 1) | 0;
    if (axi_iic_wait_while_clear(iic_sr, SR_TX_FIFO_EMPTY, timeout_iters) < 0) return -1;

    *iic_tx_fifo = reg_addr;
    if (axi_iic_wait_while_clear(iic_sr, SR_TX_FIFO_EMPTY, timeout_iters) < 0) return -1;

    *iic_tx_fifo = 0x100 | (device_addr << 1) | 1;
    if (axi_iic_wait_while_clear(iic_sr, SR_TX_FIFO_EMPTY, timeout_iters) < 0) return -1;

    *iic_tx_fifo = 0x0200 | data_length;

    for (size_t i = 0; i < data_length; i++) {
        if (axi_iic_wait_while_set(iic_sr, SR_RX_FIFO_EMPTY, timeout_iters) < 0) return -1;
        data[i] = *iic_rx_fifo;
    }

    return 0;
}

void axi_iic_read_n_bytes_from_2_mpus(void *axi_iic_map_0, void *axi_iic_map_1, uint8_t device_addr, uint8_t reg_addr, 
    uint8_t *data_0, uint8_t *data_1, size_t data_length) {

    volatile uint32_t *iic_sr_0 = (volatile uint32_t *)(axi_iic_map_0 + IIC_SR_OFFSET);
    volatile uint32_t *iic_tx_fifo_0 = (volatile uint32_t *)(axi_iic_map_0 + IIC_TX_FIFO_OFFSET);
    volatile uint32_t *iic_rx_fifo_0 = (volatile uint32_t *)(axi_iic_map_0 + IIC_RX_FIFO_OFFSET);

    volatile uint32_t *iic_sr_1 = (volatile uint32_t *)(axi_iic_map_1 + IIC_SR_OFFSET);
    volatile uint32_t *iic_tx_fifo_1 = (volatile uint32_t *)(axi_iic_map_1 + IIC_TX_FIFO_OFFSET);
    volatile uint32_t *iic_rx_fifo_1 = (volatile uint32_t *)(axi_iic_map_1 + IIC_RX_FIFO_OFFSET);

    uint16_t MPU_INSTR[4] = {0x100 | (device_addr << 1) | 0, reg_addr, 0x100 | (device_addr << 1) | 1, 0x200 | data_length};

    int mpu_0_i = 0;
    int mpu_1_i = 0;

    int mpu_0_r = 0;
    int mpu_1_r = 0;

    // Check all buses are ready for transfer
    while (*iic_sr_0 & SR_BUS_BUSY); // Wait until the bus is free
    while ((*iic_sr_0 & SR_RX_FIFO_EMPTY) == 0);
    while ((*iic_sr_0 & SR_TX_FIFO_EMPTY) == 0); // Ensure FIFO registers are empty
    while (*iic_sr_1 & SR_BUS_BUSY); // Wait until the bus is free
    while ((*iic_sr_1 & SR_RX_FIFO_EMPTY) == 0);
    while ((*iic_sr_1 & SR_TX_FIFO_EMPTY) == 0); // Ensure FIFO registers are empty


    while((mpu_0_i < 4) || (mpu_1_i < 4)){
        if(((*iic_sr_0 & SR_TX_FIFO_FULL) == 0) && (mpu_0_i < 4)){
            *iic_tx_fifo_0 = MPU_INSTR[mpu_0_i];
            mpu_0_i++;
        }
        if(((*iic_sr_1 & SR_TX_FIFO_FULL) == 0) && (mpu_1_i < 4)){
            *iic_tx_fifo_1 = MPU_INSTR[mpu_1_i];
            mpu_1_i++;
        }
    }

    while((mpu_0_r < data_length) || (mpu_1_r < data_length)){

        if(((*iic_sr_0 & SR_RX_FIFO_EMPTY) == 0)){
            data_0[mpu_0_r] = *iic_rx_fifo_0;
            mpu_0_r++;
        }

        if(((*iic_sr_1 & SR_RX_FIFO_EMPTY) == 0)){
            data_1[mpu_1_r] = *iic_rx_fifo_1;
            mpu_1_r++;
        }

    }

    // *iic_tx_fifo_0 = 0x100 | (device_addr << 1) | 0; // Write device address with write bit

    // while ((*iic_sr_0 & SR_TX_FIFO_EMPTY) == 0); 
    // *iic_tx_fifo_0 = reg_addr; // Write register address

    // while ((*iic_sr_0 & SR_TX_FIFO_EMPTY) == 0); 
    // *iic_tx_fifo_0 = 0x100 | (device_addr << 1) | 1; // Repeated start + read bit

    // while ((*iic_sr_0 & SR_TX_FIFO_EMPTY) == 0);
    // *iic_tx_fifo_0 = 0x0200 | data_length; // Set the number of bytes to read + stop bit

    // for (size_t i = 0; i < data_length; i++) {
    //     while ((*iic_sr_0 & SR_RX_FIFO_EMPTY) != 0); // Wait for data to be available
    //     data_0[i] = *iic_rx_fifo_0; // Read data byte
    // }

    // *iic_tx_fifo_1 = 0x100 | (device_addr << 1) | 0; // Write device address with write bit

    // while ((*iic_sr_1 & SR_TX_FIFO_EMPTY) == 0); 
    // *iic_tx_fifo_1 = reg_addr; // Write register address

    // while ((*iic_sr_1 & SR_TX_FIFO_EMPTY) == 0); 
    // *iic_tx_fifo_1 = 0x100 | (device_addr << 1) | 1; // Repeated start + read bit

    // while ((*iic_sr_1 & SR_TX_FIFO_EMPTY) == 0);
    // *iic_tx_fifo_1 = 0x0200 | data_length; // Set the number of bytes to read + stop bit

    // for (size_t i = 0; i < data_length; i++) {
    //     while ((*iic_sr_1 & SR_RX_FIFO_EMPTY) != 0); // Wait for data to be available
    //     data_1[i] = *iic_rx_fifo_1; // Read data byte
    // }

    return;
}


//######### SPI FUNCTIONS ##########


// Note, these functions are currently only specified for a single SPI IMU unit, will have to pass the SS value
// if we have multiple SPI slaves on the SAME bus (doesn't matter for different buses)
void axi_spi_initialize(void *axi_spi_map) {
    volatile uint32_t *spi_reset_reg = (volatile uint32_t *) (axi_spi_map + 0x40);
    volatile uint32_t *spi_control_reg = (volatile uint32_t *) (axi_spi_map + 0x60);
    
    // Reset core before beginning
    *spi_reset_reg = 0xa;
    usleep(100);

    // Set CTL Register
    *spi_control_reg = SPI_CR_MANUAL_SS_ENABLE | SPI_CR_RX_FIFO_RESET | SPI_CR_TX_FIFO_RESET | SPI_CR_MASTER_MODE | SPI_CR_ENABLE;
    usleep(100);

    return;

}

// Could potentially be made more efficient later with burst writes and reads up to 16 bytes
void axi_spi_write(void *axi_spi_map, uint8_t write_addr, uint8_t write_data) {
    volatile uint32_t *spi_status_reg = (volatile uint32_t *) (axi_spi_map + 0x64);
    volatile uint8_t *spi_tx_fifo = (volatile uint8_t *) (axi_spi_map + 0x68);
    volatile uint8_t *spi_rx_fifo = (volatile uint8_t *) (axi_spi_map + 0x6C);
    volatile uint32_t *spi_slave_select_reg = (volatile uint32_t *) (axi_spi_map + 0x70);

    while(((*spi_status_reg) & SPI_SR_TX_FULL));
    *spi_tx_fifo = write_addr;

    while(((*spi_status_reg) & SPI_SR_TX_FULL));
    *spi_tx_fifo = write_data;
    *spi_slave_select_reg = 0xFFFFFFFE;

    while(((*spi_status_reg) & SPI_SR_RX_EMPTY));
    (void) *spi_rx_fifo;

    while(((*spi_status_reg) & SPI_SR_RX_EMPTY));
    (void) *spi_rx_fifo;

    *spi_slave_select_reg = 0xFFFFFFFF;
    return;
}

// Could potentially be made more efficient later with burst writes and reads up to 16 bytes
void axi_spi_read(void *axi_spi_map, uint8_t read_addr, uint8_t *buffer, int read_size) {
    volatile uint32_t *spi_status_reg = (volatile uint32_t *) (axi_spi_map + 0x64);
    volatile uint8_t *spi_tx_fifo = (volatile uint8_t *) (axi_spi_map + 0x68);
    volatile uint8_t *spi_rx_fifo = (volatile uint8_t *) (axi_spi_map + 0x6C);
    volatile uint32_t *spi_slave_select_reg = (volatile uint32_t *) (axi_spi_map + 0x70);

    // Write addr to tx if tx not full and enable slave
    while(((*spi_status_reg) & SPI_SR_TX_FULL));
    *spi_tx_fifo = (1 << 7) | read_addr;
    *spi_slave_select_reg = 0xFFFFFFFE;

    // Flush garbage value
    while(((*spi_status_reg) & SPI_SR_RX_EMPTY));
    //buffer[0] = *spi_rx_fifo;
    (void) *spi_rx_fifo;

    // Loop, while less than read size, write to tx and read from rx
    for(int i = 0; i < read_size; i++) {
        // Write garbage value
        while(((*spi_status_reg) & SPI_SR_TX_FULL));
        *spi_tx_fifo = 0;

        while(((*spi_status_reg) & SPI_SR_RX_EMPTY));
        buffer[i] = *spi_rx_fifo;
    }

    *spi_slave_select_reg = 0xFFFFFFFF;

    return;
}


// initialization for each sensor


//MPU6050:
void initialize_MPU5060(void *axi_iic_map){
    axi_iic_write2_bytes(axi_iic_map, mpu_addr, power_management, 0x01, 0x00);
    // DLPF=1 (188 Hz gyro bandwidth)
    axi_iic_write_byte(axi_iic_map, mpu_addr, mpu_CONFIG, 0x01);
    axi_iic_write_byte(axi_iic_map, mpu_addr, USER_CTRL, 0x00);
}

//MPU9265 (register-compatible with MPU6050; address 0x69 when AD0 = HIGH):
void initialize_MPU9265(void *axi_iic_map){
    axi_iic_write2_bytes(axi_iic_map, MPU9265_ADDR, power_management, 0x01, 0x00);
    axi_iic_write_byte(axi_iic_map, MPU9265_ADDR, mpu_CONFIG, 0x01);
    axi_iic_write_byte(axi_iic_map, MPU9265_ADDR, USER_CTRL, 0x00);
}

static inline int axi_iic_initialize_ICM20948(void *axi_iic_map) {
    uint8_t tmp = 0;
    uint8_t who_am_i = 0;
    const uint32_t timeout_iters = 100000;

    if (axi_iic_write_byte_timeout(axi_iic_map, ICM_ADDR, REG_BANK_SEL, 0x00, timeout_iters) < 0) return -1;
    usleep(10000);

    if (axi_iic_write_byte_timeout(axi_iic_map, ICM_ADDR, ICM_PWR_MGMT_1, 0x01, timeout_iters) < 0) return -1;
    usleep(10000);
    if (axi_iic_write_byte_timeout(axi_iic_map, ICM_ADDR, ICM_PWR_MGMT_2, 0x00, timeout_iters) < 0) return -1;
    usleep(10000);

    if (axi_iic_read_n_bytes_timeout(axi_iic_map, ICM_ADDR, ICM_USER_CTRL, &tmp, 1, timeout_iters) < 0) return -1;
    tmp &= (uint8_t)~(1u << 5);
    if (axi_iic_write_byte_timeout(axi_iic_map, ICM_ADDR, ICM_USER_CTRL, tmp, timeout_iters) < 0) return -1;

    if (axi_iic_read_n_bytes_timeout(axi_iic_map, ICM_ADDR, ICM_INT_PIN_CFG, &tmp, 1, timeout_iters) < 0) return -1;
    tmp |= 0x02;
    if (axi_iic_write_byte_timeout(axi_iic_map, ICM_ADDR, ICM_INT_PIN_CFG, tmp, timeout_iters) < 0) return -1;
    usleep(10000);

    if (axi_iic_read_n_bytes_timeout(axi_iic_map, ICM_ADDR, ICM_WHO_AM_I_REG, &who_am_i, 1, timeout_iters) < 0) return -1;
    return (who_am_i == ICM_WHO_AM_I_20948 || who_am_i == ICM_WHO_AM_I_20649) ? 0 : -1;
}



void axi_spi_initialize_ICM20948(void *axi_spi_map) {
    
    uint8_t read_device_power_mode_1 = (1 << 7) | 0x06;
    uint8_t read_device_power_mode_2 = (1 << 7) | 0x07;
    uint8_t write_device_power_mode_1 = 0x06;
    uint8_t write_device_power_mode_2 = 0x07;
    uint8_t power_reg_1_value;
    uint8_t power_reg_2_value;

    // Read from PWR_MGMT_1 Register
    axi_spi_read(axi_spi_map, read_device_power_mode_1, &power_reg_1_value, 1);
    //printf("Power Reg 1: 0x%X\n", power_reg_1_value);

    // If device is sleeping, turn it on
    if(power_reg_1_value & (1 << 6)) {
        //printf("Writing to Power Reg 1\n");
        axi_spi_write(axi_spi_map, write_device_power_mode_1, 0x1);
        usleep(1000000); // 1000 ms sleep (could be reduced potentially)
    }

    // Read from PWR_MGMT_2 Register (read twice to get correct register value)
    axi_spi_read(axi_spi_map, read_device_power_mode_2, &power_reg_2_value, 1);
    axi_spi_read(axi_spi_map, read_device_power_mode_2, &power_reg_2_value, 1);

    // printf("Power Reg 2: 0x%X\n", power_reg_2_value);
    // axi_spi_read(axi_spi_map, read_device_power_mode_2, &power_reg_2_value, 1);
    // printf("Power Reg 2: 0x%X\n", power_reg_2_value);
    // axi_spi_read(axi_spi_map, read_device_power_mode_2, &power_reg_2_value, 1);
    // printf("Power Reg 2: 0x%X\n", power_reg_2_value);

    // If the accelerometer or gyroscope are off, turn them on
    if((power_reg_2_value & 0x3F) != 0) {
        axi_spi_write(axi_spi_map, write_device_power_mode_2, 0x0);
    }

    // axi_spi_read(axi_spi_map, read_device_power_mode_1, &power_reg_1_value, 1);
    // printf("Power Reg 1: 0x%X\n", power_reg_1_value);
    // axi_spi_read(axi_spi_map, read_device_power_mode_2, &power_reg_2_value, 1);
    // printf("Power Reg 2: 0x%X\n", power_reg_2_value);

}


/*============================================================================
 * MULTI-SENSOR ACQUISITION LIBRARY
 * High-level functions for sensor data acquisition, file I/O, and streaming
 *===========================================================================*/

#include <stdbool.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

/*====== Data Structures ======*/

/**
 * @brief Operating mode for acquisition system
 */
typedef enum {
    MODE_LOG_ONLY = 0,
    MODE_LOG_AND_STREAM = 1,
    MODE_UNKNOWN = -1
} acquisition_mode_t;

/**
 * @brief Configuration parameters for acquisition
 */
typedef struct {
    uint32_t sample_rate_hz;
    uint32_t test_duration_sec;
    uint32_t file_write_interval_sec;
    bool enable_streaming;
    uint16_t stream_port;
    uint32_t stream_rate_hz;
    uint32_t timer_clock_rate;
    char mpu_output_file[256];
    char icm_output_file[256];
    char mpu9265_output_file[256];
    char mpu_csv_file[256];
    char icm_csv_file[256];
    char mpu9265_csv_file[256];
} acquisition_config_t;

/**
 * @brief MPU6050 sensor sample (14 bytes: 6 accel + 2 temp + 6 gyro)
 */
typedef struct {
    uint8_t raw_data[14];
} mpu6050_sample_t;

/**
 * @brief MPU9265 sensor sample — same layout as MPU6050 (14 bytes)
 */
typedef struct {
    uint8_t raw_data[14];
} mpu9265_sample_t;

/**
 * @brief ICM20948 sensor sample (12 bytes: 6 accel + 6 gyro)
 */
typedef struct {
    uint8_t raw_data[12];
} icm20649_sample_t;

/**
 * @brief Timer context for 64-bit overflow-safe timing
 * Red Pitaya 32-bit timer overflows every ~34 seconds at 125MHz
 */
typedef struct {
    uint64_t cumulative_ticks;
    uint32_t last_raw_value;
    uint64_t start_ticks;
} timer_context_t;

/**
 * @brief Sensor interface context
 */
typedef struct {
    void* iic_map;
    void* spi_map;
    bool icm_uses_i2c;
    volatile uint32_t* iic_sr;
    volatile uint32_t* iic_tx;
    volatile uint32_t* iic_rx;
    volatile uint32_t* spi_sr;
    volatile uint8_t* spi_tx;
    volatile uint8_t* spi_rx;
    volatile uint32_t* spi_ss;
} sensor_context_t;

/**
 * @brief File I/O context with double-buffered async writes for three sensors
 */
typedef struct {
    int fd_mpu, fd_icm, fd_mpu9265;
    uint8_t* mpu_buffers[2];
    uint8_t* icm_buffers[2];
    uint8_t* mpu9265_buffers[2];
    size_t mpu_buffer_size, icm_buffer_size, mpu9265_buffer_size;
    size_t mpu_offsets[2], icm_offsets[2], mpu9265_offsets[2];
    int mpu_active_idx, icm_active_idx, mpu9265_active_idx;
    int mpu_pending_idx, icm_pending_idx, mpu9265_pending_idx;
    bool mpu_pending_ready, icm_pending_ready, mpu9265_pending_ready;
    bool writer_stop;
    int writer_error;
    pthread_t writer_thread;
    pthread_mutex_t writer_mutex;
    pthread_cond_t writer_cond;
    pthread_cond_t space_cond;
} file_context_t;

// Sensor index constants for file_queue_full_buffer
#define FILE_SENSOR_MPU    0
#define FILE_SENSOR_ICM    1
#define FILE_SENSOR_MPU9265 2

// Packet layout (31 channels × int16 = 62 bytes payload + 22-byte header = 84 bytes total):
// ch[ 0: 6]  MPU6050  ax,ay,az,gx,gy,gz
// ch[ 6:10]  MPU6050  qw,qx,qy,qz
// ch[10:16]  MPU9265  ax,ay,az,gx,gy,gz
// ch[16:20]  MPU9265  qw,qx,qy,qz
// ch[20:26]  ICM20948 ax,ay,az,gx,gy,gz
// ch[26:30]  ICM20948 qw,qx,qy,qz
// ch[30]     joint angle × 100
#define STREAM_NUM_CHANNELS 31
#define STREAM_PACKET_SIZE  84   // 22 header + 31*2 payload

/**
 * @brief Network streaming context
 */
typedef struct {
    int server_fd, client_fd;
    uint16_t port;
    uint32_t stream_rate_hz, sample_rate_hz;
    uint32_t decimation, stream_counter;
    uint8_t header[22], packet[STREAM_PACKET_SIZE];
    uint32_t packets_sent;
} stream_context_t;


/*====== MODULE 1: Timer Management ======*/

/**
 * @brief Initialize timer and reset hardware counter
 */
static inline void timer_init(timer_context_t* ctx, volatile uint32_t* timer_gpio_map) {
    *(timer_gpio_map + 0x8) = 0x00000001;
    usleep(1);
    *(timer_gpio_map + 0x8) = 0x00000000;
    ctx->last_raw_value = *(timer_gpio_map + 0x0);
    ctx->cumulative_ticks = 0;
    ctx->start_ticks = ctx->last_raw_value;
}

/**
 * @brief Get current 64-bit tick count (overflow-safe)
 */
static inline uint64_t timer_get_ticks(timer_context_t* ctx, volatile uint32_t* timer_gpio_map) {
    uint32_t current_raw = *(timer_gpio_map + 0x0);
    if (current_raw < ctx->last_raw_value) {
        ctx->cumulative_ticks += ((uint64_t)1 << 32);
    }
    ctx->last_raw_value = current_raw;
    return ctx->cumulative_ticks + (uint64_t)current_raw;
}

/**
 * @brief Calculate actual sample rate from tick count
 */
static inline uint32_t timer_calc_frequency(uint64_t total_ticks, uint64_t total_samples, uint32_t clock_rate) {
    if (total_ticks == 0) return 0;
    double seconds = (double)total_ticks / (double)clock_rate;
    if (seconds <= 0.0) return 0;
    return (uint32_t)((double)total_samples / seconds);
}


/*====== MODULE 2: Sensor I/O ======*/

/**
 * @brief Initialize sensor interface context with register pointers
 */
static inline void sensor_init_context(sensor_context_t* ctx, void* iic_map, void* spi_map) {
    ctx->iic_map = iic_map;
    ctx->spi_map = spi_map;
    ctx->icm_uses_i2c = false;
    ctx->iic_sr = (volatile uint32_t*)(iic_map + IIC_SR_OFFSET);
    ctx->iic_tx = (volatile uint32_t*)(iic_map + IIC_TX_FIFO_OFFSET);
    ctx->iic_rx = (volatile uint32_t*)(iic_map + IIC_RX_FIFO_OFFSET);
    ctx->spi_sr = (volatile uint32_t*)(spi_map + 0x64);
    ctx->spi_tx = (volatile uint8_t*)(spi_map + 0x68);
    ctx->spi_rx = (volatile uint8_t*)(spi_map + 0x6C);
    ctx->spi_ss = (volatile uint32_t*)(spi_map + 0x70);
}

/**
 * @brief Read MPU6050 only with timeout.
 * @return 0 on success, -1 on timeout
 */
static inline int sensor_read_mpu_timed(sensor_context_t* ctx, mpu6050_sample_t* mpu_out,
                                        timer_context_t* timer_ctx, volatile uint32_t* timer_gpio_map,
                                        uint64_t timeout_ticks) {
    static const uint16_t MPU6050_instr[4] = {
        (0x100 | (mpu_addr << 1) | 0), mpu_ACC,
        (0x100 | (mpu_addr << 1) | 1), (0x0200 | 0xE)
    };

    int instr_idx = 0;
    int read_idx = 0;
    uint64_t start_tick = 0;

    if (timeout_ticks > 0 && timer_ctx != NULL && timer_gpio_map != NULL) {
        start_tick = timer_get_ticks(timer_ctx, timer_gpio_map);
    }

    while (read_idx < 14) {
        if (timeout_ticks > 0 && timer_ctx != NULL && timer_gpio_map != NULL) {
            uint64_t elapsed = timer_get_ticks(timer_ctx, timer_gpio_map) - start_tick;
            if (elapsed >= timeout_ticks) {
                return -1;
            }
        }

        if ((instr_idx < 4) && (!(*ctx->iic_sr & SR_TX_FIFO_FULL))) {
            if (instr_idx == 0) {
                if (*ctx->iic_sr & SR_BUS_BUSY) {
                    continue;
                }
                if ((*ctx->iic_sr & SR_RX_FIFO_EMPTY) == 0) {
                    (void)*ctx->iic_rx;
                    continue;
                }
                if ((*ctx->iic_sr & SR_TX_FIFO_EMPTY) == 0) {
                    continue;
                }
            }
            *ctx->iic_tx = MPU6050_instr[instr_idx];
            instr_idx++;
        } else if ((read_idx < 14) && (!(*ctx->iic_sr & SR_RX_FIFO_EMPTY))) {
            mpu_out->raw_data[read_idx] = *ctx->iic_rx;
            read_idx++;
        }
    }

    return 0;
}

/**
 * @brief Read ICM20948 accel+gyro sample over SPI.
 * SPI is synchronous; timeout parameters are accepted but ignored.
 * @return 0 on success
 */
static inline int sensor_read_icm_timed(sensor_context_t* ctx, icm20649_sample_t* icm_out,
                                        timer_context_t* timer_ctx, volatile uint32_t* timer_gpio_map,
                                        uint64_t timeout_ticks) {
    if (ctx->icm_uses_i2c) {
        uint32_t timeout_iters = 100000;

        if (timeout_ticks > 0 && timer_ctx != NULL && timer_gpio_map != NULL) {
            uint64_t start_tick = timer_get_ticks(timer_ctx, timer_gpio_map);
            while (1) {
                if (axi_iic_write_byte_timeout(ctx->iic_map, ICM_ADDR, REG_BANK_SEL, 0x00, timeout_iters) == 0 &&
                    axi_iic_read_n_bytes_timeout(ctx->iic_map, ICM_ADDR, ICM_ACCEL_OUT,
                                                 icm_out->raw_data, sizeof(icm_out->raw_data), timeout_iters) == 0) {
                    return 0;
                }
                if ((timer_get_ticks(timer_ctx, timer_gpio_map) - start_tick) >= timeout_ticks) {
                    return -1;
                }
                usleep(1000);
            }
        }

        if (axi_iic_write_byte_timeout(ctx->iic_map, ICM_ADDR, REG_BANK_SEL, 0x00, timeout_iters) < 0) return -1;
        return axi_iic_read_n_bytes_timeout(ctx->iic_map, ICM_ADDR, ICM_ACCEL_OUT,
                                            icm_out->raw_data, sizeof(icm_out->raw_data), timeout_iters);
    }

    (void)timer_ctx;
    (void)timer_gpio_map;
    (void)timeout_ticks;
    axi_spi_read(ctx->spi_map, ICM_ACCEL_OUT, icm_out->raw_data, sizeof(icm_out->raw_data));
    return 0;
}

/**
 * @brief Read MPU9265 accel+gyro sample over I2C (address 0x69, same registers as MPU6050).
 * @return 0 on success, -1 on timeout
 */
static inline int sensor_read_mpu9265_timed(sensor_context_t* ctx, mpu9265_sample_t* mpu9265_out,
                                            timer_context_t* timer_ctx, volatile uint32_t* timer_gpio_map,
                                            uint64_t timeout_ticks) {
    static const uint16_t MPU9265_instr[4] = {
        (0x100 | (MPU9265_ADDR << 1) | 0), mpu_ACC,
        (0x100 | (MPU9265_ADDR << 1) | 1), (0x0200 | 0xE)
    };

    int instr_idx = 0;
    int read_idx = 0;
    uint64_t start_tick = 0;

    if (timeout_ticks > 0 && timer_ctx != NULL && timer_gpio_map != NULL)
        start_tick = timer_get_ticks(timer_ctx, timer_gpio_map);

    while (read_idx < 14) {
        if (timeout_ticks > 0 && timer_ctx != NULL && timer_gpio_map != NULL) {
            if ((timer_get_ticks(timer_ctx, timer_gpio_map) - start_tick) >= timeout_ticks) return -1;
        }
        if ((instr_idx < 4) && (!(*ctx->iic_sr & SR_TX_FIFO_FULL))) {
            if (instr_idx == 0) {
                if (*ctx->iic_sr & SR_BUS_BUSY) continue;
                if ((*ctx->iic_sr & SR_RX_FIFO_EMPTY) == 0) { (void)*ctx->iic_rx; continue; }
                if ((*ctx->iic_sr & SR_TX_FIFO_EMPTY) == 0) continue;
            }
            *ctx->iic_tx = MPU9265_instr[instr_idx];
            instr_idx++;
        } else if ((read_idx < 14) && (!(*ctx->iic_sr & SR_RX_FIFO_EMPTY))) {
            mpu9265_out->raw_data[read_idx] = *ctx->iic_rx;
            read_idx++;
        }
    }
    return 0;
}

/**
 * @brief Read both MPU6050 and ICM20948 sensors with optional timeout.
 * The reads are sequential on the shared I2C helper path to avoid mixing legacy SPI behavior.
 * @return 0 on success, -1 on timeout
 */
static inline int sensor_read_both_timed(sensor_context_t* ctx, mpu6050_sample_t* mpu_out,
                                         icm20649_sample_t* icm_out, timer_context_t* timer_ctx,
                                         volatile uint32_t* timer_gpio_map, uint64_t timeout_ticks) {
    if (sensor_read_mpu_timed(ctx, mpu_out, timer_ctx, timer_gpio_map, timeout_ticks) < 0) {
        return -1;
    }
    return sensor_read_icm_timed(ctx, icm_out, timer_ctx, timer_gpio_map, timeout_ticks);
}

/**
 * @brief Read both MPU6050 and ICM20948 sensors
 * Uses parallel I2C and SPI state machines for efficient acquisition
 */
static inline void sensor_read_both(sensor_context_t* ctx, mpu6050_sample_t* mpu_out, icm20649_sample_t* icm_out) {
    (void)sensor_read_both_timed(ctx, mpu_out, icm_out, NULL, NULL, 0);
}


/*====== MODULE 3: File I/O ======*/

static inline int file_write_all(int fd, const uint8_t* data, size_t len) {
    size_t written = 0;
    while (written < len) {
        ssize_t n = write(fd, data + written, len - written);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (n == 0) return -1;
        written += (size_t)n;
    }
    return 0;
}

static inline int file_queue_full_buffer(file_context_t* ctx, int sensor) {
    int* pending_idx;
    bool* pending_ready;
    int* active_idx;
    size_t* offsets;
    const char* label;
    if (sensor == FILE_SENSOR_MPU) {
        pending_idx = &ctx->mpu_pending_idx;   pending_ready = &ctx->mpu_pending_ready;
        active_idx  = &ctx->mpu_active_idx;    offsets = ctx->mpu_offsets;   label = "mpu";
    } else if (sensor == FILE_SENSOR_ICM) {
        pending_idx = &ctx->icm_pending_idx;   pending_ready = &ctx->icm_pending_ready;
        active_idx  = &ctx->icm_active_idx;    offsets = ctx->icm_offsets;   label = "icm";
    } else {
        pending_idx = &ctx->mpu9265_pending_idx; pending_ready = &ctx->mpu9265_pending_ready;
        active_idx  = &ctx->mpu9265_active_idx;  offsets = ctx->mpu9265_offsets; label = "mpu9265";
    }

    while (*pending_idx != -1 && ctx->writer_error == 0) {
        pthread_cond_wait(&ctx->space_cond, &ctx->writer_mutex);
    }
    if (ctx->writer_error != 0) return -1;

    *pending_idx = *active_idx;
    *pending_ready = true;
    *active_idx = 1 - *active_idx;
    offsets[*active_idx] = 0;

    pthread_cond_signal(&ctx->writer_cond);
    printf("  [DISK-QUEUE] %s queued %zu bytes\n", label, offsets[*pending_idx]);
    return 0;
}

static inline void* file_writer_thread_main(void* arg) {
    file_context_t* ctx = (file_context_t*)arg;
    while (1) {
        int mpu_idx = -1, icm_idx = -1, mpu9265_idx = -1;
        size_t mpu_len = 0, icm_len = 0, mpu9265_len = 0;

        pthread_mutex_lock(&ctx->writer_mutex);
        while (!ctx->writer_stop && !ctx->mpu_pending_ready &&
               !ctx->icm_pending_ready && !ctx->mpu9265_pending_ready) {
            pthread_cond_wait(&ctx->writer_cond, &ctx->writer_mutex);
        }
        if (ctx->mpu_pending_ready) {
            mpu_idx = ctx->mpu_pending_idx;
            mpu_len = (mpu_idx >= 0) ? ctx->mpu_offsets[mpu_idx] : 0;
            ctx->mpu_pending_ready = false;
        }
        if (ctx->icm_pending_ready) {
            icm_idx = ctx->icm_pending_idx;
            icm_len = (icm_idx >= 0) ? ctx->icm_offsets[icm_idx] : 0;
            ctx->icm_pending_ready = false;
        }
        if (ctx->mpu9265_pending_ready) {
            mpu9265_idx = ctx->mpu9265_pending_idx;
            mpu9265_len = (mpu9265_idx >= 0) ? ctx->mpu9265_offsets[mpu9265_idx] : 0;
            ctx->mpu9265_pending_ready = false;
        }
        if (ctx->writer_stop && mpu_idx == -1 && icm_idx == -1 && mpu9265_idx == -1) {
            pthread_mutex_unlock(&ctx->writer_mutex);
            break;
        }
        pthread_mutex_unlock(&ctx->writer_mutex);

        if (mpu_idx >= 0 && mpu_len > 0) {
            if (file_write_all(ctx->fd_mpu, ctx->mpu_buffers[mpu_idx], mpu_len) < 0) {
                pthread_mutex_lock(&ctx->writer_mutex);
                ctx->writer_error = -1;
                pthread_cond_broadcast(&ctx->space_cond);
                pthread_mutex_unlock(&ctx->writer_mutex);
                perror("write mpu"); break;
            }
            printf("  [DISK] MPU wrote %zu bytes\n", mpu_len);
            pthread_mutex_lock(&ctx->writer_mutex);
            ctx->mpu_offsets[mpu_idx] = 0;
            ctx->mpu_pending_idx = -1;
            pthread_cond_broadcast(&ctx->space_cond);
            pthread_mutex_unlock(&ctx->writer_mutex);
        }
        if (icm_idx >= 0 && icm_len > 0) {
            if (file_write_all(ctx->fd_icm, ctx->icm_buffers[icm_idx], icm_len) < 0) {
                pthread_mutex_lock(&ctx->writer_mutex);
                ctx->writer_error = -1;
                pthread_cond_broadcast(&ctx->space_cond);
                pthread_mutex_unlock(&ctx->writer_mutex);
                perror("write icm"); break;
            }
            printf("  [DISK] ICM wrote %zu bytes\n", icm_len);
            pthread_mutex_lock(&ctx->writer_mutex);
            ctx->icm_offsets[icm_idx] = 0;
            ctx->icm_pending_idx = -1;
            pthread_cond_broadcast(&ctx->space_cond);
            pthread_mutex_unlock(&ctx->writer_mutex);
        }
        if (mpu9265_idx >= 0 && mpu9265_len > 0) {
            if (file_write_all(ctx->fd_mpu9265, ctx->mpu9265_buffers[mpu9265_idx], mpu9265_len) < 0) {
                pthread_mutex_lock(&ctx->writer_mutex);
                ctx->writer_error = -1;
                pthread_cond_broadcast(&ctx->space_cond);
                pthread_mutex_unlock(&ctx->writer_mutex);
                perror("write mpu9265"); break;
            }
            printf("  [DISK] MPU9265 wrote %zu bytes\n", mpu9265_len);
            pthread_mutex_lock(&ctx->writer_mutex);
            ctx->mpu9265_offsets[mpu9265_idx] = 0;
            ctx->mpu9265_pending_idx = -1;
            pthread_cond_broadcast(&ctx->space_cond);
            pthread_mutex_unlock(&ctx->writer_mutex);
        }
    }
    return NULL;
}

/**
 * @brief Initialize file I/O with async double-buffered writes
 */
static inline int file_io_init(file_context_t* ctx, const acquisition_config_t* config) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->fd_mpu = -1; ctx->fd_icm = -1; ctx->fd_mpu9265 = -1;
    ctx->mpu_pending_idx = -1; ctx->icm_pending_idx = -1; ctx->mpu9265_pending_idx = -1;

    ctx->mpu_buffer_size    = (size_t)config->sample_rate_hz * config->test_duration_sec * 14;
    ctx->icm_buffer_size    = (size_t)config->sample_rate_hz * config->test_duration_sec * 12;
    ctx->mpu9265_buffer_size = (size_t)config->sample_rate_hz * config->test_duration_sec * 14;

    for (int i = 0; i < 2; i++) {
        ctx->mpu_buffers[i]     = (uint8_t*)malloc(ctx->mpu_buffer_size);
        ctx->icm_buffers[i]     = (uint8_t*)malloc(ctx->icm_buffer_size);
        ctx->mpu9265_buffers[i] = (uint8_t*)malloc(ctx->mpu9265_buffer_size);
        if (!ctx->mpu_buffers[i] || !ctx->icm_buffers[i] || !ctx->mpu9265_buffers[i]) {
            perror("malloc buffer");
            for (int j = 0; j < 2; j++) {
                if (ctx->mpu_buffers[j])     free(ctx->mpu_buffers[j]);
                if (ctx->icm_buffers[j])     free(ctx->icm_buffers[j]);
                if (ctx->mpu9265_buffers[j]) free(ctx->mpu9265_buffers[j]);
            }
            return -1;
        }
        ctx->mpu_offsets[i] = ctx->icm_offsets[i] = ctx->mpu9265_offsets[i] = 0;
    }

    ctx->fd_mpu    = open(config->mpu_output_file,    O_CREAT | O_WRONLY | O_TRUNC, 0666);
    ctx->fd_icm    = open(config->icm_output_file,    O_CREAT | O_WRONLY | O_TRUNC, 0666);
    ctx->fd_mpu9265 = open(config->mpu9265_output_file, O_CREAT | O_WRONLY | O_TRUNC, 0666);
    if (ctx->fd_mpu < 0 || ctx->fd_icm < 0 || ctx->fd_mpu9265 < 0) {
        perror("open output file");
        if (ctx->fd_mpu    >= 0) close(ctx->fd_mpu);
        if (ctx->fd_icm    >= 0) close(ctx->fd_icm);
        if (ctx->fd_mpu9265 >= 0) close(ctx->fd_mpu9265);
        for (int i = 0; i < 2; i++) {
            free(ctx->mpu_buffers[i]); free(ctx->icm_buffers[i]); free(ctx->mpu9265_buffers[i]);
        }
        return -1;
    }

    if (pthread_mutex_init(&ctx->writer_mutex, NULL) != 0 ||
        pthread_cond_init(&ctx->writer_cond,  NULL) != 0 ||
        pthread_cond_init(&ctx->space_cond,   NULL) != 0) {
        perror("pthread init");
        close(ctx->fd_mpu); close(ctx->fd_icm); close(ctx->fd_mpu9265);
        for (int i = 0; i < 2; i++) {
            free(ctx->mpu_buffers[i]); free(ctx->icm_buffers[i]); free(ctx->mpu9265_buffers[i]);
        }
        return -1;
    }

    if (pthread_create(&ctx->writer_thread, NULL, file_writer_thread_main, ctx) != 0) {
        perror("pthread_create");
        pthread_cond_destroy(&ctx->space_cond);
        pthread_cond_destroy(&ctx->writer_cond);
        pthread_mutex_destroy(&ctx->writer_mutex);
        close(ctx->fd_mpu); close(ctx->fd_icm); close(ctx->fd_mpu9265);
        for (int i = 0; i < 2; i++) {
            free(ctx->mpu_buffers[i]); free(ctx->icm_buffers[i]); free(ctx->mpu9265_buffers[i]);
        }
        return -1;
    }
    return 0;
}

/**
 * @brief Write selected sensor samples to double-buffered async queue.
 */
static inline int file_write_samples_masked(file_context_t* ctx,
                                            const mpu6050_sample_t* mpu_sample,
                                            const icm20649_sample_t* icm_sample,
                                            const mpu9265_sample_t* mpu9265_sample,
                                            bool write_mpu, bool write_icm, bool write_mpu9265) {
    pthread_mutex_lock(&ctx->writer_mutex);
    if (ctx->writer_error != 0) { pthread_mutex_unlock(&ctx->writer_mutex); return -1; }

    if (write_icm) {
        int idx = ctx->icm_active_idx;
        if (ctx->icm_offsets[idx] + 12 > ctx->icm_buffer_size) {
            if (file_queue_full_buffer(ctx, FILE_SENSOR_ICM) < 0) {
                pthread_mutex_unlock(&ctx->writer_mutex); return -1;
            }
            idx = ctx->icm_active_idx;
        }
        memcpy(&ctx->icm_buffers[idx][ctx->icm_offsets[idx]], icm_sample->raw_data, 12);
        ctx->icm_offsets[idx] += 12;
        if (ctx->icm_offsets[idx] >= ctx->icm_buffer_size)
            if (file_queue_full_buffer(ctx, FILE_SENSOR_ICM) < 0) {
                pthread_mutex_unlock(&ctx->writer_mutex); return -1;
            }
    }

    if (write_mpu) {
        int idx = ctx->mpu_active_idx;
        if (ctx->mpu_offsets[idx] + 14 > ctx->mpu_buffer_size) {
            if (file_queue_full_buffer(ctx, FILE_SENSOR_MPU) < 0) {
                pthread_mutex_unlock(&ctx->writer_mutex); return -1;
            }
            idx = ctx->mpu_active_idx;
        }
        memcpy(&ctx->mpu_buffers[idx][ctx->mpu_offsets[idx]], mpu_sample->raw_data, 14);
        ctx->mpu_offsets[idx] += 14;
        if (ctx->mpu_offsets[idx] >= ctx->mpu_buffer_size)
            if (file_queue_full_buffer(ctx, FILE_SENSOR_MPU) < 0) {
                pthread_mutex_unlock(&ctx->writer_mutex); return -1;
            }
    }

    if (write_mpu9265) {
        int idx = ctx->mpu9265_active_idx;
        if (ctx->mpu9265_offsets[idx] + 14 > ctx->mpu9265_buffer_size) {
            if (file_queue_full_buffer(ctx, FILE_SENSOR_MPU9265) < 0) {
                pthread_mutex_unlock(&ctx->writer_mutex); return -1;
            }
            idx = ctx->mpu9265_active_idx;
        }
        memcpy(&ctx->mpu9265_buffers[idx][ctx->mpu9265_offsets[idx]], mpu9265_sample->raw_data, 14);
        ctx->mpu9265_offsets[idx] += 14;
        if (ctx->mpu9265_offsets[idx] >= ctx->mpu9265_buffer_size)
            if (file_queue_full_buffer(ctx, FILE_SENSOR_MPU9265) < 0) {
                pthread_mutex_unlock(&ctx->writer_mutex); return -1;
            }
    }

    pthread_mutex_unlock(&ctx->writer_mutex);
    return 0;
}

static inline int file_write_samples(file_context_t* ctx, const mpu6050_sample_t* mpu_sample,
                                     const icm20649_sample_t* icm_sample,
                                     const mpu9265_sample_t* mpu9265_sample) {
    return file_write_samples_masked(ctx, mpu_sample, icm_sample, mpu9265_sample, true, true, true);
}

/**
 * @brief Flush remaining data to disk and wait for background writer.
 */
static inline int file_flush(file_context_t* ctx) {
    pthread_mutex_lock(&ctx->writer_mutex);
    if (ctx->writer_error != 0) { pthread_mutex_unlock(&ctx->writer_mutex); return -1; }

    if (ctx->mpu_offsets[ctx->mpu_active_idx] > 0)
        if (file_queue_full_buffer(ctx, FILE_SENSOR_MPU) < 0) {
            pthread_mutex_unlock(&ctx->writer_mutex); return -1;
        }
    if (ctx->icm_offsets[ctx->icm_active_idx] > 0)
        if (file_queue_full_buffer(ctx, FILE_SENSOR_ICM) < 0) {
            pthread_mutex_unlock(&ctx->writer_mutex); return -1;
        }
    if (ctx->mpu9265_offsets[ctx->mpu9265_active_idx] > 0)
        if (file_queue_full_buffer(ctx, FILE_SENSOR_MPU9265) < 0) {
            pthread_mutex_unlock(&ctx->writer_mutex); return -1;
        }

    while ((ctx->mpu_pending_idx != -1 || ctx->icm_pending_idx != -1 ||
            ctx->mpu9265_pending_idx != -1) && ctx->writer_error == 0) {
        pthread_cond_wait(&ctx->space_cond, &ctx->writer_mutex);
    }

    int err = ctx->writer_error;
    pthread_mutex_unlock(&ctx->writer_mutex);
    return (err == 0) ? 0 : -1;
}

/**
 * @brief Close files and free buffers
 */
static inline void file_cleanup(file_context_t* ctx) {
    pthread_mutex_lock(&ctx->writer_mutex);
    ctx->writer_stop = true;
    pthread_cond_signal(&ctx->writer_cond);
    pthread_mutex_unlock(&ctx->writer_mutex);

    pthread_join(ctx->writer_thread, NULL);
    pthread_cond_destroy(&ctx->space_cond);
    pthread_cond_destroy(&ctx->writer_cond);
    pthread_mutex_destroy(&ctx->writer_mutex);

    if (ctx->fd_mpu    >= 0) close(ctx->fd_mpu);
    if (ctx->fd_icm    >= 0) close(ctx->fd_icm);
    if (ctx->fd_mpu9265 >= 0) close(ctx->fd_mpu9265);
    for (int i = 0; i < 2; i++) {
        if (ctx->mpu_buffers[i])     free(ctx->mpu_buffers[i]);
        if (ctx->icm_buffers[i])     free(ctx->icm_buffers[i]);
        if (ctx->mpu9265_buffers[i]) free(ctx->mpu9265_buffers[i]);
    }
}

/**
 * @brief Convert a binary sensor file to CSV for an arbitrary channel layout.
 *
 * Each sample is assumed to contain `channel_count` consecutive int16 values.
 * If `channel_names` is NULL, default names ch0..chN are emitted.
 * Set `little_endian` to true for data written from native int16 buffers on the
 * Red Pitaya, or false for raw sensor register dumps stored in big-endian order.
 */
static inline int bin_to_csv(
    const char* bin_path,
    const char* csv_path,
    int sample_rate_hz,
    int channel_count,
    const char* const* channel_names,
    bool little_endian
) {
    if (sample_rate_hz <= 0 || channel_count <= 0) {
        fprintf(stderr, "bin_to_csv: invalid sample_rate_hz or channel_count\n");
        return -1;
    }

    const size_t sample_size = (size_t)channel_count * sizeof(int16_t);
    uint8_t* sample_bytes = NULL;
    int fd = open(bin_path, O_RDONLY);
    if (fd < 0) { perror("open bin"); return -1; }

    FILE* fp = fopen(csv_path, "w");
    if (!fp) {
        perror("fopen csv");
        close(fd);
        return -1;
    }

    sample_bytes = (uint8_t*)malloc(sample_size);
    if (!sample_bytes) {
        perror("malloc");
        fclose(fp);
        close(fd);
        return -1;
    }

    fprintf(fp, "t_s,us");
    for (int ch = 0; ch < channel_count; ++ch) {
        if (channel_names != NULL && channel_names[ch] != NULL) {
            fprintf(fp, ",%s", channel_names[ch]);
        } else {
            fprintf(fp, ",ch%d", ch);
        }
    }
    fprintf(fp, "\n");

    int idx = 0;
    while (1) {
        ssize_t r = read(fd, sample_bytes, sample_size);
        if (r == 0) break;
        if (r < 0) {
            perror("read bin");
            free(sample_bytes);
            fclose(fp);
            close(fd);
            return -1;
        }
        if ((size_t)r != sample_size) {
            fprintf(stderr, "partial sample (%zd bytes)\n", r);
            break;
        }

        double t_s = (double)idx / (double)sample_rate_hz;
        long long t_us = ((long long)idx * 1000000LL) / (long long)sample_rate_hz;
        fprintf(fp, "%.9f,%lld", t_s, t_us);

        for (int ch = 0; ch < channel_count; ++ch) {
            const int byte_offset = ch * 2;
            int16_t value;

            if (little_endian) {
                value = (int16_t)((uint16_t)sample_bytes[byte_offset] |
                                  ((uint16_t)sample_bytes[byte_offset + 1] << 8));
            } else {
                value = (int16_t)(((uint16_t)sample_bytes[byte_offset] << 8) |
                                   (uint16_t)sample_bytes[byte_offset + 1]);
            }

            fprintf(fp, ",%d", value);
        }

        fprintf(fp, "\n");
        idx++;
    }

    free(sample_bytes);
    fclose(fp);
    close(fd);
    return 0;
}


/*====== MODULE 4: Network Streaming ======*/

/**
 * @brief Initialize TCP streaming server (non-blocking)
 */
static inline int stream_init(stream_context_t* ctx, uint16_t port, uint32_t sample_rate_hz, uint32_t stream_rate_hz) {
    ctx->port = port;
    ctx->stream_rate_hz = stream_rate_hz;
    ctx->sample_rate_hz = sample_rate_hz;
    ctx->decimation = sample_rate_hz / stream_rate_hz;
    if (ctx->decimation <= 0) ctx->decimation = 1;
    ctx->stream_counter = 0;
    ctx->packets_sent = 0;
    ctx->client_fd = -1;

    ctx->server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (ctx->server_fd < 0) { perror("socket"); return -1; }
    int opt = 1;
    setsockopt(ctx->server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(port);
    if (bind(ctx->server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind"); close(ctx->server_fd); return -1;
    }
    if (listen(ctx->server_fd, 1) < 0) {
        perror("listen"); close(ctx->server_fd); return -1;
    }
    int flags = fcntl(ctx->server_fd, F_GETFL, 0);
    fcntl(ctx->server_fd, F_SETFL, flags | O_NONBLOCK);

    // Pre-build Open Ephys-style header for 31-channel, 84-byte packet
    int32_t offset = 0, bytesPerBuffer = 62, elementSize = 2, numChannels = 31, numSamples = 1;
    int16_t dataType = 3;
    memcpy(&ctx->header[0],  &offset, 4);
    memcpy(&ctx->header[4],  &bytesPerBuffer, 4);
    memcpy(&ctx->header[8],  &dataType, 2);
    memcpy(&ctx->header[10], &elementSize, 4);
    memcpy(&ctx->header[14], &numChannels, 4);
    memcpy(&ctx->header[18], &numSamples, 4);
    printf("Streaming server listening on port %d (stream rate %d Hz)\n", port, stream_rate_hz);
    return 0;
}

/**
 * @brief Accept new client connection (non-blocking)
 */
static inline int stream_accept_client(stream_context_t* ctx, timer_context_t* timer_ctx, volatile uint32_t* timer_gpio_map) {
    (void)timer_ctx;
    (void)timer_gpio_map;
    if (ctx->client_fd >= 0) return 0;
    struct sockaddr_in cliaddr;
    socklen_t clilen = sizeof(cliaddr);
    ctx->client_fd = accept(ctx->server_fd, (struct sockaddr*)&cliaddr, &clilen);
    if (ctx->client_fd >= 0) {
        printf("Client connected for streaming.\n");
        int cflags = fcntl(ctx->client_fd, F_GETFL, 0);
        fcntl(ctx->client_fd, F_SETFL, cflags | O_NONBLOCK);
        return 1;
    }
    else if (errno != EAGAIN && errno != EWOULDBLOCK) {
        perror("accept");
        return -1;
    }
    return 0;
}

/**
 * @brief Send decimated sample to streaming client (with per-sensor enable mask).
 */
// Scale factor for quaternion values (-1..1) to int16_t
#define STREAM_QUAT_SCALE 30000.0

#define _STREAM_CLAMP(v) ((int16_t)((v) > 32767.0 ? 32767 : (v) < -32768.0 ? -32768 : (v)))
#define _STREAM_Q(q, i, out, ch) do { double _v = (double)(q)[i] * STREAM_QUAT_SCALE; (out)[ch] = _STREAM_CLAMP(_v); } while(0)

static inline int stream_send_sample_masked(stream_context_t* ctx,
                                            const mpu6050_sample_t*  mpu_sample,
                                            const mpu9265_sample_t*  mpu9265_sample,
                                            const icm20649_sample_t* icm_sample,
                                            bool use_mpu, bool use_mpu9265, bool use_icm,
                                            const vqf_real_t* mpu_quat,
                                            const vqf_real_t* mpu9265_quat,
                                            const vqf_real_t* icm_quat,
                                            float joint_angle_deg) {
    if (ctx->client_fd < 0) return 0;
    ctx->stream_counter++;
    if (ctx->stream_counter < ctx->decimation) return 0;
    ctx->stream_counter = 0;

    int16_t ch[STREAM_NUM_CHANNELS] = {0};

    // ch[0:6]  MPU6050  ax,ay,az,gx,gy,gz   ch[6:10]  qw,qx,qy,qz
    if (use_mpu) {
        ch[0] = (int16_t)((mpu_sample->raw_data[0]  << 8) | mpu_sample->raw_data[1]);
        ch[1] = (int16_t)((mpu_sample->raw_data[2]  << 8) | mpu_sample->raw_data[3]);
        ch[2] = (int16_t)((mpu_sample->raw_data[4]  << 8) | mpu_sample->raw_data[5]);
        ch[3] = (int16_t)((mpu_sample->raw_data[8]  << 8) | mpu_sample->raw_data[9]);
        ch[4] = (int16_t)((mpu_sample->raw_data[10] << 8) | mpu_sample->raw_data[11]);
        ch[5] = (int16_t)((mpu_sample->raw_data[12] << 8) | mpu_sample->raw_data[13]);
        if (mpu_quat) { _STREAM_Q(mpu_quat,0,ch,6); _STREAM_Q(mpu_quat,1,ch,7);
                        _STREAM_Q(mpu_quat,2,ch,8); _STREAM_Q(mpu_quat,3,ch,9); }
    }

    // ch[10:16] MPU9265 ax,ay,az,gx,gy,gz   ch[16:20] qw,qx,qy,qz
    if (use_mpu9265) {
        ch[10] = (int16_t)((mpu9265_sample->raw_data[0]  << 8) | mpu9265_sample->raw_data[1]);
        ch[11] = (int16_t)((mpu9265_sample->raw_data[2]  << 8) | mpu9265_sample->raw_data[3]);
        ch[12] = (int16_t)((mpu9265_sample->raw_data[4]  << 8) | mpu9265_sample->raw_data[5]);
        ch[13] = (int16_t)((mpu9265_sample->raw_data[8]  << 8) | mpu9265_sample->raw_data[9]);
        ch[14] = (int16_t)((mpu9265_sample->raw_data[10] << 8) | mpu9265_sample->raw_data[11]);
        ch[15] = (int16_t)((mpu9265_sample->raw_data[12] << 8) | mpu9265_sample->raw_data[13]);
        if (mpu9265_quat) { _STREAM_Q(mpu9265_quat,0,ch,16); _STREAM_Q(mpu9265_quat,1,ch,17);
                            _STREAM_Q(mpu9265_quat,2,ch,18); _STREAM_Q(mpu9265_quat,3,ch,19); }
    }

    // ch[20:26] ICM20948 ax,ay,az,gx,gy,gz  ch[26:30] qw,qx,qy,qz
    if (use_icm) {
        ch[20] = (int16_t)((icm_sample->raw_data[0]  << 8) | icm_sample->raw_data[1]);
        ch[21] = (int16_t)((icm_sample->raw_data[2]  << 8) | icm_sample->raw_data[3]);
        ch[22] = (int16_t)((icm_sample->raw_data[4]  << 8) | icm_sample->raw_data[5]);
        ch[23] = (int16_t)((icm_sample->raw_data[6]  << 8) | icm_sample->raw_data[7]);
        ch[24] = (int16_t)((icm_sample->raw_data[8]  << 8) | icm_sample->raw_data[9]);
        ch[25] = (int16_t)((icm_sample->raw_data[10] << 8) | icm_sample->raw_data[11]);
        if (icm_quat) { _STREAM_Q(icm_quat,0,ch,26); _STREAM_Q(icm_quat,1,ch,27);
                        _STREAM_Q(icm_quat,2,ch,28); _STREAM_Q(icm_quat,3,ch,29); }
    }

    // ch[30] joint angle × 100
    { double v = joint_angle_deg * 100.0; ch[30] = _STREAM_CLAMP(v); }

    memcpy(ctx->packet, ctx->header, 22);
    memcpy(ctx->packet + 22, ch, STREAM_NUM_CHANNELS * 2);
    ssize_t s = send(ctx->client_fd, ctx->packet, STREAM_PACKET_SIZE, MSG_NOSIGNAL | MSG_DONTWAIT);
    if (s > 0) {
        ctx->packets_sent++;
        if ((ctx->packets_sent % 100) == 0) printf("Sent %u packets\n", ctx->packets_sent);
    } else if (s < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        perror("send");
        close(ctx->client_fd);
        ctx->client_fd = -1;
        printf("Client disconnected; streaming will wait for reconnect.\n");
        return -1;
    }
    return 0;
}

/**
 * @brief Close streaming connections
 */
static inline void stream_cleanup(stream_context_t* ctx) {
    if (ctx->client_fd >= 0) close(ctx->client_fd);
    if (ctx->server_fd >= 0) close(ctx->server_fd);
}


/*====== MODULE 5: Configuration ======*/

/**
 * @brief Initialize configuration with defaults
 */
static inline void config_init_defaults(acquisition_config_t* config) {
    config->sample_rate_hz = 1000;
    config->test_duration_sec = 120;
    config->file_write_interval_sec = 5;
    config->enable_streaming = false;
    config->stream_port = 9001;
    config->stream_rate_hz = 100;
    config->timer_clock_rate = 125000000;
    strcpy(config->mpu_output_file,    "mpu6050.bin");
    strcpy(config->icm_output_file,    "icm20948.bin");
    strcpy(config->mpu9265_output_file, "mpu9265.bin");
    strcpy(config->mpu_csv_file,    "mpu6050.csv");
    strcpy(config->icm_csv_file,    "icm20948.csv");
    strcpy(config->mpu9265_csv_file, "mpu9265.csv");
}

/**
 * @brief Parse command-line arguments
 */
static inline acquisition_mode_t config_parse_args(acquisition_config_t* config, int argc, char** argv) {
    (void)config;
    if (argc > 1) {
        if (strcmp(argv[1], "log_only") == 0) {
            printf("Command-line: LOG ONLY mode.\n");
            fflush(stdout);
            return MODE_LOG_ONLY;
        }
        else if (strcmp(argv[1], "stream") == 0) {
            printf("Command-line: LOG AND STREAM mode.\n");
            fflush(stdout);
            return MODE_LOG_AND_STREAM;
        }
    }
    return MODE_UNKNOWN;
}

/**
 * @brief Prompt user for operating mode
 */
static inline acquisition_mode_t prompt_for_mode(void) {
    printf("\n==============================================\n");
    printf("SELECT OPERATING MODE (before connecting to PC):\n");
    printf("==============================================\n");
    printf("  1. Log only (no streaming, no PC connection)\n");
    printf("  2. Log and stream (will wait for PC client)\n");
    printf("==============================================\n");
    printf("Enter choice (1 or 2): ");
    fflush(stdout);
    char input[32];
    if (fgets(input, sizeof(input), stdin) != NULL) {
        int choice = atoi(input);
        if (choice == 1) {
            printf("\n>>> LOG ONLY mode selected <<<\n");
            fflush(stdout);
            sleep(2);
            return MODE_LOG_ONLY;
        }
        else if (choice == 2) {
            printf("\n>>> LOG AND STREAM mode selected <<<\n");
            fflush(stdout);
            sleep(2);
            return MODE_LOG_AND_STREAM;
        }
        else {
            printf("\n>>> Invalid input, defaulting to LOG AND STREAM <<<\n");
        }
    }
    else {
        printf("\n>>> No input (non-interactive?), defaulting to LOG AND STREAM <<<\n");
    }
    fflush(stdout);
    sleep(2);
    return MODE_LOG_AND_STREAM;
}

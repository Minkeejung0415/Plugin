/*
    ============================================================
    RedPitaya_justin.c — IMU streaming firmware for Red Pitaya ARM
    ============================================================

    This C program runs on the Red Pitaya's ARM processor (Linux).
    It does five things:
      1. Scans up to 4 AXI I2C buses + 2 AXI SPI buses to find attached IMU chips.
      2. Calibrates each sensor's DC bias (gyro/accel offset) with 3000 still samples.
      3. Runs VQF sensor fusion on each IMU to produce orientation quaternions.
      4. Packs all sensor data into 22-byte-header binary frames.
      5. Streams those frames over a TCP socket (port 5000) to the Open Ephys plugin on the PC.

    The PC plugin sends ASCII commands back (START, STOP, RECORD ON/OFF, FILTER ON/OFF,
    FREQ:<hz>, CFG <si> ACC/GYR/SRATE <val>) to control streaming and sensor configuration.
*/
#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "rp.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <stdbool.h>
#include <setjmp.h>
#include <signal.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>

#include <netinet/tcp.h>

#include "axi_header.h"
#include "sensor_fusion.h"
#include "vqf.h"

// --- Hardware Constants ---
#define AXI_GPIO_ADDRESS  0x41200000  /* Physical memory address of the AXI GPIO block (FPGA hardware timer) */
#define RANGE             64000       /* mmap size for the GPIO block (bytes) */
#define IIC_RANGE         64000       /* mmap size for each AXI IIC or SPI block (bytes) */
#define PORT              5000        /* TCP port Open Ephys connects to */
#define DESIRED_SAMPLE_RATE_HZ 100   /* Default hardware tick rate at startup (Hz); FREQ: command changes it */
#define CTR_CLK_RATE      125000000  /* FPGA clock speed (125 MHz); used to compute ticks_per_sample */
#define HEADER_SIZE       22          /* Every frame starts with a 22-byte binary header (see write_stream_header) */
#define ANALOG_WAVEFORM_CHANNELS 2   /* Red Pitaya CH1/CH2 oscilloscope inputs appended to each frame */
#define GYRO_BIAS_CALIBRATION_SAMPLES 3000  /* ~3 seconds of still samples used to estimate sensor DC bias */
#define IMU_BIAS_CALIBRATION_SLEEP_US 1000  /* 1 ms sleep between calibration samples so we don't spam the I2C bus */
#define ICM20948_BANK_0 0x00
#define ICM20948_BANK_2 0x02
#define ICM20948_BANK_3 0x03
#define ICM20948_EXT_SENS_DATA_00 0x3B
#define ICM20948_I2C_MST_CTRL 0x01
#define ICM20948_I2C_SLV0_ADDR 0x03
#define ICM20948_I2C_SLV0_REG 0x04
#define ICM20948_I2C_SLV0_CTRL 0x05
#define ICM20948_I2C_SLV0_DO 0x06
#define ICM20948_AK09916_ADDR 0x0C
#define ICM20948_AK09916_ST1 0x10
#define ICM20948_AK09916_HXL 0x11
#define ICM20948_AK09916_CNTL2 0x31

#define BUF_SAMPLES 1000 // Flushes to SD card every 1,000 samples
#define UDP_PORT       55001
#define CHUNK_SAMPLES  1    // 1 packet per UDP datagram keeps latency at ~1/sampleRate; increase only if network overhead becomes an issue
#define MAX_STREAM_SENSORS 6

/* Hardware tick rate for streaming (Hz); FREQ: command updates this before START. */
static volatile int g_stream_hw_hz = DESIRED_SAMPLE_RATE_HZ;

static int clamp_stream_hw_hz(int hz)
{
    if (hz < 1) return 1;
    if (hz > 2000) return 2000;
    return hz;
}

static int current_stream_hw_hz(void)
{
    return clamp_stream_hw_hz(g_stream_hw_hz);
}

/*
    ticks_for_stream_hz() — converts a desired sample rate (Hz) into the number of
    FPGA hardware clock ticks between samples.

    Math: ticks_per_sample = CTR_CLK_RATE / sample_rate_hz
    Example: at 100 Hz → 125,000,000 / 100 = 1,250,000 ticks between samples.
    The main stream loop spin-waits on the GPIO counter until this many ticks pass,
    pacing the stream at the exact desired rate without a software sleep.
*/
static uint32_t ticks_for_stream_hz(int hz)
{
    uint32_t ticks = (uint32_t)(CTR_CLK_RATE / clamp_stream_hw_hz(hz));
    return ticks > 0 ? ticks : 1;
}

/*
    SensorInstance — one IMU chip (one physical sensor board).
    Up to 6 of these can be active at once (4 I2C + 2 SPI).
*/
typedef struct {
    char name[16];       /* chip model string: "MPU6050", "MPU9250", "ICM20948", or "BNO055" */
    void *axi_map;       /* mmap pointer to the AXI IP block (IIC or SPI) for this sensor's bus */
    uint8_t i2c_addr;    /* I2C slave address (e.g. 0x68 for MPU6050); unused for SPI */
    int num_channels;    /* 6 for accel+gyro only, 9 for accel+gyro+mag */
    uint8_t data_reg_start; /* first data register to burst-read (e.g. 0x3B on MPU6050) */
    bool active;         /* true if this slot holds a detected sensor */
    bool split_read;     /* true if temperature register sits between accel and gyro (MPU6050) */
    bool is_spi;         /* true = SPI protocol via axi_spi_*, false = I2C via axi_iic_* */
    int16_t gyro_bias[3]; /* 3-axis gyroscope DC offset (raw counts); subtracted before VQF */
    int16_t acc_bias[3];  /* 3-axis accelerometer DC offset (raw counts); subtracted before VQF */
    bool mag_is_fresh;   /* true if the magnetometer reading in mag_cache was updated this tick */
    int16_t mag_cache[3]; /* last valid magnetometer reading, held until the mag updates again */
    int mag_skip_counter; /* counts hardware ticks between magnetometer reads (mag runs slower) */
    /* Per-sensor UI config (CFG lines from Open Ephys); applied during stream */
    int cfg_acc_id;      /* 0=±2g, 1=±4g, 2=±8g, 3=±16g (chosen by operator in device editor) */
    int cfg_gyr_id;      /* 0=±250°/s, 1=±500, 2=±1000, 3=±2000 */
    int cfg_target_hz;   /* desired effective sample rate for this sensor (firmware decimates to achieve it) */
} SensorInstance;

/*
    HardwareContext — the entire system state, passed around to most functions.
*/
typedef struct {
    void *axi_gpio_map;              /* mmap pointer to the FPGA GPIO block */
    volatile uint32_t *gpio_counter; /* hardware counter register — increments at 125 MHz;
                                        we spin-wait until it advances by ticks_per_sample */
    volatile uint32_t *gpio_reset;   /* writing 1 then 0 resets the counter to zero */

    SensorInstance sensors[6]; /* up to 4 I2C slots + 2 SPI slots */
    int active_sensor_count;   /* how many sensors were actually found during init */
    int total_channels;        /* sum of num_channels+4 (fusion) for all active sensors + 2 analog */
} HardwareContext;

// Global jump buffer for the watchdog
static sigjmp_buf watchdog_bucket;
static volatile sig_atomic_t stop_requested = 0;
static bool analog_inputs_ready = false;

static long long timespec_diff_ns(const struct timespec *start, const struct timespec *end) {
    return ((long long)(end->tv_sec - start->tv_sec) * 1000000000LL) +
           (long long)(end->tv_nsec - start->tv_nsec);
}
/*
    AXI IIC register offsets — from the Xilinx AXI IIC data sheet (PG090).
    We mmap the core's register block and access these offsets as volatile uint32_t pointers.
    This is the standard Xilinx I2C IP core used in FPGA designs.
*/
#define XIIC_RESETR_OFFSET           0x40u   /* Software Reset Register — write 0x0A to reset the core */
#define XIIC_CR_REG_OFFSET           0x100u  /* Control Register — enable/reset/configure the core */
#define XIIC_SR_REG_OFFSET           0x104u  /* Status Register — read to check bus/FIFO state */
#define XIIC_DRR_REG_OFFSET          0x10Cu  /* Data Receive Register — read to drain the RX FIFO */
#define XIIC_RESET_MASK              0x0Au   /* Magic value required by Xilinx spec to trigger core reset */
#define XIIC_CR_ENABLE_DEVICE_MASK   0x01u   /* CR bit 0: set to 1 to enable the I2C core */
#define XIIC_CR_TX_FIFO_RESET_MASK   0x02u   /* CR bit 1: set to 1 to flush the TX FIFO */
#define XIIC_SR_BUS_BUSY_MASK        0x04u   /* SR bit 2: 1 = another master is on the wire (or bus hung) */
#define XIIC_SR_RX_FIFO_EMPTY_MASK   0x40u   /* SR bit 6: 1 = RX FIFO is empty (nothing stuck in receive queue) */
#define XIIC_SR_TX_FIFO_EMPTY_MASK   0x80u   /* SR bit 7: 1 = TX FIFO is empty (all pending sends completed) */

static uint32_t axi_iic_read_sr(void *axi_map)
{
    volatile uint32_t *sr = (volatile uint32_t *)((uint8_t *)axi_map + XIIC_SR_REG_OFFSET);
    return *sr;
}

/*
    axi_iic_force_reset() — recovers a hung AXI IIC (I2C) core.

    I2C buses can get stuck if a transaction is interrupted mid-way (e.g. power glitch,
    firmware crash) and the slave device is still driving SDA low. This function performs
    a full Xilinx software recovery sequence:
      Step 1: Write the reset value (0x0A) to the Reset Register — tells the core to reinitialise.
      Step 2: Set TX_FIFO_RESET bit in the Control Register — flushes any bytes stuck in TX queue.
      Step 3: Drain the RX FIFO by reading the DRR register up to 256 times until
              the RX_FIFO_EMPTY bit is set — clears any garbage received before the hang.
      Step 4: Set ENABLE_DEVICE bit in Control Register — brings the core back online.
*/
static void axi_iic_force_reset(void *axi_map)
{
    if (axi_map == NULL)
        return;

    volatile uint32_t *reset_reg = (volatile uint32_t *)((uint8_t *)axi_map + XIIC_RESETR_OFFSET);
    volatile uint8_t *cr = (volatile uint8_t *)((uint8_t *)axi_map + XIIC_CR_REG_OFFSET);
    volatile uint32_t *drr = (volatile uint32_t *)((uint8_t *)axi_map + XIIC_DRR_REG_OFFSET);

    *reset_reg = XIIC_RESET_MASK;   /* Step 1: software reset the core */
    usleep(2000);

    *cr = XIIC_CR_TX_FIFO_RESET_MASK;  /* Step 2: flush the TX FIFO */
    usleep(200);

    for (int i = 0; i < 256; i++) {    /* Step 3: drain the RX FIFO */
        if (axi_iic_read_sr(axi_map) & XIIC_SR_RX_FIFO_EMPTY_MASK)
            break;
        (void)*drr;  /* read and discard one byte from RX FIFO */
    }

    *cr = XIIC_CR_ENABLE_DEVICE_MASK;  /* Step 4: re-enable the I2C core */
    usleep(500);
}

static bool axi_iic_bus_looks_stuck(void *axi_map)
{
    const uint32_t sr = axi_iic_read_sr(axi_map);

    if (!(sr & XIIC_SR_TX_FIFO_EMPTY_MASK))
        return true;

    if ((sr & XIIC_SR_BUS_BUSY_MASK) && !(sr & XIIC_SR_TX_FIFO_EMPTY_MASK))
        return true;

    return false;
}

static void axi_iic_recover_bus(void *axi_map, const char *reason)
{
    static time_t last_log_sec;
    static unsigned suppressed_logs;

    if (axi_map == NULL)
        return;

    {
        const time_t now = time(NULL);
        if (now != last_log_sec) {
            if (suppressed_logs > 0)
                fprintf(stderr, "  (%u additional I2C recoveries suppressed)\n", suppressed_logs);
            fprintf(stderr, "axi_iic_recover_bus: %s (SR=0x%08X before reset)\n",
                    reason ? reason : "recovery", axi_iic_read_sr(axi_map));
            last_log_sec = now;
            suppressed_logs = 0;
        } else {
            suppressed_logs++;
        }
    }

    axi_iic_force_reset(axi_map);
    axi_iic_initialize(axi_map);
    usleep(500);
}

/*
    iic_read_n_bytes_recovering() — a fault-tolerant I2C read.

    I2C buses are fragile: a sensor that misses a clock edge can lock up the bus.
    We use a two-check pattern:
      Check 1 (PRE-READ):  if the bus looks stuck before we try, reset it first.
                           This prevents sending a doomed transaction.
      Do the read.
      Check 2 (POST-READ): if the bus looks stuck AFTER the read, the transaction
                           may have partially hung mid-way. Reset and retry once.
    This handles both "bus was already stuck" and "bus hung mid-transaction" cases.
*/
static void iic_read_n_bytes_recovering(void *axi_map, uint8_t slave_addr, uint8_t reg_addr,
                                        uint8_t *buffer, int num_bytes)
{
    if (axi_map == NULL || buffer == NULL || num_bytes <= 0)
        return;

    if (axi_iic_bus_looks_stuck(axi_map))
        axi_iic_recover_bus(axi_map, "pre-read");  /* check 1: clean up before attempting */

    axi_iic_read_n_bytes(axi_map, slave_addr, reg_addr, buffer, num_bytes);

    if (axi_iic_bus_looks_stuck(axi_map)) {
        axi_iic_recover_bus(axi_map, "post-read"); /* check 2: transaction may have hung mid-way */
        axi_iic_read_n_bytes(axi_map, slave_addr, reg_addr, buffer, num_bytes); /* retry once */
    }
}

static void recover_active_i2c_buses(HardwareContext *ctx)
{
    if (ctx == NULL)
        return;

    for (int i = 0; i < ctx->active_sensor_count; i++) {
        SensorInstance *s = &ctx->sensors[i];
        if (s->is_spi || s->axi_map == NULL)
            continue;
        axi_iic_recover_bus(s->axi_map, "stream-stop");
    }
}


static void icm20948_spi_select_bank(void *axi_map, uint8_t bank)
{
    axi_spi_write(axi_map, REG_BANK_SEL, (uint8_t)(bank << 4));
}

static void icm20948_spi_aux_write_byte(void *axi_map, uint8_t slave_addr, uint8_t reg_addr, uint8_t data)
{
    icm20948_spi_select_bank(axi_map, ICM20948_BANK_3);
    axi_spi_write(axi_map, ICM20948_I2C_SLV0_ADDR, slave_addr);
    axi_spi_write(axi_map, ICM20948_I2C_SLV0_REG, reg_addr);
    axi_spi_write(axi_map, ICM20948_I2C_SLV0_DO, data);
    axi_spi_write(axi_map, ICM20948_I2C_SLV0_CTRL, 0x81);
    usleep(10000);
    axi_spi_write(axi_map, ICM20948_I2C_SLV0_CTRL, 0x00);
    icm20948_spi_select_bank(axi_map, ICM20948_BANK_0);
}

static void icm20948_spi_configure_mag_passthrough(void *axi_map)
{
    icm20948_spi_select_bank(axi_map, ICM20948_BANK_0);
    axi_spi_write(axi_map, ICM_USER_CTRL, 0x20);
    usleep(10000);

    icm20948_spi_select_bank(axi_map, ICM20948_BANK_3);
    axi_spi_write(axi_map, ICM20948_I2C_MST_CTRL, 0x07);
    usleep(10000);

    icm20948_spi_aux_write_byte(axi_map, ICM20948_AK09916_ADDR, ICM20948_AK09916_CNTL2, 0x08);

    icm20948_spi_select_bank(axi_map, ICM20948_BANK_3);
    axi_spi_write(axi_map, ICM20948_I2C_SLV0_ADDR, (uint8_t)(0x80 | ICM20948_AK09916_ADDR));
    axi_spi_write(axi_map, ICM20948_I2C_SLV0_REG, ICM20948_AK09916_ST1);
    axi_spi_write(axi_map, ICM20948_I2C_SLV0_CTRL, 0x88);
    usleep(10000);

    icm20948_spi_select_bank(axi_map, ICM20948_BANK_0);
}

// Signal handler that triggers when the AXI bus hangs
void watchdog_handler(int sig) {
    siglongjmp(watchdog_bucket, 1);
}

void stop_handler(int sig) {
    (void)sig;
    stop_requested = 1;
}

static bool prompt_initial_fusion_state(void) {
    char input[32];

    while (1) {
        printf("Enable sensor fusion for this run? (true/false): ");
        fflush(stdout);

        if (fgets(input, sizeof(input), stdin) == NULL) {
            printf("\nNo input received. Defaulting sensor fusion to false.\n");
            return false;
        }

        input[strcspn(input, "\r\n")] = '\0';

        if (strcmp(input, "true") == 0 || strcmp(input, "TRUE") == 0 ||
            strcmp(input, "1") == 0 || strcmp(input, "yes") == 0 ||
            strcmp(input, "YES") == 0 || strcmp(input, "y") == 0 ||
            strcmp(input, "Y") == 0) {
            return true;
        }

        if (strcmp(input, "false") == 0 || strcmp(input, "FALSE") == 0 ||
            strcmp(input, "0") == 0 || strcmp(input, "no") == 0 ||
            strcmp(input, "NO") == 0 || strcmp(input, "n") == 0 ||
            strcmp(input, "N") == 0) {
            return false;
        }

        printf("Invalid input. Please enter true or false.\n");
    }
}

static bool prompt_yes_no(const char *question, bool default_value) {
    char input[32];

    while (1) {
        printf("%s (%s/%s) [%s]: ",
               question,
               "yes",
               "no",
               default_value ? "yes" : "no");
        fflush(stdout);

        if (fgets(input, sizeof(input), stdin) == NULL) {
            printf("\nNo input received. Defaulting to %s.\n", default_value ? "yes" : "no");
            return default_value;
        }

        input[strcspn(input, "\r\n")] = '\0';
        if (input[0] == '\0') return default_value;

        if (strcmp(input, "true") == 0 || strcmp(input, "TRUE") == 0 ||
            strcmp(input, "1") == 0 || strcmp(input, "yes") == 0 ||
            strcmp(input, "YES") == 0 || strcmp(input, "y") == 0 ||
            strcmp(input, "Y") == 0) {
            return true;
        }

        if (strcmp(input, "false") == 0 || strcmp(input, "FALSE") == 0 ||
            strcmp(input, "0") == 0 || strcmp(input, "no") == 0 ||
            strcmp(input, "NO") == 0 || strcmp(input, "n") == 0 ||
            strcmp(input, "N") == 0) {
            return false;
        }

        printf("Invalid input. Please enter yes or no.\n");
    }
}

static int prompt_capture_duration_seconds(void) {
    char input[32];
    char *endptr = NULL;
    long value;

    while (1) {
        printf("How many seconds should the standalone capture run?: ");
        fflush(stdout);

        if (fgets(input, sizeof(input), stdin) == NULL) {
            printf("\nNo input received. Defaulting to 10 seconds.\n");
            return 10;
        }

        input[strcspn(input, "\r\n")] = '\0';
        if (input[0] == '\0') {
            printf("Please enter a positive number of seconds.\n");
            continue;
        }

        value = strtol(input, &endptr, 10);
        if (endptr == input || *endptr != '\0' || value <= 0) {
            printf("Invalid duration. Please enter a positive integer.\n");
            continue;
        }

        return (int)value;
    }
}

/*
    write_stream_header() — fills the 22-byte binary frame header.

    Every frame starts with this header so the PC plugin knows how to decode it.
    We use memcpy() instead of struct assignment to avoid any compiler-added padding
    bytes that would corrupt the layout when the PC reads it as a flat byte array.

    Header layout (all little-endian):
      bytes 0-3:   offset (int32)  = 0 (always)
      bytes 4-7:   bytes_per_frame (int32) — payload size in bytes
      bytes 8-9:   dtype (int16)   = 3 (meaning int16 payload)
      bytes 10-13: elm (int32)     = 2 (element size in bytes = sizeof(int16_t))
      bytes 14-17: total_channels (int32) — number of int16 values in the payload
      bytes 18-21: ns (int32)      — monotonically increasing frame counter (sample number)
*/
static void write_stream_header(uint8_t *packet, int bytes_per_frame, int total_channels, int32_t ns)
{
    int32_t offset = 0;
    int32_t bpb = bytes_per_frame;
    int32_t elm = 2;
    int16_t dtype = 3;

    memcpy(packet + 0,  &offset, 4);   /* byte offset from start of payload (always 0) */
    memcpy(packet + 4,  &bpb, 4);      /* payload size in bytes */
    memcpy(packet + 8,  &dtype, 2);    /* data type code: 3 = int16 */
    memcpy(packet + 10, &elm, 4);      /* bytes per element: 2 */
    memcpy(packet + 14, &total_channels, 4); /* number of channels in this frame */
    memcpy(packet + 18, &ns, 4);       /* sample/frame counter */
}

static void build_measurement_path(char *buffer, size_t buffer_size, const char *prefix, const char *extension) {
    time_t rawtime;
    struct tm *timeinfo;
    char time_str[20];

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", timeinfo);
    snprintf(buffer, buffer_size, "/root/Measurements/%s_%s.%s", prefix, time_str, extension);
}

static void init_analog_waveform_inputs(void) {
    signal(SIGBUS, watchdog_handler);

    if (sigsetjmp(watchdog_bucket, 1) != 0) {
        fprintf(stderr, "SIGBUS caught during RP analog init: acquisition hardware not present in current FPGA bitstream.\n");
        fprintf(stderr, "Analog waveform channels will be zero until a bitstream with the oscilloscope IP is loaded.\n");
        analog_inputs_ready = false;
        signal(SIGBUS, SIG_DFL);
        return;
    }

    if (rp_Init() != RP_OK) {
        fprintf(stderr, "Red Pitaya analog input API init failed. Analog waveform channels will be zero.\n");
        analog_inputs_ready = false;
        signal(SIGBUS, SIG_DFL);
        return;
    }

    rp_AcqReset();
    rp_AcqSetDecimation(RP_DEC_1);
    rp_AcqStart();
    analog_inputs_ready = true;
    signal(SIGBUS, SIG_DFL);
    printf("Red Pitaya analog waveform inputs enabled (%d channels).\n", ANALOG_WAVEFORM_CHANNELS);
}

static void read_analog_waveform_channels(int16_t *channel_out) {
    for (int ch = 0; ch < ANALOG_WAVEFORM_CHANNELS; ch++) {
        channel_out[ch] = 0;
    }

    if (!analog_inputs_ready) {
        return;
    }

    for (int ch = 0; ch < ANALOG_WAVEFORM_CHANNELS; ch++) {
        uint32_t size = 1;
        int16_t sample = 0;
        rp_channel_t channel = ch == 0 ? RP_CH_1 : RP_CH_2;

        if (rp_AcqGetLatestDataRaw(channel, &size, &sample) == RP_OK && size > 0) {
            channel_out[ch] = sample;
        }
    }
}

static void read_sensor_raw_channels(SensorInstance *s, int16_t *channel_out) {
    s->mag_is_fresh = false;

    if (strcmp(s->name, "MPU9250") == 0) {
        if (s->is_spi) {
            uint8_t raw[12];
            axi_spi_read(s->axi_map, 0x3B, raw, 12);
            for (int j = 0; j < 6; j++) {
                channel_out[j] = (int16_t)((raw[j * 2] << 8) | raw[j * 2 + 1]);
            }
            channel_out[6] = 0;
            channel_out[7] = 0;
            channel_out[8] = 0;
        } else {
            uint8_t raw[12];
            iic_read_n_bytes_recovering(s->axi_map, s->i2c_addr, 0x3B, raw, 12);

            for (int j = 0; j < 6; j++) {
                channel_out[j] = (int16_t)((raw[j * 2] << 8) | raw[j * 2 + 1]);
            }

            int mag_period = current_stream_hw_hz() / 100;
            if (mag_period < 1) mag_period = 1;

            if (++s->mag_skip_counter >= mag_period) {
                uint8_t mag_raw[7];
                s->mag_skip_counter = 0;
                iic_read_n_bytes_recovering(s->axi_map, 0x0C, 0x03, mag_raw, 7);

                if (mag_raw[6] & 0x08) {
                    channel_out[6] = s->mag_cache[0];
                    channel_out[7] = s->mag_cache[1];
                    channel_out[8] = s->mag_cache[2];
                } else {
                    for (int j = 0; j < 3; j++) {
                        int16_t val = (int16_t)(mag_raw[j * 2] | (mag_raw[j * 2 + 1] << 8));
                        int32_t amplified = (int32_t)val * 16;
                        if (amplified > 32767) amplified = 32767;
                        if (amplified < -32768) amplified = -32768;
                        channel_out[j + 6] = (int16_t)amplified;
                    }
                    s->mag_cache[0] = channel_out[6];
                    s->mag_cache[1] = channel_out[7];
                    s->mag_cache[2] = channel_out[8];
                    s->mag_is_fresh = true;
                }
            } else {
                channel_out[6] = s->mag_cache[0];
                channel_out[7] = s->mag_cache[1];
                channel_out[8] = s->mag_cache[2];
            }
        }
    } else if (strcmp(s->name, "ICM20948") == 0 && s->is_spi) {
        uint8_t raw[12];
        uint8_t mag_raw[8];

        icm20948_spi_select_bank(s->axi_map, ICM20948_BANK_0);
        axi_spi_read(s->axi_map, s->data_reg_start, raw, 12);
        axi_spi_read(s->axi_map, ICM20948_EXT_SENS_DATA_00, mag_raw, 8);

        for (int j = 0; j < 6; j++) {
            channel_out[j] = (int16_t)((raw[j * 2] << 8) | raw[j * 2 + 1]);
        }

        if (mag_raw[0] & 0x01) {
            channel_out[6] = (int16_t)((mag_raw[2] << 8) | mag_raw[1]);
            channel_out[7] = (int16_t)((mag_raw[4] << 8) | mag_raw[3]);
            channel_out[8] = (int16_t)((mag_raw[6] << 8) | mag_raw[5]);
            s->mag_cache[0] = channel_out[6];
            s->mag_cache[1] = channel_out[7];
            s->mag_cache[2] = channel_out[8];
            s->mag_is_fresh = true;
        } else {
            channel_out[6] = s->mag_cache[0];
            channel_out[7] = s->mag_cache[1];
            channel_out[8] = s->mag_cache[2];
        }
    } else if (strcmp(s->name, "BNO055") == 0) {
        uint8_t raw[18];
        iic_read_n_bytes_recovering(s->axi_map, s->i2c_addr, s->data_reg_start, raw, 18);
        for (int j = 0; j < s->num_channels; j++) {
            channel_out[j] = (int16_t)(raw[j * 2] | (raw[j * 2 + 1] << 8));
        }
    } else if (s->is_spi) {
        uint8_t raw[32];
        axi_spi_read(s->axi_map, s->data_reg_start, raw, s->num_channels * 2);
        for (int j = 0; j < s->num_channels; j++) {
            channel_out[j] = (int16_t)((raw[j * 2] << 8) | raw[j * 2 + 1]);
        }
    } else if (s->split_read) {
        uint8_t raw[14];
        iic_read_n_bytes_recovering(s->axi_map, s->i2c_addr, 0x3B, raw, 14);
        for (int j = 0; j < 3; j++) {
            channel_out[j] = (int16_t)((raw[j * 2] << 8) | raw[j * 2 + 1]);
        }
        for (int j = 0; j < 3; j++) {
            channel_out[j + 3] = (int16_t)((raw[8 + j * 2] << 8) | raw[8 + j * 2 + 1]);
        }
    } else {
        uint8_t raw[32];
        iic_read_n_bytes_recovering(s->axi_map, s->i2c_addr, s->data_reg_start, raw, s->num_channels * 2);
        for (int j = 0; j < s->num_channels; j++) {
            channel_out[j] = (int16_t)((raw[j * 2] << 8) | raw[j * 2 + 1]);
        }
    }
}

static void get_sensor_gyro_from_channels(const SensorInstance *s, const int16_t *channel_out, int16_t raw_gyr[3]) {
    if (strcmp(s->name, "BNO055") == 0) {
        raw_gyr[0] = channel_out[6];
        raw_gyr[1] = channel_out[7];
        raw_gyr[2] = channel_out[8];
    } else {
        raw_gyr[0] = channel_out[3];
        raw_gyr[1] = channel_out[4];
        raw_gyr[2] = channel_out[5];
    }
}

/*
    calibrate_imu_biases() — measures each sensor's DC offset (bias) before streaming.

    Gyroscopes always output a small non-zero value even when perfectly still because
    of chip-level imperfections. This is called "gyro drift" or "bias". If we don't
    subtract it, the VQF orientation filter will think the sensor is slowly rotating
    and the 3D model will drift over time.

    Method: take GYRO_BIAS_CALIBRATION_SAMPLES (3000) readings while the sensors
    are completely still, then average them. That average IS the bias. Store it in
    s->gyro_bias and s->acc_bias so fusion_update_from_channels() can subtract it.

    Keep the sensors still during the ~3-second calibration window or the bias
    estimate will include real motion and the fusion will be off.
*/
static void calibrate_imu_biases(HardwareContext *ctx) {
    int16_t raw_channels[16];
    int64_t gyro_sums[6][3] = {0};
    int64_t acc_sums[6][3] = {0};

    if (ctx->active_sensor_count <= 0) return;

    printf("Calibrating accel/gyro bias using %d stationary samples (~3 s). Keep sensors still...\n",
           GYRO_BIAS_CALIBRATION_SAMPLES);
    fflush(stdout);

    for (int sample = 0; sample < GYRO_BIAS_CALIBRATION_SAMPLES; sample++) {
        for (int i = 0; i < ctx->active_sensor_count; i++) {
            SensorInstance *s = &ctx->sensors[i];
            if (s->num_channels < 6) continue;

            memset(raw_channels, 0, sizeof(raw_channels));
            read_sensor_raw_channels(s, raw_channels);

            acc_sums[i][0] += raw_channels[0];
            acc_sums[i][1] += raw_channels[1];
            acc_sums[i][2] += raw_channels[2];

            int16_t raw_gyr[3];
            get_sensor_gyro_from_channels(s, raw_channels, raw_gyr);
            gyro_sums[i][0] += raw_gyr[0];
            gyro_sums[i][1] += raw_gyr[1];
            gyro_sums[i][2] += raw_gyr[2];
        }
        usleep(IMU_BIAS_CALIBRATION_SLEEP_US);
    }

    for (int i = 0; i < ctx->active_sensor_count; i++) {
        SensorInstance *s = &ctx->sensors[i];
        if (s->num_channels < 6) continue;

        s->acc_bias[0] = (int16_t)(acc_sums[i][0] / GYRO_BIAS_CALIBRATION_SAMPLES);
        s->acc_bias[1] = (int16_t)(acc_sums[i][1] / GYRO_BIAS_CALIBRATION_SAMPLES);
        s->acc_bias[2] = (int16_t)(acc_sums[i][2] / GYRO_BIAS_CALIBRATION_SAMPLES);
        s->gyro_bias[0] = (int16_t)(gyro_sums[i][0] / GYRO_BIAS_CALIBRATION_SAMPLES);
        s->gyro_bias[1] = (int16_t)(gyro_sums[i][1] / GYRO_BIAS_CALIBRATION_SAMPLES);
        s->gyro_bias[2] = (int16_t)(gyro_sums[i][2] / GYRO_BIAS_CALIBRATION_SAMPLES);

        printf("Sensor %d (%s) acc bias: [%d, %d, %d] gyro bias: [%d, %d, %d]\n",
               i, s->name,
               s->acc_bias[0], s->acc_bias[1], s->acc_bias[2],
               s->gyro_bias[0], s->gyro_bias[1], s->gyro_bias[2]);
    }
}

static void write_sensor_csv_labels(FILE *fp, const SensorInstance *s, int sensor_index, bool with_fusion) {
    const char *name = s->name;

    if (strcmp(name, "MPU6050") == 0) {
        fprintf(fp,
                ",sensor%d_%s_ax,sensor%d_%s_ay,sensor%d_%s_az"
                ",sensor%d_%s_gx,sensor%d_%s_gy,sensor%d_%s_gz",
                sensor_index, name, sensor_index, name, sensor_index, name,
                sensor_index, name, sensor_index, name, sensor_index, name);
    } else if (strcmp(name, "BNO055") == 0) {
        fprintf(fp,
                ",sensor%d_%s_ax,sensor%d_%s_ay,sensor%d_%s_az"
                ",sensor%d_%s_mx,sensor%d_%s_my,sensor%d_%s_mz"
                ",sensor%d_%s_gx,sensor%d_%s_gy,sensor%d_%s_gz",
                sensor_index, name, sensor_index, name, sensor_index, name,
                sensor_index, name, sensor_index, name, sensor_index, name,
                sensor_index, name, sensor_index, name, sensor_index, name);
    } else {
        fprintf(fp,
                ",sensor%d_%s_ax,sensor%d_%s_ay,sensor%d_%s_az"
                ",sensor%d_%s_gx,sensor%d_%s_gy,sensor%d_%s_gz"
                ",sensor%d_%s_mx,sensor%d_%s_my,sensor%d_%s_mz",
                sensor_index, name, sensor_index, name, sensor_index, name,
                sensor_index, name, sensor_index, name, sensor_index, name,
                sensor_index, name, sensor_index, name, sensor_index, name);
    }

    (void)with_fusion;

    {
        fprintf(fp,
                ",sensor%d_%s_qw,sensor%d_%s_qx,sensor%d_%s_qy,sensor%d_%s_qz",
                sensor_index, name, sensor_index, name, sensor_index, name, sensor_index, name);
    }
}

static void write_csv_header(FILE *fp, HardwareContext *ctx, bool with_fusion) {
    fprintf(fp, "# hardware_stream_hz=%d\n", current_stream_hw_hz());
    fprintf(fp, "# elapsed_seconds = nominal time (sum of 1/hz per sample; matches Open Ephys plugin)\n");
    fprintf(fp, "sample_index,elapsed_seconds");
    for (int i = 0; i < ctx->active_sensor_count; i++) {
        write_sensor_csv_labels(fp, &ctx->sensors[i], i, with_fusion);
    }
    for (int ch = 0; ch < ANALOG_WAVEFORM_CHANNELS; ch++) {
        fprintf(fp, ",analog_input%d", ch + 1);
    }
    fprintf(fp, "\n");
}

static void write_csv_row(FILE *fp, HardwareContext *ctx, bool with_fusion, int sample_index, double elapsed_seconds, const int16_t *frame_buffer) {
    int channel_offset = 0;
    (void)with_fusion;

    fprintf(fp, "%d,%.6f", sample_index, elapsed_seconds);

    for (int i = 0; i < ctx->active_sensor_count; i++) {
        const SensorInstance *s = &ctx->sensors[i];
        int sensor_channels = s->num_channels + 4;

        for (int j = 0; j < sensor_channels; j++) {
            fprintf(fp, ",%d", frame_buffer[channel_offset + j]);
        }

        channel_offset += sensor_channels;
    }

    for (int ch = 0; ch < ANALOG_WAVEFORM_CHANNELS; ch++) {
        fprintf(fp, ",%d", frame_buffer[channel_offset + ch]);
    }

    fprintf(fp, "\n");
}

static void init_frame_layout(HardwareContext *ctx, int base_channels,
                               int *current_channels, int *bytes_per_frame) {
    int next_channels       = base_channels + ctx->active_sensor_count * 4 + ANALOG_WAVEFORM_CHANNELS;
    *current_channels       = next_channels;
    *bytes_per_frame        = next_channels * 2;
    ctx->total_channels     = next_channels;
}

static void update_fusion_state(bool *with_fusion) {
    *with_fusion = fusion_is_enabled();
}

/*
    sensor_decim_interval_for_hz() — computes how many hardware ticks to skip between
    VQF updates for a sensor.

    Example: if hw_hz = 1000 and the sensor's cfg_target_hz = 100, we want to run VQF
    only every 10 ticks. The interval = ceiling(hw_hz / target_hz) = 10.

    Ceiling division (hw + t - 1) / t is used instead of plain hw/t to handle cases
    where the target doesn't divide evenly (e.g. hw=1000, target=300 → interval=4,
    giving ~250 Hz actual, which is the best we can do with whole-tick steps).

    Must stay in sync with init_sensor_decimation(), which uses the same formula.
*/
static int sensor_decim_interval_for_hz(const SensorInstance *s, int hw_hz)
{
    int t = s->cfg_target_hz;
    if (t < 1) t = 1;
    if (t > hw_hz) t = hw_hz;
    int inv = (t >= hw_hz) ? 1 : (hw_hz + t - 1) / t;  /* ceiling division */
    return inv < 1 ? 1 : inv;
}

/*
    fusion_update_from_channels() — runs one VQF update step for a single sensor.

    Before calling fusion_update_sensor() (VQF), we subtract the calibrated bias from
    each axis. This is the key step that prevents drift: without bias removal, the VQF
    filter would integrate a constant offset into the orientation estimate, causing the
    simulated limb to slowly rotate even when the real limb is still.

    We wrap the VQF call in clock_gettime() to measure filter latency in nanoseconds.
    The timing is reported periodically (maybe_report_vqf_stats) so we can verify
    that VQF stays fast enough for the target sample rate.

    The quaternion result (4 × int16, Q15 scaled) is written directly after the raw
    channel data in channel_out. So if num_channels=6 (accel+gyro), the quaternion
    occupies indices [6], [7], [8], [9].
*/
static long long fusion_update_from_channels(SensorInstance *s, int sensor_index, int16_t *channel_out)
{
    /* Subtract calibrated DC offsets so VQF sees zero when the sensor is still */
    int16_t raw_acc[3] = {
        (int16_t)(channel_out[0] - s->acc_bias[0]),
        (int16_t)(channel_out[1] - s->acc_bias[1]),
        (int16_t)(channel_out[2] - s->acc_bias[2]),
    };
    int16_t raw_gyr[3];
    const int16_t *raw_mag = NULL;
    bool mag_is_fresh = false;

    get_sensor_gyro_from_channels(s, channel_out, raw_gyr);

    if (strcmp(s->name, "BNO055") == 0) {
        raw_mag = &channel_out[3];
        mag_is_fresh = true;
    } else if (strcmp(s->name, "MPU9250") == 0 && !s->is_spi) {
        raw_mag = &channel_out[6];
        mag_is_fresh = s->mag_is_fresh;
    } else if (strcmp(s->name, "ICM20948") == 0 && s->is_spi) {
        raw_mag = &channel_out[6];
        mag_is_fresh = s->mag_is_fresh;
    }

    raw_gyr[0] -= s->gyro_bias[0];  /* remove calibrated gyro bias */
    raw_gyr[1] -= s->gyro_bias[1];
    raw_gyr[2] -= s->gyro_bias[2];

    struct timespec vqf_start;
    struct timespec vqf_end;
    clock_gettime(CLOCK_MONOTONIC, &vqf_start);  /* measure VQF execution time */
    fusion_update_sensor(sensor_index, raw_acc, raw_gyr, raw_mag, mag_is_fresh,
                         channel_out + s->num_channels);  /* quaternion written here */
    clock_gettime(CLOCK_MONOTONIC, &vqf_end);
    return timespec_diff_ns(&vqf_start, &vqf_end);  /* return latency in nanoseconds */
}

static void acquire_sensor_samples(
    HardwareContext *ctx,
    bool with_fusion,
    int bytes_per_frame,
    int16_t *frame_buffer,
    long long *vqf_total_ns,
    long long *vqf_max_ns,
    unsigned long long *vqf_call_count
) {
    memset(frame_buffer, 0, bytes_per_frame);

    int current_byte_offset = 0;

    for (int i = 0; i < ctx->active_sensor_count; i++) {
        SensorInstance *s = &ctx->sensors[i];
        int16_t *channel_out = (int16_t*)(((uint8_t*)frame_buffer) + current_byte_offset);
        read_sensor_raw_channels(s, channel_out);

        if (with_fusion) {
            long long vqf_elapsed_ns = fusion_update_from_channels(s, i, channel_out);
            *vqf_total_ns += vqf_elapsed_ns;
            (*vqf_call_count)++;
            if (vqf_elapsed_ns > *vqf_max_ns) {
                *vqf_max_ns = vqf_elapsed_ns;
            }
        }

        current_byte_offset += (s->num_channels + 4) * 2;
    }

    read_analog_waveform_channels((int16_t*)(((uint8_t*)frame_buffer) + current_byte_offset));
}

/*
    Decimation state — one counter and one interval per sensor slot.

    g_decim_counter[i]: counts hardware ticks since the last real read for sensor i.
                         Incremented every tick in sensor_worker().
    g_decim_interval[i]: how many ticks to wait between real reads (computed by
                          sensor_decim_interval_for_hz). E.g. interval=10 means one
                          real VQF update every 10 hardware ticks.
    g_sensor_hold[i]:    the most recent channel data for sensor i. When we skip a tick
                          (counter < interval), we copy the held data into the frame
                          instead of doing a real read, so the PC still gets a full frame.
*/
#define HOLD_SLOTS 6
#define HOLD_INT16 32
static int16_t g_sensor_hold[HOLD_SLOTS][HOLD_INT16];   /* last good reading per sensor */
static int g_decim_counter[HOLD_SLOTS];                  /* ticks since last real read */
static int g_decim_interval[HOLD_SLOTS];                 /* ticks between real reads */

static void init_sensor_decimation(HardwareContext *ctx, int hw_hz)
{
    for (int i = 0; i < HOLD_SLOTS; i++) {
        g_decim_counter[i] = 0;
        g_decim_interval[i] = 1;
        memset(g_sensor_hold[i], 0, sizeof(g_sensor_hold[i]));
    }
    for (int i = 0; i < ctx->active_sensor_count && i < HOLD_SLOTS; i++) {
        g_decim_interval[i] = sensor_decim_interval_for_hz(&ctx->sensors[i], hw_hz);
        g_decim_counter[i] = g_decim_interval[i];
    }
}

typedef struct {
    HardwareContext   *ctx;
    int                sensor_index;
    int16_t           *channel_out;
    bool               with_fusion;
    long long          vqf_total_ns;
    long long          vqf_max_ns;
    unsigned long long vqf_call_count;
} SensorWorkArgs;

typedef struct {
    SensorWorkArgs args;
    pthread_t      thread;
    sem_t          go;
    sem_t          done;
    volatile int   running;
} SensorThreadCtx;

static SensorThreadCtx g_sensor_threads[6];
static int             g_sensor_thread_count = 0;

static void *sensor_worker(void *arg);

static void *sensor_thread_loop(void *arg)
{
    SensorThreadCtx *t = (SensorThreadCtx *)arg;
    while (1) {
        sem_wait(&t->go);
        if (!t->running) break;
        sensor_worker(&t->args);
        sem_post(&t->done);
    }
    return NULL;
}

/*
    sensor_threads_init() — creates N-1 worker threads for parallel sensor reads.

    Threading pattern: with N sensors we create N-1 permanent worker threads.
    Each frame, we send the first N-1 sensors to these threads while the main
    thread reads the last sensor simultaneously. All finish at roughly the same
    time, cutting total read latency by a factor of ~N compared to sequential reads.

    The threads are pre-created here (not per-frame) so there is no thread
    creation overhead inside the hot path. They block on semaphores between frames.
*/
static void sensor_threads_init(HardwareContext *ctx, int n)
{
    g_sensor_thread_count = (n > 1) ? n - 1 : 0;
    for (int i = 0; i < g_sensor_thread_count; i++) {
        g_sensor_threads[i].running = 1;
        sem_init(&g_sensor_threads[i].go,   0, 0);
        sem_init(&g_sensor_threads[i].done, 0, 0);
        g_sensor_threads[i].args.ctx          = ctx;
        g_sensor_threads[i].args.sensor_index = i;
        pthread_create(&g_sensor_threads[i].thread, NULL,
                       sensor_thread_loop, &g_sensor_threads[i]);
    }
}

static void sensor_threads_shutdown(void)
{
    for (int i = 0; i < g_sensor_thread_count; i++) {
        g_sensor_threads[i].running = 0;
        sem_post(&g_sensor_threads[i].go);
        pthread_join(g_sensor_threads[i].thread, NULL);
        sem_destroy(&g_sensor_threads[i].go);
        sem_destroy(&g_sensor_threads[i].done);
    }
    g_sensor_thread_count = 0;
}

/*
    sensor_worker() — reads one sensor for one frame, applying decimation.

    Called on either a worker thread (sensors 0..N-2) or the main thread (sensor N-1).
    Each call either does a REAL read (counter reached interval, VQF runs) or a HOLD
    copy (counter not yet at interval, return last value instead of waking the I2C bus).

    Why hold? Because the decimated sensor must still contribute data to every frame
    (the frame layout is fixed at stream start). Holding the last value is far better
    than sending zeros, which would look like a sensor dropout.
*/
static void *sensor_worker(void *arg)
{
    SensorWorkArgs *a = (SensorWorkArgs *)arg;
    HardwareContext *ctx = a->ctx;
    int i = a->sensor_index;
    SensorInstance *s = &ctx->sensors[i];
    int16_t *channel_out = a->channel_out;
    int slot_ints  = s->num_channels + 4;
    int slot_bytes = slot_ints * 2;

    if (i < HOLD_SLOTS) {
        g_decim_counter[i]++;
        if (g_decim_counter[i] >= g_decim_interval[i]) {
            g_decim_counter[i] = 0;  /* reset counter — time for a real read */
            read_sensor_raw_channels(s, channel_out);

            if (a->with_fusion) {
                long long elapsed = fusion_update_from_channels(s, i, channel_out);
                a->vqf_total_ns += elapsed;
                a->vqf_call_count++;
                if (elapsed > a->vqf_max_ns) a->vqf_max_ns = elapsed;
            }

            if (slot_ints <= HOLD_INT16)
                memcpy(g_sensor_hold[i], channel_out, (size_t)slot_bytes);
        } else {
            if (slot_ints <= HOLD_INT16)
                memcpy(channel_out, g_sensor_hold[i], (size_t)slot_bytes);
        }
    } else {
        read_sensor_raw_channels(s, channel_out);
        if (a->with_fusion) {
            long long elapsed = fusion_update_from_channels(s, i, channel_out);
            a->vqf_total_ns += elapsed;
            a->vqf_call_count++;
            if (elapsed > a->vqf_max_ns) a->vqf_max_ns = elapsed;
        }
    }

    return NULL;
}

static void acquire_sensor_samples_decimated(
    HardwareContext *ctx,
    bool with_fusion,
    int bytes_per_frame,
    int16_t *frame_buffer,
    long long *vqf_total_ns,
    long long *vqf_max_ns,
    unsigned long long *vqf_call_count
) {
    memset(frame_buffer, 0, bytes_per_frame);

    int n = ctx->active_sensor_count;
    if (n <= 0) return;

    /* Pre-compute each sensor's output pointer into frame_buffer */
    int16_t *sensor_ptrs[6];
    {
        int byte_off = 0;
        for (int i = 0; i < n; i++) {
            sensor_ptrs[i] = (int16_t *)(((uint8_t *)frame_buffer) + byte_off);
            byte_off += (ctx->sensors[i].num_channels + 4) * 2;
        }
    }

    if (n == 1) {
        /* Single sensor — no thread overhead */
        SensorWorkArgs a = { ctx, 0, sensor_ptrs[0], with_fusion, 0, 0, 0 };
        sensor_worker(&a);
        *vqf_total_ns   += a.vqf_total_ns;
        *vqf_call_count += a.vqf_call_count;
        if (a.vqf_max_ns > *vqf_max_ns) *vqf_max_ns = a.vqf_max_ns;
    } else {
        /* Signal each pre-created worker thread with fresh args for this iteration */
        for (int i = 0; i < n - 1; i++) {
            g_sensor_threads[i].args.channel_out    = sensor_ptrs[i];
            g_sensor_threads[i].args.with_fusion    = with_fusion;
            g_sensor_threads[i].args.vqf_total_ns   = 0;
            g_sensor_threads[i].args.vqf_max_ns     = 0;
            g_sensor_threads[i].args.vqf_call_count = 0;
            sem_post(&g_sensor_threads[i].go);
        }

        /* Run the last sensor on the calling thread simultaneously */
        int last = n - 1;
        SensorWorkArgs a = { ctx, last, sensor_ptrs[last], with_fusion, 0, 0, 0 };
        sensor_worker(&a);

        /* Wait for all worker threads then merge VQF stats */
        for (int i = 0; i < n - 1; i++) {
            sem_wait(&g_sensor_threads[i].done);
            *vqf_total_ns   += g_sensor_threads[i].args.vqf_total_ns;
            *vqf_call_count += g_sensor_threads[i].args.vqf_call_count;
            if (g_sensor_threads[i].args.vqf_max_ns > *vqf_max_ns)
                *vqf_max_ns = g_sensor_threads[i].args.vqf_max_ns;
        }
        *vqf_total_ns   += a.vqf_total_ns;
        *vqf_call_count += a.vqf_call_count;
        if (a.vqf_max_ns > *vqf_max_ns) *vqf_max_ns = a.vqf_max_ns;
    }

    // read_analog_waveform_channels((int16_t *)(((uint8_t *)frame_buffer) + current_byte_offset));
}

static void warn_if_sample_loop_slow_us(const struct timespec *t_start, const struct timespec *t_end)
{
    long elapsed_us = (long)((t_end->tv_sec - t_start->tv_sec) * 1000000L +
                             (t_end->tv_nsec - t_start->tv_nsec) / 1000L);
    if (elapsed_us > 900) { /* approaching 1 ms limit */
        printf("WARNING: loop took %ld us\n", elapsed_us);
    }
}

static void maybe_report_vqf_stats(
    bool with_fusion,
    int sample_number,
    long long *vqf_total_ns,
    long long *vqf_max_ns,
    unsigned long long *vqf_call_count
) {
    int report_interval = current_stream_hw_hz();
    if (with_fusion && *vqf_call_count > 0 && report_interval > 0 && (sample_number % report_interval) == 0) {
        double avg_us = (double)(*vqf_total_ns) / (double)(*vqf_call_count) / 1000.0;
        double max_us = (double)(*vqf_max_ns) / 1000.0;
        printf("VQF filter time over last %llu calls: avg %.2f us, max %.2f us\n",
               *vqf_call_count, avg_us, max_us);
        *vqf_total_ns = 0;
        *vqf_max_ns = 0;
        *vqf_call_count = 0;
    }
}

/*
 * Preset index -> full-scale range mapping (same for MPU6050, MPU9250):
 *   0 = ±2g  / ±250 °/s   (ACCEL_CONFIG/GYRO_CONFIG bits [4:3] = 00)
 *   1 = ±4g  / ±500 °/s   (bits = 01)
 *   2 = ±8g  / ±1000 °/s  (bits = 10)
 *   3 = ±16g / ±2000 °/s  (bits = 11)
 */
static const uint8_t MPU_ACC_REG[4] = { 0x00, 0x08, 0x10, 0x18 }; /* reg 0x1C */
static const uint8_t MPU_GYR_REG[4] = { 0x00, 0x08, 0x10, 0x18 }; /* reg 0x1B */

/*
 * ICM20948 Bank-2 ACCEL_CONFIG (0x14) / GYRO_CONFIG_1 (0x01):
 *   ACCEL_FS_SEL in bits [2:1], GYRO_FS_SEL in bits [2:1]
 *   FCHOICE left 0 (bypass DLPF for max rate).
 */
static const uint8_t ICM_ACC_REG[4] = { 0x00, 0x02, 0x04, 0x06 }; /* Bank2 0x14 */
static const uint8_t ICM_GYR_REG[4] = { 0x00, 0x02, 0x04, 0x06 }; /* Bank2 0x01 */

/*
 * BNO055 ACC_Config (0x08) in CONFIG mode:
 *   bits [1:0] = range, bits [4:2] = bandwidth (62.5 Hz = 011).
 * BNO055 GYR_Config_0 (0x0A) in CONFIG mode:
 *   bits [2:0] = range (inverted order), bits [5:3] = bandwidth (32 Hz = 111).
 *   Preset 0->±250 (reg 3), preset 3->±2000 (reg 0).
 */
static const uint8_t BNO_ACC_REG[4] = { 0x0C, 0x0D, 0x0E, 0x0F }; /* 0x08 */
static const uint8_t BNO_GYR_REG[4] = { 0x3B, 0x3A, 0x39, 0x38 }; /* 0x0A */

/*
 * Write the sensor chip's internal ODR (Output Data Rate) register so the
 * hardware produces samples at target_hz instead of its power-on default.
 *
 * MPU6050 / MPU9250
 *   CONFIG  (0x1A) = 0x01  — enable DLPF so gyro output rate = 1 kHz
 *   SMPLRT_DIV (0x19)      — Sample Rate = 1000 / (1 + div)
 *
 * ICM20948 (Bank 2)
 *   GYRO_SMPLRT_DIV  (0x00) — ODR = 1100 / (1 + div),  8-bit
 *   ACCEL_SMPLRT_DIV (0x10/0x11) — ODR = 1125 / (1 + div), 12-bit
 *
 * BNO055
 *   Fixed at 100 Hz in NDOF mode — no register write possible.
 */
static void apply_sensor_odr(SensorInstance *s, int target_hz)
{
    if (target_hz < 1) target_hz = 1;

    if (strcmp(s->name, "MPU6050") == 0 || strcmp(s->name, "MPU9250") == 0) {
        int div = (1000 / target_hz) - 1;
        if (div < 0)   div = 0;
        if (div > 255) div = 255;
        uint8_t smplrt_div = (uint8_t)div;
        int actual_hz = 1000 / (div + 1);

        if (s->is_spi) {
            axi_spi_write(s->axi_map, 0x1A, 0x01); /* DLPF on → 1 kHz gyro rate */
            axi_spi_write(s->axi_map, 0x19, smplrt_div);
        } else {
            axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x1A, 0x01);
            axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x19, smplrt_div);
        }
        printf("  %s ODR: SMPLRT_DIV=%d -> ~%d Hz\n", s->name, div, actual_hz);

    } else if (strcmp(s->name, "ICM20948") == 0) {
        int gyro_div  = (1100  / target_hz) - 1;
        int accel_div = (1125  / target_hz) - 1;
        if (gyro_div  < 0)    gyro_div  = 0;
        if (gyro_div  > 255)  gyro_div  = 255;
        if (accel_div < 0)    accel_div = 0;
        if (accel_div > 4095) accel_div = 4095;

        int actual_gyro_hz  = 1100  / (gyro_div  + 1);
        int actual_accel_hz = 1125  / (accel_div + 1);

        if (s->is_spi) {
            icm20948_spi_select_bank(s->axi_map, ICM20948_BANK_2);
            axi_spi_write(s->axi_map, 0x00, (uint8_t)gyro_div);
            axi_spi_write(s->axi_map, 0x10, (uint8_t)((accel_div >> 8) & 0x0F));
            axi_spi_write(s->axi_map, 0x11, (uint8_t)(accel_div & 0xFF));
            icm20948_spi_select_bank(s->axi_map, ICM20948_BANK_0);
        } else {
            axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x7F, (uint8_t)(ICM20948_BANK_2 << 4));
            axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x00, (uint8_t)gyro_div);
            axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x10, (uint8_t)((accel_div >> 8) & 0x0F));
            axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x11, (uint8_t)(accel_div & 0xFF));
            axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x7F, (uint8_t)(ICM20948_BANK_0 << 4));
        }
        printf("  %s ODR: gyro ~%d Hz, accel ~%d Hz\n", s->name, actual_gyro_hz, actual_accel_hz);

    } else if (strcmp(s->name, "BNO055") == 0) {
        /* NDOF fusion engine owns the ODR (fixed ~100 Hz); no register to write. */
        printf("  BNO055 ODR fixed at ~100 Hz in NDOF mode\n");
    }
}

static void apply_sensor_cfg_acc(SensorInstance *s, int preset)
{
    if (preset < 0 || preset > 3) return;

    if (strcmp(s->name, "MPU6050") == 0) {
        axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x1C, MPU_ACC_REG[preset]);
        printf("  %s ACC preset %d -> reg 0x1C = 0x%02X\n", s->name, preset, MPU_ACC_REG[preset]);
    } else if (strcmp(s->name, "MPU9250") == 0) {
        if (s->is_spi)
            axi_spi_write(s->axi_map, 0x1C, MPU_ACC_REG[preset]);
        else
            axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x1C, MPU_ACC_REG[preset]);
        printf("  %s ACC preset %d -> reg 0x1C = 0x%02X\n", s->name, preset, MPU_ACC_REG[preset]);
    } else if (strcmp(s->name, "ICM20948") == 0) {
        if (s->is_spi) {
            icm20948_spi_select_bank(s->axi_map, ICM20948_BANK_2);
            axi_spi_write(s->axi_map, 0x14, ICM_ACC_REG[preset]);
            icm20948_spi_select_bank(s->axi_map, ICM20948_BANK_0);
        } else {
            axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x7F, (uint8_t)(ICM20948_BANK_2 << 4));
            axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x14, ICM_ACC_REG[preset]);
            axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x7F, (uint8_t)(ICM20948_BANK_0 << 4));
        }
        printf("  %s ACC preset %d -> Bank2 reg 0x14 = 0x%02X\n", s->name, preset, ICM_ACC_REG[preset]);
    } else if (strcmp(s->name, "BNO055") == 0) {
        axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x3D, 0x00); /* CONFIG mode */
        usleep(25000);
        axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x08, BNO_ACC_REG[preset]);
        usleep(10000);
        axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x3D, 0x0C); /* NDOF mode */
        usleep(25000);
        printf("  %s ACC preset %d -> reg 0x08 = 0x%02X\n", s->name, preset, BNO_ACC_REG[preset]);
    }
}

static void apply_sensor_cfg_gyr(SensorInstance *s, int preset)
{
    if (preset < 0 || preset > 3) return;

    if (strcmp(s->name, "MPU6050") == 0) {
        axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x1B, MPU_GYR_REG[preset]);
        printf("  %s GYR preset %d -> reg 0x1B = 0x%02X\n", s->name, preset, MPU_GYR_REG[preset]);
    } else if (strcmp(s->name, "MPU9250") == 0) {
        if (s->is_spi)
            axi_spi_write(s->axi_map, 0x1B, MPU_GYR_REG[preset]);
        else
            axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x1B, MPU_GYR_REG[preset]);
        printf("  %s GYR preset %d -> reg 0x1B = 0x%02X\n", s->name, preset, MPU_GYR_REG[preset]);
    } else if (strcmp(s->name, "ICM20948") == 0) {
        if (s->is_spi) {
            icm20948_spi_select_bank(s->axi_map, ICM20948_BANK_2);
            axi_spi_write(s->axi_map, 0x01, ICM_GYR_REG[preset]);
            icm20948_spi_select_bank(s->axi_map, ICM20948_BANK_0);
        } else {
            axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x7F, (uint8_t)(ICM20948_BANK_2 << 4));
            axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x01, ICM_GYR_REG[preset]);
            axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x7F, (uint8_t)(ICM20948_BANK_0 << 4));
        }
        printf("  %s GYR preset %d -> Bank2 reg 0x01 = 0x%02X\n", s->name, preset, ICM_GYR_REG[preset]);
    } else if (strcmp(s->name, "BNO055") == 0) {
        axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x3D, 0x00); /* CONFIG mode */
        usleep(25000);
        axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x0A, BNO_GYR_REG[preset]);
        usleep(10000);
        axi_iic_write_byte(s->axi_map, s->i2c_addr, 0x3D, 0x0C); /* NDOF mode */
        usleep(25000);
        printf("  %s GYR preset %d -> reg 0x0A = 0x%02X\n", s->name, preset, BNO_GYR_REG[preset]);
    }
}

static void fusion_config_for_sensor(const SensorInstance *s, int hw_hz, FusionSensorConfig *cfg)
{
    FusionSensorType ftype;
    if      (strcmp(s->name, "MPU6050")  == 0) ftype = FUSION_SENSOR_TYPE_MPU6050;
    else if (strcmp(s->name, "MPU9250")  == 0) ftype = FUSION_SENSOR_TYPE_MPU9250;
    else if (strcmp(s->name, "ICM20948") == 0) ftype = FUSION_SENSOR_TYPE_ICM20948;
    else if (strcmp(s->name, "BNO055")   == 0) ftype = FUSION_SENSOR_TYPE_BNO055;
    else                                        ftype = FUSION_SENSOR_TYPE_GENERIC;
    bool has_mag = (strcmp(s->name, "MPU9250")  == 0 && !s->is_spi) ||
                   (strcmp(s->name, "ICM20948") == 0) ||
                   (strcmp(s->name, "BNO055")   == 0);
    fusion_get_default_sensor_config(ftype, has_mag, cfg);
    /* Decimated sensors only reach VQF every Nth hardware tick, so VQF must
       integrate with the per-sensor effective rate, not the tick rate. */
    cfg->imu_sample_rate_hz = (float)hw_hz / (float)sensor_decim_interval_for_hz(s, hw_hz);
}

static void reinit_fusion_for_hz(HardwareContext *ctx, float hz)
{
    int hw_hz = clamp_stream_hw_hz((int)hz);
    fusion_shutdown();
    fusion_init(ctx->active_sensor_count, (float)hw_hz);
    for (int i = 0; i < ctx->active_sensor_count; i++) {
        FusionSensorConfig cfg;
        fusion_config_for_sensor(&ctx->sensors[i], hw_hz, &cfg);
        fusion_register_sensor_ex(i, &cfg);
    }
}

/*
    process_stream_commands() — non-blocking poll for ASCII commands on the TCP socket.

    Called once per frame loop. Uses MSG_DONTWAIT so it returns immediately if no
    command is waiting (returns EAGAIN), keeping the stream loop from blocking.

    Commands understood:
      STOP           — flush/close recording files, break the stream loop.
      RECORD ON      — start writing frames to SD card (bin + csv files).
      RECORD OFF     — stop writing; flush any buffered data.
      FILTER ON/OFF  — enable/disable VQF sensor fusion.
      AIN_GAIN:<f>   — set analog input gain (float).
      AOUT:<v>       — set analog output voltage (float).
      FREQ:<hz>      — change hardware tick rate; also resets decimation intervals
                       and re-initialises VQF for the new rate so filter continuity
                       is maintained.
      CFG <si> ACC <0-3>   — accelerometer full-scale range preset for sensor si.
      CFG <si> GYR <0-3>   — gyroscope full-scale range preset for sensor si.
      CFG <si> SRATE <hz>  — per-sensor effective sample rate (ODR + decimation update).

    Returns: 0 = continue streaming, 1 = client sent STOP, -1 = TCP disconnected.
*/
static int process_stream_commands(
    int client_fd,
    HardwareContext *ctx,
    FILE **bin_file,
    FILE **csv_file,
    bool *record,
    int *buf_idx,
    uint8_t *sd_write_buffer,
    int buffered_bytes_per_frame
) {
    char cmd[256];
    int n = 0;

    while ((n = recv(client_fd, cmd, sizeof(cmd) - 1, MSG_DONTWAIT)) > 0) {
        cmd[n] = '\0';

        if (strstr(cmd, "STOP")) {
            if (*record && *bin_file != NULL && *buf_idx > 0) {
                fwrite(sd_write_buffer, 1, buffered_bytes_per_frame * (*buf_idx), *bin_file);
                fflush(*bin_file);
            }
            if (*csv_file != NULL) fflush(*csv_file);
            if (*bin_file != NULL) fclose(*bin_file);
            if (*csv_file != NULL) fclose(*csv_file);
            *bin_file = NULL;
            *csv_file = NULL;
            return 1;
        }

        if (strstr(cmd, "RECORD ON")) {
            *record = true;
        }

        if (strstr(cmd, "FILTER ON") || strstr(cmd, "FUSION ON")) {
            fusion_set_enabled(true);
            printf("Filter enabled. Next frame will include filtered/VQF values.\n");
        }

        if (strstr(cmd, "FILTER OFF") || strstr(cmd, "FUSION OFF")) {
            fusion_set_enabled(false);
            printf("Filter disabled. Next frame will zero filtered/VQF channels.\n");
        }

        if (strstr(cmd, "AIN_GAIN:")) {
            float gain = strtof(strstr(cmd, "AIN_GAIN:") + 9, NULL);
            printf("Analog input gain set to %.2f.\n", gain);
        }

        if (strstr(cmd, "AOUT:")) {
            float voltage = strtof(strstr(cmd, "AOUT:") + 5, NULL);
            printf("Analog output voltage set to %.2f V.\n", voltage);
        }

        if (strstr(cmd, "FREQ:")) {
            /* FREQ: changes the hardware tick rate mid-stream. We must also:
               - Apply the new ODR to all sensors (write hardware registers).
               - Reset decimation intervals because the number of ticks per second changed.
               - Reinitialise VQF because VQF's internal sample-rate assumption changed.
                 Without reinit, VQF would integrate with the wrong time step and drift. */
            int frequency_hz = atoi(strstr(cmd, "FREQ:") + 5);
            g_stream_hw_hz = clamp_stream_hw_hz(frequency_hz);
            for (int si = 0; si < ctx->active_sensor_count; si++) {
                ctx->sensors[si].cfg_target_hz = g_stream_hw_hz;
                apply_sensor_odr(&ctx->sensors[si], g_stream_hw_hz);
            }
            init_sensor_decimation(ctx, g_stream_hw_hz);
            reinit_fusion_for_hz(ctx, (float)g_stream_hw_hz);
            printf("Hardware stream tick rate set to %d Hz.\n", g_stream_hw_hz);
        }

        if (strstr(cmd, "CFG ")) {
            /* CFG changes sensor hardware registers on the fly.
               ACC preset 0-3 → ±2g / ±4g / ±8g / ±16g (writes to chip ACCEL_CONFIG register).
               GYR preset 0-3 → ±250 / ±500 / ±1000 / ±2000 °/s (writes to GYRO_CONFIG register).
               SRATE <hz>     → per-sensor effective rate: applies ODR to chip registers AND
                                recomputes decimation interval. */
            int si = -1, val = -1;
            if (sscanf(cmd, "CFG %d ACC %d", &si, &val) == 2) {
                if (si >= 0 && si < ctx->active_sensor_count && val >= 0 && val <= 3) {
                    ctx->sensors[si].cfg_acc_id = val;
                    apply_sensor_cfg_acc(&ctx->sensors[si], val);
                    printf("CFG sensor %d ACC preset %d applied\n", si, val);
                }
            } else if (sscanf(cmd, "CFG %d GYR %d", &si, &val) == 2) {
                if (si >= 0 && si < ctx->active_sensor_count && val >= 0 && val <= 3) {
                    ctx->sensors[si].cfg_gyr_id = val;
                    apply_sensor_cfg_gyr(&ctx->sensors[si], val);
                    printf("CFG sensor %d GYR preset %d applied\n", si, val);
                }
            } else if (sscanf(cmd, "CFG %d SRATE %d", &si, &val) == 2) {
                if (si >= 0 && si < ctx->active_sensor_count && val >= 1) {
                    ctx->sensors[si].cfg_target_hz = val;
                    if (ctx->sensors[si].cfg_target_hz > g_stream_hw_hz)
                        ctx->sensors[si].cfg_target_hz = g_stream_hw_hz;
                    apply_sensor_odr(&ctx->sensors[si], ctx->sensors[si].cfg_target_hz);
                    printf("CFG sensor %d SRATE target %d Hz (ODR + decimation updated)\n",
                           si, ctx->sensors[si].cfg_target_hz);
                    init_sensor_decimation(ctx, g_stream_hw_hz);
                    FusionSensorConfig cfg;
                    fusion_config_for_sensor(&ctx->sensors[si], current_stream_hw_hz(), &cfg);
                    fusion_register_sensor_ex(si, &cfg);
                }
            }
        }

        if (strstr(cmd, "RECORD OFF")) {
            *record = false;
            if (*bin_file != NULL && *buf_idx > 0) {
                fwrite(sd_write_buffer, 1, buffered_bytes_per_frame * (*buf_idx), *bin_file);
                fflush(*bin_file);
                *buf_idx = 0;
            }
            if (*csv_file != NULL) fflush(*csv_file);
            printf("Recording stopped.\n");
        }
    }

    if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
        if (*record && *bin_file != NULL && *buf_idx > 0) {
            fwrite(sd_write_buffer, 1, buffered_bytes_per_frame * (*buf_idx), *bin_file);
            fflush(*bin_file);
        }
        if (*csv_file != NULL) fflush(*csv_file);
        if (*bin_file != NULL) fclose(*bin_file);
        if (*csv_file != NULL) fclose(*csv_file);
        *bin_file = NULL;
        *csv_file = NULL;
        return -1;
    }

    return 0;
}

static int run_timed_csv_capture(HardwareContext *ctx, int base_channels, int duration_seconds, const char *csv_path) {
    int capture_hz = current_stream_hw_hz();
    uint32_t ticks_per_sample = ticks_for_stream_hz(capture_hz);
    const int max_total_channels = base_channels + ctx->active_sensor_count * 4 + ANALOG_WAVEFORM_CHANNELS;
    const int max_bytes_per_frame = max_total_channels * 2;
    const int total_samples = duration_seconds * capture_hz;
    int16_t *frame_buffer = (int16_t *)malloc(max_bytes_per_frame);
    bool with_fusion = fusion_is_enabled();
    int current_channels = 0;
    int bytes_per_frame  = 0;
    init_frame_layout(ctx, base_channels, &current_channels, &bytes_per_frame);
    long long vqf_total_ns = 0;
    long long vqf_max_ns = 0;
    unsigned long long vqf_call_count = 0;
    double csv_elapsed = 0.0;
    FILE *csv_file = NULL;

    if (frame_buffer == NULL) {
        return -1;
    }

    csv_file = fopen(csv_path, "w");
    if (csv_file == NULL) {
        free(frame_buffer);
        perror("Failed to open CSV file");
        return -1;
    }

    write_csv_header(csv_file, ctx, with_fusion);
    *ctx->gpio_reset = 1; usleep(1); *ctx->gpio_reset = 0;
    uint32_t last_counter = *ctx->gpio_counter;

    printf("Standalone CSV capture running for %d seconds (%d samples)...\n",
           duration_seconds, total_samples);
    printf("Press Ctrl+C to stop early and save the CSV collected so far.\n");

    for (int sample_index = 0; sample_index < total_samples; sample_index++) {
        if (stop_requested) {
            printf("\nStop requested. Finishing capture early at sample %d.\n", sample_index);
            break;
        }

        while (1) {
            uint32_t now = *ctx->gpio_counter;
            if ((now - last_counter) >= ticks_per_sample) {
                last_counter += ticks_per_sample;
                break;
            }
            if (stop_requested) {
                break;
            }
            usleep(100);
        }

        if (stop_requested) {
            printf("\nStop requested while waiting for next sample. Finishing capture early.\n");
            break;
        }

        struct timespec t_start, t_end;
        clock_gettime(CLOCK_MONOTONIC, &t_start);
        update_fusion_state(&with_fusion);
        acquire_sensor_samples(ctx, with_fusion, bytes_per_frame, frame_buffer,
                               &vqf_total_ns, &vqf_max_ns, &vqf_call_count);
        clock_gettime(CLOCK_MONOTONIC, &t_end);
        warn_if_sample_loop_slow_us(&t_start, &t_end);

        write_csv_row(csv_file, ctx, with_fusion, sample_index, csv_elapsed, frame_buffer);
        csv_elapsed += 1.0 / (double)current_stream_hw_hz();
        maybe_report_vqf_stats(with_fusion, sample_index + 1, &vqf_total_ns, &vqf_max_ns, &vqf_call_count);

        if (((sample_index + 1) % capture_hz) == 0) {
            int elapsed_whole_seconds = (sample_index + 1) / capture_hz;
            int remaining_seconds = duration_seconds - elapsed_whole_seconds;
            if (remaining_seconds < 0) remaining_seconds = 0;
            printf("Capture progress: %d/%d seconds complete.\n", elapsed_whole_seconds, duration_seconds);
        }
    }

    fflush(csv_file);
    fclose(csv_file);
    free(frame_buffer);
    return 0;
}

/*
    identify_and_add_sensor() — configures a newly discovered sensor and adds it to ctx.

    The `id` byte comes from the WHO_AM_I register of the IMU chip. Each chip has a
    unique ID so we can identify the chip model from a single byte read:
      0x68 = MPU6050  (6-axis: accel + gyro only, I2C only)
      0x71 = MPU9250  (9-axis: accel + gyro + magnetometer AK8963)
      0xEA = ICM20948 (9-axis: accel + gyro + magnetometer AK09916, newer chip)
      0xA0 = BNO055   (9-axis with internal fusion, I2C only)

    For MPU9250 (I2C path): we set register 0x37 bit 1 (I2C_BYPASS_EN) to enable
    "bypass mode", which connects the auxiliary I2C bus directly to the main I2C pins.
    This lets us talk to the AK8963 magnetometer as a normal I2C slave without going
    through the MPU's I2C master engine.
*/
static void identify_and_add_sensor(HardwareContext *ctx, void *map, uint8_t id, uint8_t addr, bool is_spi) {
    SensorInstance *s = &ctx->sensors[ctx->active_sensor_count];
    s->axi_map = map;
    s->i2c_addr = addr;
    s->is_spi = is_spi;
    s->active = true;

    if (id == 0x68 && !is_spi) { // MPU6050 — 6-axis (accel+gyro), I2C only
        strcpy(s->name, "MPU6050");
        s->split_read = true;
        s->num_channels = 6;
        s->data_reg_start = 0x3B;
        axi_iic_write_byte(map, addr, 0x6B, 0x80);
        usleep(10000); // 10ms Reset delay
        axi_iic_write_byte(map, addr, 0x6B, 0x01);
        usleep(5000);  // 5ms Wake delay
    }
    else if (id == 0x71) { // MPU9250 — 9-axis (accel+gyro+mag); SPI or I2C
        strcpy(s->name, "MPU9250");
        s->split_read = false;
        s->num_channels = 9;
        s->data_reg_start = 0x3B;

        if (is_spi) {
            axi_spi_write(map, 0x6B, 0x01); // Wake up Accel/Gyro
            usleep(5000);
            printf("  -> MPU9250 (SPI) initialized (6-axis only for now).\n");
        } else {
            // --- I2C PATH ---
            axi_iic_write_byte(map, addr, 0x6B, 0x01); // Wake
            usleep(5000);

            /* Enable I2C bypass mode so the AK8963 magnetometer (at 0x0C) is visible
               directly on the external I2C bus. Reg 0x6A bit 5 = 0 disables I2C master.
               Reg 0x37 bit 1 = 1 connects the auxiliary I2C bus to the external pins. */
            axi_iic_write_byte(map, addr, 0x6A, 0x00); /* disable MPU I2C master */
            axi_iic_write_byte(map, addr, 0x37, 0x02); /* enable I2C bypass for AK8963 */
            usleep(5000);

            uint8_t ak_wia = 0;
            axi_iic_read_n_bytes(map, 0x0C, 0x00, &ak_wia, 1);
            if (ak_wia != 0x48) {
                printf("  WARNING: AK8963 WIA = 0x%02X (expected 0x48) - bypass mode may have failed\n", ak_wia);
            } else {
                printf("  -> AK8963 verified at 0x0C (WIA = 0x48)\n");
            }

            // This call is safe here because we are actually on an I2C bus
            axi_iic_write_byte(map, 0x0C, 0x0A, 0x16);
            printf("  -> MPU9250 (I2C) initialized (9-axis enabled).\n");
        }
    }
    else if (id == 0xEA) { // ICM20948 — 9-axis (accel+gyro+mag AK09916); SPI or I2C
        strcpy(s->name, "ICM20948");
        s->split_read = false;
        s->num_channels = 9;
        s->data_reg_start = 0x2D;

        if (is_spi) {
            axi_spi_write(map, 0x7F, 0x00); // Bank 0
            axi_spi_write(map, 0x06, 0x01); // Wake
            usleep(10000);
            icm20948_spi_configure_mag_passthrough(map);
            printf("  -> ICM20948 (SPI) initialized (9-axis mag auto-read enabled).\n");
        } else {
            axi_iic_write_byte(map, addr, 0x7F, 0x00); // Bank 0
            axi_iic_write_byte(map, addr, 0x06, 0x01); // Wake
            usleep(10000);
        }
    }
    else if (id == 0xA0 && !is_spi) { // BNO055 — 9-axis with onboard fusion engine; I2C only
        strcpy(s->name, "BNO055");
        s->split_read = false;
        s->num_channels = 9;
        s->data_reg_start = 0x08;
        axi_iic_write_byte(map, addr, 0x3D, 0x0C); // NDOF Mode
        usleep(20000);
    }

    ctx->total_channels += s->num_channels;
    s->cfg_acc_id = 0;
    s->cfg_gyr_id = 0;
    s->cfg_target_hz = DESIRED_SAMPLE_RATE_HZ;
    apply_sensor_odr(s, DESIRED_SAMPLE_RATE_HZ);
    ctx->active_sensor_count++;
}

// --- Init Hardware (AXI Sweep) ---
static int init_hardware(HardwareContext *ctx) {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("Failed to open /dev/mem");
        return -1;
    }

    // Register the handler for the SIGALRM signal
    signal(SIGALRM, watchdog_handler);

    // Map the GPIO for the hardware timer/counter
    ctx->axi_gpio_map = mmap(NULL, RANGE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, AXI_GPIO_ADDRESS);
    if (ctx->axi_gpio_map == MAP_FAILED) {
        close(fd);
        return -1;
    }
    ctx->gpio_counter = (volatile uint32_t *)(ctx->axi_gpio_map + 0x0);
    ctx->gpio_reset   = (volatile uint32_t *)(ctx->axi_gpio_map + 0x8);

    uint32_t i2c_bases[] = {0x41600000, 0x41610000, 0x41620000, 0x41630000};
    ctx->active_sensor_count = 0;
    ctx->total_channels = 0;

    for (int i = 0; i < 4; i++) {
        printf("Probing AXI I2C Slot %d at 0x%08X...\n", i, i2c_bases[i]);
        fflush(stdout);

        void* map = mmap(NULL, IIC_RANGE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, i2c_bases[i]);
        if (map == MAP_FAILED) continue;

        /* Clear stuck state from a previous session (motion glitch / abrupt stop). */
        axi_iic_force_reset(map);
        usleep(2000);

        uint8_t id_mpu = 0, id_icm = 0, id_bno = 0;
        bool sensor_found = false;

        // --- PHASE 1: Probe for 0x68 (MPU / ICM) ---
        if (sigsetjmp(watchdog_bucket, 1) == 0) {
            alarm(1); // 1-second fuse for 0x68
            axi_iic_initialize(map);

            // This will hang if 0x68 is empty, triggering the else block below
            axi_iic_write_byte(map, 0x68, 0x7F, 0x00);
            usleep(1000);
            axi_iic_read_n_bytes(map, 0x68, 0x75, &id_mpu, 1);
            axi_iic_read_n_bytes(map, 0x68, 0x00, &id_icm, 1);

            alarm(0); // Success, turn off alarm

            if (id_mpu == 0x68) {
                printf("  -> Found MPU6050!\n");
                identify_and_add_sensor(ctx, map, id_mpu, 0x68, false);
                sensor_found = true;
            } else if (id_icm == 0xEA) {
                printf("  -> Found ICM20948!\n");
                identify_and_add_sensor(ctx, map, id_icm, 0x68, false);
                sensor_found = true;
            }
            else if (id_mpu == 0x71) {
                printf("  -> Found MPU9250!\n");
                identify_and_add_sensor(ctx, map, id_mpu, 0x68, false);
                sensor_found = true;
            }
        } else {
            alarm(0); // Clear the alarm and safely proceed to Phase 2.
        }

        if (sensor_found) continue; // If we found one, move to the next physical slot!

        // --- PHASE 2: Probe for 0x28 (BNO055) ---
        // We only reach this code if Phase 1 triggered a hang (meaning no 0x68 device)
        if (sigsetjmp(watchdog_bucket, 1) == 0) {
            alarm(1);

            axi_iic_initialize(map);
            // Probe BNO055
            axi_iic_read_n_bytes(map, 0x28, 0x00, &id_bno, 1);
            alarm(0); // Success, turn off alarm

            if (id_bno == 0xA0) {
                printf("  -> Found BNO055!\n");
                identify_and_add_sensor(ctx, map, id_bno, 0x28, false);
            } else {
                printf("  -> Slot %d is completely empty.\n", i);
                munmap(map, IIC_RANGE);
            }
        } else {
            alarm(0);
            printf("  -> Slot %d is empty (Bus hung on all addresses).\n", i);
            munmap(map, IIC_RANGE);
        }
    }
    // --- PHASE 3: SPI SENSOR DISCOVERY ---
    printf("\n--- Starting SPI Discovery ---\n");
    fflush(stdout);

    uint32_t spi_bases[] = {0x41E00000, 0x41E10000};
    int num_spi_slots = sizeof(spi_bases) / sizeof(spi_bases[0]);

    for (int i = 0; i < num_spi_slots; i++) {
        printf("Probing AXI SPI Slot %d at 0x%08X... ", i, spi_bases[i]);
        fflush(stdout);

        void* map = mmap(NULL, IIC_RANGE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, spi_bases[i]);
        if (map == MAP_FAILED) {
            printf("[!] mmap failed.\n");
            continue;
        }

        axi_spi_initialize(map);
        uint8_t id_val = 0;

        axi_spi_read(map, 0x75, &id_val, 1); 

        if (id_val == 0x71) {
            printf("Found MPU9250 on SPI! (ID: 0x%02X)\n", id_val);
            identify_and_add_sensor(ctx, map, id_val, 0x00, true);
        } 
        else {
            axi_spi_write(map, 0x7F, 0x00); 
            usleep(1000);
            axi_spi_read(map, 0x00, &id_val, 1);

            if (id_val == 0xEA) {
                printf("Found ICM20948 on SPI! (ID: 0x%02X)\n", id_val);
                identify_and_add_sensor(ctx, map, id_val, 0x00, true);
            } else {
                printf("Empty (Returned 0x%02X at 0x00 and 0x75)\n", id_val);
                munmap(map, IIC_RANGE);
            }
        }
    }

    close(fd);
    printf("--- Discovery Complete (Active Sensors: %d) ---\n", ctx->active_sensor_count);
    return 0;
}

// --- Streaming Logic ---
/** Discard any bytes left in the TCP RX queue after a stream ends so the next
 *  read() in the command loop does not see tail bytes from binary packets. */
static void drain_client_rx(int fd) {
    char buf[512];
    ssize_t n;
    while ((n = recv(fd, buf, sizeof(buf), MSG_DONTWAIT)) > 0) {
        /* discard */
    }
}

/** One line after STARTED: active sensors at stream start (indices 0..n-1). */
static void write_sensors_snapshot_line(int client_fd, HardwareContext *ctx) {
    char line[512];
    int pos = snprintf(line, sizeof(line), "SENSORS:");
    if (ctx->active_sensor_count <= 0) {
        snprintf(line + pos, sizeof(line) - (size_t) pos, "\n");
        write(client_fd, line, strlen(line));
        return;
    }
    for (int i = 0; i < ctx->active_sensor_count; i++) {
        char safe[20];
        strncpy(safe, ctx->sensors[i].name, sizeof(safe) - 1);
        safe[sizeof(safe) - 1] = '\0';
        for (char *p = safe; *p; p++) {
            if (*p == ',' || *p == ';' || *p == '\n' || *p == '\r')
                *p = '_';
        }
        pos += snprintf(line + pos, sizeof(line) - (size_t) pos, "%d,%s%s",
                        i, safe, (i + 1 < ctx->active_sensor_count) ? ";" : "");
        if (pos >= (int) sizeof(line) - 4)
            break;
    }
    snprintf(line + pos, sizeof(line) - (size_t) pos, "\n");
    write(client_fd, line, strlen(line));
}

static int run_stream(int client_fd, int udp_fd, struct sockaddr_in *client_udp_addr,
                      HardwareContext *ctx, FILE *bin_file, FILE *csv_file, int base_channels) {
    int hw_hz = current_stream_hw_hz();
    uint32_t ticks_per_sample = ticks_for_stream_hz(hw_hz);
    const int max_total_channels = base_channels + ctx->active_sensor_count * 4 + ANALOG_WAVEFORM_CHANNELS;
    const int max_bytes_per_frame = max_total_channels * 2;

    // --- Memory Allocations ---
    uint8_t *packet = (uint8_t *)malloc(HEADER_SIZE + max_bytes_per_frame);
    int16_t *frame_buffer = (int16_t *)malloc(max_bytes_per_frame);
    uint8_t *sd_write_buffer = (uint8_t *)malloc(max_bytes_per_frame * BUF_SAMPLES); // The massive RAM queue

    bool with_fusion = fusion_is_enabled();
    int current_channels = 0;
    int bytes_per_frame  = 0;
    init_frame_layout(ctx, base_channels, &current_channels, &bytes_per_frame);
    sensor_threads_init(ctx, ctx->active_sensor_count);
    int buffered_bytes_per_frame = bytes_per_frame;
    bool record = true;
    int buf_idx = 0;
    long long vqf_total_ns = 0;
    long long vqf_max_ns = 0;
    unsigned long long vqf_call_count = 0;
    double csv_elapsed = 0.0;

    // UDP chunk buffer: accumulates CHUNK_SAMPLES packets before a single sendto().
    const int chunk_pkt_size  = HEADER_SIZE + bytes_per_frame;
    const int chunk_buf_bytes = chunk_pkt_size * CHUNK_SAMPLES;
    uint8_t  *chunk_buffer    = (uint8_t *)malloc(chunk_buf_bytes);
    int       chunk_idx       = 0;

    if (packet == NULL || frame_buffer == NULL || sd_write_buffer == NULL || chunk_buffer == NULL) {
        recover_active_i2c_buses(ctx);
        sensor_threads_shutdown();
        free(packet);
        free(frame_buffer);
        free(sd_write_buffer);
        free(chunk_buffer);
        return -1;
    }

    // --- Header Setup (first sent frame uses sequence 0; ns increments before each send) ---
    int32_t ns = -1;

    *ctx->gpio_reset = 1; usleep(1); *ctx->gpio_reset = 0;
    uint32_t last_counter = *ctx->gpio_counter;
    init_sensor_decimation(ctx, hw_hz);

    while (1) {
        int current_hw_hz = current_stream_hw_hz();
        if (current_hw_hz != hw_hz) {
            hw_hz = current_hw_hz;
            ticks_per_sample = ticks_for_stream_hz(hw_hz);
            init_sensor_decimation(ctx, hw_hz);
            last_counter = *ctx->gpio_counter;
        }

        uint32_t now = *ctx->gpio_counter;
        if ((now - last_counter) < ticks_per_sample) {
            int command_state = process_stream_commands(client_fd, ctx, &bin_file, &csv_file, &record, &buf_idx, sd_write_buffer, buffered_bytes_per_frame);
            if (command_state != 0) {
                recover_active_i2c_buses(ctx);
                sensor_threads_shutdown();
                free(packet); free(frame_buffer); free(sd_write_buffer); free(chunk_buffer);
                return command_state > 0 ? 0 : -1;
            }
            usleep(100); continue;
        }
        last_counter += ticks_per_sample;

        {
            int command_state = process_stream_commands(client_fd, ctx, &bin_file, &csv_file, &record, &buf_idx, sd_write_buffer, buffered_bytes_per_frame);
            if (command_state != 0) {
                recover_active_i2c_buses(ctx);
                sensor_threads_shutdown();
                free(packet); free(frame_buffer); free(sd_write_buffer); free(chunk_buffer);
                return command_state > 0 ? 0 : -1;
            }
        }

        struct timespec t_start, t_end;
        clock_gettime(CLOCK_MONOTONIC, &t_start);
        {
            bool old_with_fusion = with_fusion;
            update_fusion_state(&with_fusion);

            if (with_fusion != old_with_fusion) {
                printf("Fusion %s during stream.\n", with_fusion ? "enabled" : "disabled");
            }
        }
        ns++;
        acquire_sensor_samples_decimated(ctx, with_fusion, bytes_per_frame, frame_buffer,
                               &vqf_total_ns, &vqf_max_ns, &vqf_call_count);
        clock_gettime(CLOCK_MONOTONIC, &t_end);
        warn_if_sample_loop_slow_us(&t_start, &t_end);
        maybe_report_vqf_stats(with_fusion, ns, &vqf_total_ns, &vqf_max_ns, &vqf_call_count);

        // --- File Logging (The Unblocked Double-Buffer) ---
        if (record && bin_file != NULL) {
            // Copy this single frame into our giant RAM block
            memcpy(sd_write_buffer + (buf_idx * buffered_bytes_per_frame), frame_buffer, bytes_per_frame);
            buf_idx++;

            // Only trigger the slow SD card write once the block is full
            if (buf_idx >= BUF_SAMPLES) {
                fwrite(sd_write_buffer, 1, buffered_bytes_per_frame * BUF_SAMPLES, bin_file);
                fflush(bin_file);
                buf_idx = 0;     
            }
        }

        if (record && csv_file != NULL) {
            write_csv_row(csv_file, ctx, with_fusion, ns, csv_elapsed, frame_buffer);
            csv_elapsed += 1.0 / (double)current_stream_hw_hz();
        }

        // --- Network Send over UDP (buffered into CHUNK_SAMPLES-packet datagrams) ---
        memcpy(packet + HEADER_SIZE, frame_buffer, bytes_per_frame);
        write_stream_header(packet, bytes_per_frame, ctx->total_channels, ns);

        memcpy(chunk_buffer + (chunk_idx * chunk_pkt_size), packet, chunk_pkt_size);
        chunk_idx++;
        if (chunk_idx >= CHUNK_SAMPLES) {
            sendto(udp_fd, chunk_buffer, chunk_pkt_size * CHUNK_SAMPLES, 0,
                   (struct sockaddr*)client_udp_addr, sizeof(*client_udp_addr));
            chunk_idx = 0;
        }
    }

    // Flush any packets remaining in the chunk buffer before exiting.
    if (chunk_idx > 0) {
        sendto(udp_fd, chunk_buffer, chunk_pkt_size * chunk_idx, 0,
               (struct sockaddr*)client_udp_addr, sizeof(*client_udp_addr));
    }

    // Standard exit cleanup
    if (record && bin_file != NULL && buf_idx > 0) {
        fwrite(sd_write_buffer, 1, buffered_bytes_per_frame * buf_idx, bin_file);
    }
    if (csv_file != NULL) fflush(csv_file);
    if (bin_file != NULL) fclose(bin_file);
    if (csv_file != NULL) fclose(csv_file);
    recover_active_i2c_buses(ctx);
    sensor_threads_shutdown();
    free(packet); free(frame_buffer); free(sd_write_buffer); free(chunk_buffer);
    return 0;
}


int main(void) {
    HardwareContext ctx = {0};
    printf(" Starting Server \n");
    /* SIGINT left at OS default so Ctrl+C terminates immediately. */
    signal(SIGPIPE, SIG_IGN); /* writing to a closed socket returns -1, not SIGPIPE */
    if (init_hardware(&ctx) < 0) {
        fprintf(stderr, "Error: Hardware initialization failed!\n");
        return 1;
    }
    printf("Hardware initialized. Active Sensors: %d, Total Channels: %d\n",ctx.active_sensor_count, ctx.total_channels);

    int base_channels = ctx.total_channels;
    bool start_with_fusion = false;

    reinit_fusion_for_hz(&ctx, (float)DESIRED_SAMPLE_RATE_HZ);

    calibrate_imu_biases(&ctx);
    init_analog_waveform_inputs();

    // Plugin controls fusion state - default to OFF
    start_with_fusion = false;
    fusion_set_enabled(start_with_fusion);
    ctx.total_channels = base_channels + ctx.active_sensor_count * 4 + ANALOG_WAVEFORM_CHANNELS;
    printf("Sensor fusion disabled (controlled by plugin). Total channels: %d\n", ctx.total_channels);

    // Always use TCP streaming - plugin controls all operations, no prompts
    printf("TCP streaming enabled (plugin controlled). No user prompts.\n");

    int server_fd, client_fd, opt = 1;
    struct sockaddr_in servaddr = {0};
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#ifdef SO_REUSEPORT
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt)) < 0) {
        perror("SO_REUSEPORT (ignored if unsupported)");
    }
#endif
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(PORT);
    if (bind(server_fd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("Bind failed"); // This will tell you if Port 5000 is busy
        return 1;
    }

    listen(server_fd, 1);
    printf("Server is listening on Port %d. Waiting for client...\n", PORT);

    // UDP socket for streaming data back to the client on port UDP_PORT.
    int udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd < 0) { perror("UDP socket creation failed"); return 1; }
    {
        struct sockaddr_in udp_addr = {0};
        udp_addr.sin_family      = AF_INET;
        udp_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        udp_addr.sin_port        = htons(UDP_PORT);
        if (bind(udp_fd, (struct sockaddr *)&udp_addr, sizeof(udp_addr)) < 0) {
            perror("UDP bind failed");
            return 1;
        }
    }
    printf("UDP data socket bound on port %d.\n", UDP_PORT);

    while (1) {
        client_fd = accept(server_fd, NULL, NULL);
        
        if (client_fd < 0) {
            perror("Accept failed");
            continue; // Go back to waiting if the connection drops
        }
        int flag = 1;
        if (setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int)) < 0) {
            perror("Could not set TCP_NODELAY");
        } else {
            printf("Client connected! TCP_NODELAY enabled. Stream unblocked.\n");
        }

        char buffer[64];
        while (1) {
            memset(buffer, 0, sizeof(buffer));
            int n = read(client_fd, buffer, sizeof(buffer) - 1);
            if (n <= 0) break;
            buffer[n] = '\0';

            /* FREQ and CFG are checked unconditionally so they are always
               applied even when combined with START in the same TCP read. */
            if (strstr(buffer, "FREQ:")) {
                int frequency_hz = atoi(strstr(buffer, "FREQ:") + 5);
                g_stream_hw_hz = clamp_stream_hw_hz(frequency_hz);
                for (int si = 0; si < ctx.active_sensor_count; si++) {
                    ctx.sensors[si].cfg_target_hz = g_stream_hw_hz;
                    apply_sensor_odr(&ctx.sensors[si], g_stream_hw_hz);
                }
                reinit_fusion_for_hz(&ctx, (float)g_stream_hw_hz);
                printf("Hardware stream tick rate set to %d Hz (idle).\n", g_stream_hw_hz);
            }
            if (strncmp(buffer, "CFG ", 4) == 0) {
                int si = -1, val = -1;
                if (sscanf(buffer, "CFG %d ACC %d", &si, &val) == 2) {
                    if (si >= 0 && si < ctx.active_sensor_count && val >= 0 && val <= 3) {
                        ctx.sensors[si].cfg_acc_id = val;
                        apply_sensor_cfg_acc(&ctx.sensors[si], val);
                        printf("CFG sensor %d ACC preset %d applied\n", si, val);
                    }
                } else if (sscanf(buffer, "CFG %d GYR %d", &si, &val) == 2) {
                    if (si >= 0 && si < ctx.active_sensor_count && val >= 0 && val <= 3) {
                        ctx.sensors[si].cfg_gyr_id = val;
                        apply_sensor_cfg_gyr(&ctx.sensors[si], val);
                        printf("CFG sensor %d GYR preset %d applied\n", si, val);
                    }
                } else if (sscanf(buffer, "CFG %d SRATE %d", &si, &val) == 2) {
                    if (si >= 0 && si < ctx.active_sensor_count && val >= 1) {
                        ctx.sensors[si].cfg_target_hz = val;
                        if (ctx.sensors[si].cfg_target_hz > g_stream_hw_hz)
                            ctx.sensors[si].cfg_target_hz = g_stream_hw_hz;
                        apply_sensor_odr(&ctx.sensors[si], ctx.sensors[si].cfg_target_hz);
                        FusionSensorConfig cfg;
                        fusion_config_for_sensor(&ctx.sensors[si], current_stream_hw_hz(), &cfg);
                        fusion_register_sensor_ex(si, &cfg);
                        printf("CFG sensor %d SRATE %d Hz (ODR applied)\n", si, val);
                    }
                }
            }

            if (strstr(buffer, "FUSION ON") && !fusion_is_enabled()) {
                fusion_set_enabled(true);
            }
            else if (strstr(buffer, "FUSION OFF") && fusion_is_enabled()) {
                fusion_set_enabled(false);
            }
            else if (strstr(buffer, "REDPITAYA")) {
                char msg[64];
                sprintf(msg, "OK CHANNELS:%d\n", ctx.total_channels);
                write(client_fd, msg, strlen(msg));
            }
            else if (strstr(buffer, "FILTER ON")) {
                fusion_set_enabled(true);
                printf("Filter enabled.\n");
            }
            else if (strstr(buffer, "FILTER OFF")) {
                fusion_set_enabled(false);
                printf("Filter disabled.\n");
            }
            else if (strstr(buffer, "AIN_GAIN:")) {
                float gain = strtof(strstr(buffer, "AIN_GAIN:") + 9, NULL);
                printf("Analog input gain set to %.2f.\n", gain);
            }
            else if (strstr(buffer, "AOUT:")) {
                float voltage = strtof(strstr(buffer, "AOUT:") + 5, NULL);
                printf("Analog output voltage set to %.2f V.\n", voltage);
            }
            else if (strstr(buffer, "START")) {
                system("rw");
                mkdir("/root/Measurements", 0775);

                time_t rawtime; struct tm *timeinfo;
                char time_str[20]; char bin_filename[128]; char csv_filename[128];
                time(&rawtime);
                timeinfo = localtime(&rawtime);
                strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", timeinfo);
                sprintf(bin_filename, "/root/Measurements/recording_%s.bin", time_str);
                sprintf(csv_filename, "/root/Measurements/recording_%s.csv", time_str);

                FILE *bin_fp = fopen(bin_filename, "wb");
                if (bin_fp == NULL) {
                    perror("Failed to open binary file");
                    write(client_fd, "ERROR_FILE\n", 11);
                    continue;
                }

                FILE *csv_fp = fopen(csv_filename, "w");
                if (csv_fp == NULL) {
                    perror("Failed to open CSV file");
                    fclose(bin_fp);
                    write(client_fd, "ERROR_FILE\n", 11);
                    continue;
                }

                write_csv_header(csv_fp, &ctx, true);

                for (int si = 0; si < ctx.active_sensor_count; si++) {
                    if (ctx.sensors[si].cfg_target_hz > g_stream_hw_hz)
                        ctx.sensors[si].cfg_target_hz = g_stream_hw_hz;
                }

                char started_msg[320];
                snprintf(started_msg, sizeof(started_msg), "STARTED BIN:%s CSV:%s\n", bin_filename, csv_filename);
                write(client_fd, started_msg, strlen(started_msg));
                write_sensors_snapshot_line(client_fd, &ctx);

                // Derive client UDP address from the established TCP connection.
                struct sockaddr_in client_udp_addr = {0};
                {
                    struct sockaddr_in tcp_peer = {0};
                    socklen_t peer_len = sizeof(tcp_peer);
                    getpeername(client_fd, (struct sockaddr *)&tcp_peer, &peer_len);
                    client_udp_addr.sin_family = AF_INET;
                    client_udp_addr.sin_addr   = tcp_peer.sin_addr;
                    client_udp_addr.sin_port   = htons(UDP_PORT);
                }

                if (run_stream(client_fd, udp_fd, &client_udp_addr, &ctx, bin_fp, csv_fp, base_channels) < 0) {
                    break;
                }
                drain_client_rx(client_fd);
                system("sync");
                write(client_fd, "STOPPED\n", 8);
            }
        }
        close(client_fd);
    }
    fusion_shutdown();
    if (analog_inputs_ready) {
        rp_Release();
    }
    return 0;
}

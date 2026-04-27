#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "rp.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <stdbool.h>
#include <setjmp.h>
#include <signal.h>

#include <netinet/tcp.h>

#include "axi_header.h"
#include "sensor_fusion.h"
#include "vqf.h"

// --- Hardware Constants ---
#define AXI_GPIO_ADDRESS  0x41200000
#define RANGE             64000
#define IIC_RANGE         64000
#define PORT              5000
#define DESIRED_SAMPLE_RATE_HZ 100
#define CTR_CLK_RATE      125000000
#define HEADER_SIZE       22
#define GYRO_BIAS_CALIBRATION_SAMPLES 200
#define ICM20948_BANK_0 0x00
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
// --- Discovery Logic Structures ---
typedef struct {
    char name[16];
    void *axi_map;
    uint8_t i2c_addr;
    int num_channels;
    uint8_t data_reg_start;
    bool active;
    bool split_read;
    bool is_spi; // NEW: Flag to tell run_stream which protocol to use
    int16_t gyro_bias[3];
    bool mag_is_fresh;
} SensorInstance;

typedef struct {
    void *axi_gpio_map;
    volatile uint32_t *gpio_counter;
    volatile uint32_t *gpio_reset;

    SensorInstance sensors[6]; // UPDATED: 4 I2C slots + 2 SPI slots
    int active_sensor_count;
    int total_channels;
} HardwareContext;

// Global jump buffer for the watchdog
static sigjmp_buf watchdog_bucket;
static volatile sig_atomic_t stop_requested = 0;

static long long timespec_diff_ns(const struct timespec *start, const struct timespec *end) {
    return ((long long)(end->tv_sec - start->tv_sec) * 1000000000LL) +
           (long long)(end->tv_nsec - start->tv_nsec);
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

static void write_stream_header(uint8_t *packet, int bytes_per_frame, int total_channels, int32_t ns)
{
    int32_t offset = 0;
    int32_t bpb = bytes_per_frame;
    int32_t elm = 2;
    int16_t dtype = 3;

    memcpy(packet + 0,  &offset, 4);
    memcpy(packet + 4,  &bpb, 4);
    memcpy(packet + 8,  &dtype, 2);
    memcpy(packet + 10, &elm, 4);
    memcpy(packet + 14, &total_channels, 4);
    memcpy(packet + 18, &ns, 4);
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
            uint8_t mag_raw[7];
            axi_iic_read_n_bytes(s->axi_map, s->i2c_addr, 0x3B, raw, 12);
            axi_iic_read_n_bytes(s->axi_map, 0x0C, 0x03, mag_raw, 7);

            for (int j = 0; j < 6; j++) {
                channel_out[j] = (int16_t)((raw[j * 2] << 8) | raw[j * 2 + 1]);
            }

            for (int j = 0; j < 3; j++) {
                int16_t val = (int16_t)(mag_raw[j * 2] | (mag_raw[j * 2 + 1] << 8));
                int32_t amplified = (int32_t)val * 16;
                if (amplified > 32767) amplified = 32767;
                if (amplified < -32768) amplified = -32768;
                channel_out[j + 6] = (int16_t)amplified;
            }
            s->mag_is_fresh = true;
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
            s->mag_is_fresh = true;
        } else {
            channel_out[6] = 0;
            channel_out[7] = 0;
            channel_out[8] = 0;
        }
    } else if (strcmp(s->name, "BNO055") == 0) {
        uint8_t raw[18];
        axi_iic_read_n_bytes(s->axi_map, s->i2c_addr, s->data_reg_start, raw, 18);
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
        uint8_t raw[12];
        axi_iic_read_n_bytes(s->axi_map, s->i2c_addr, 0x3B, raw, 6);
        axi_iic_read_n_bytes(s->axi_map, s->i2c_addr, 0x43, raw + 6, 6);
        for (int j = 0; j < s->num_channels; j++) {
            channel_out[j] = (int16_t)((raw[j * 2] << 8) | raw[j * 2 + 1]);
        }
    } else {
        uint8_t raw[32];
        axi_iic_read_n_bytes(s->axi_map, s->i2c_addr, s->data_reg_start, raw, s->num_channels * 2);
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

static void calibrate_gyro_biases(HardwareContext *ctx) {
    int16_t raw_channels[16];
    int64_t gyro_sums[6][3] = {0};

    if (ctx->active_sensor_count <= 0) return;

    printf("Calibrating gyro bias using %d stationary samples. Keep sensors still...\n",
           GYRO_BIAS_CALIBRATION_SAMPLES);
    fflush(stdout);

    for (int sample = 0; sample < GYRO_BIAS_CALIBRATION_SAMPLES; sample++) {
        for (int i = 0; i < ctx->active_sensor_count; i++) {
            SensorInstance *s = &ctx->sensors[i];
            if (s->num_channels < 6) continue;

            memset(raw_channels, 0, sizeof(raw_channels));
            read_sensor_raw_channels(s, raw_channels);

            int16_t raw_gyr[3];
            get_sensor_gyro_from_channels(s, raw_channels, raw_gyr);
            gyro_sums[i][0] += raw_gyr[0];
            gyro_sums[i][1] += raw_gyr[1];
            gyro_sums[i][2] += raw_gyr[2];
        }
        usleep(10000);
    }

    for (int i = 0; i < ctx->active_sensor_count; i++) {
        SensorInstance *s = &ctx->sensors[i];
        if (s->num_channels < 6) continue;

        s->gyro_bias[0] = (int16_t)(gyro_sums[i][0] / GYRO_BIAS_CALIBRATION_SAMPLES);
        s->gyro_bias[1] = (int16_t)(gyro_sums[i][1] / GYRO_BIAS_CALIBRATION_SAMPLES);
        s->gyro_bias[2] = (int16_t)(gyro_sums[i][2] / GYRO_BIAS_CALIBRATION_SAMPLES);

        printf("Sensor %d (%s) gyro bias: [%d, %d, %d]\n",
               i, s->name, s->gyro_bias[0], s->gyro_bias[1], s->gyro_bias[2]);
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
    fprintf(fp, "sample_index,elapsed_seconds");
    for (int i = 0; i < ctx->active_sensor_count; i++) {
        write_sensor_csv_labels(fp, &ctx->sensors[i], i, with_fusion);
    }
    fprintf(fp, "\n");
}

static void write_csv_row(FILE *fp, HardwareContext *ctx, bool with_fusion, int sample_index, const int16_t *frame_buffer) {
    int channel_offset = 0;
    (void)with_fusion;

    fprintf(fp, "%d,%.6f", sample_index, (double)sample_index / (double)DESIRED_SAMPLE_RATE_HZ);

    for (int i = 0; i < ctx->active_sensor_count; i++) {
        const SensorInstance *s = &ctx->sensors[i];
        int sensor_channels = s->num_channels + 4;

        for (int j = 0; j < sensor_channels; j++) {
            fprintf(fp, ",%d", frame_buffer[channel_offset + j]);
        }

        channel_offset += sensor_channels;
    }

    fprintf(fp, "\n");
}

static int update_frame_layout(HardwareContext *ctx, int base_channels, bool *with_fusion, int *current_channels, int *bytes_per_frame) {
    bool next_with_fusion = fusion_is_enabled();
    int next_channels = base_channels + ctx->active_sensor_count * 4;
    int next_bytes_per_frame = next_channels * 2;

    *with_fusion = next_with_fusion;
    *current_channels = next_channels;
    *bytes_per_frame = next_bytes_per_frame;
    ctx->total_channels = next_channels;
    return 0;
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
            int16_t raw_acc[3] = { channel_out[0], channel_out[1], channel_out[2] };
            int16_t raw_gyr[3];
            const int16_t *raw_mag = NULL;
            bool mag_is_fresh = false;

            if (strcmp(s->name, "BNO055") == 0) {
                get_sensor_gyro_from_channels(s, channel_out, raw_gyr);
                raw_mag = &channel_out[3];
                mag_is_fresh = true;
            } else {
                get_sensor_gyro_from_channels(s, channel_out, raw_gyr);
                if (strcmp(s->name, "MPU9250") == 0 && !s->is_spi) {
                    raw_mag = &channel_out[6];
                    mag_is_fresh = true;
                } else if (strcmp(s->name, "ICM20948") == 0 && s->is_spi) {
                    raw_mag = &channel_out[6];
                    mag_is_fresh = s->mag_is_fresh;
                }
            }

            raw_gyr[0] -= s->gyro_bias[0];
            raw_gyr[1] -= s->gyro_bias[1];
            raw_gyr[2] -= s->gyro_bias[2];

            struct timespec vqf_start;
            struct timespec vqf_end;
            clock_gettime(CLOCK_MONOTONIC, &vqf_start);
            fusion_update_sensor(i, raw_acc, raw_gyr, raw_mag, mag_is_fresh, channel_out + s->num_channels);
            clock_gettime(CLOCK_MONOTONIC, &vqf_end);

            long long vqf_elapsed_ns = timespec_diff_ns(&vqf_start, &vqf_end);
            *vqf_total_ns += vqf_elapsed_ns;
            (*vqf_call_count)++;
            if (vqf_elapsed_ns > *vqf_max_ns) {
                *vqf_max_ns = vqf_elapsed_ns;
            }
        }

        current_byte_offset += (s->num_channels + 4) * 2;
    }
}

static void maybe_report_vqf_stats(
    bool with_fusion,
    int sample_number,
    long long *vqf_total_ns,
    long long *vqf_max_ns,
    unsigned long long *vqf_call_count
) {
    if (with_fusion && *vqf_call_count > 0 && (sample_number % DESIRED_SAMPLE_RATE_HZ) == 0) {
        double avg_us = (double)(*vqf_total_ns) / (double)(*vqf_call_count) / 1000.0;
        double max_us = (double)(*vqf_max_ns) / 1000.0;
        printf("VQF filter time over last %llu calls: avg %.2f us, max %.2f us\n",
               *vqf_call_count, avg_us, max_us);
        *vqf_total_ns = 0;
        *vqf_max_ns = 0;
        *vqf_call_count = 0;
    }
}

static int run_timed_csv_capture(HardwareContext *ctx, int base_channels, int duration_seconds, const char *csv_path) {
    uint32_t ticks_per_sample = CTR_CLK_RATE / DESIRED_SAMPLE_RATE_HZ;
    const int max_total_channels = base_channels + ctx->active_sensor_count * 4;
    const int max_bytes_per_frame = max_total_channels * 2;
    const int total_samples = duration_seconds * DESIRED_SAMPLE_RATE_HZ;
    int16_t *frame_buffer = (int16_t *)malloc(max_bytes_per_frame);
    bool with_fusion = fusion_is_enabled();
    int current_channels = base_channels + ctx->active_sensor_count * 4;
    int bytes_per_frame = current_channels * 2;
    long long vqf_total_ns = 0;
    long long vqf_max_ns = 0;
    unsigned long long vqf_call_count = 0;
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

        update_frame_layout(ctx, base_channels, &with_fusion, &current_channels, &bytes_per_frame);
        acquire_sensor_samples(ctx, with_fusion, bytes_per_frame, frame_buffer,
                               &vqf_total_ns, &vqf_max_ns, &vqf_call_count);
        write_csv_row(csv_file, ctx, with_fusion, sample_index, frame_buffer);
        maybe_report_vqf_stats(with_fusion, sample_index + 1, &vqf_total_ns, &vqf_max_ns, &vqf_call_count);

        if (((sample_index + 1) % DESIRED_SAMPLE_RATE_HZ) == 0) {
            int elapsed_seconds = (sample_index + 1) / DESIRED_SAMPLE_RATE_HZ;
            int remaining_seconds = duration_seconds - elapsed_seconds;
            if (remaining_seconds < 0) remaining_seconds = 0;
            printf("Capture progress: %d/%d seconds complete.\n", elapsed_seconds, duration_seconds);
        }
    }

    fflush(csv_file);
    fclose(csv_file);
    free(frame_buffer);
    return 0;
}

// --- Helper: Sensor Identification ---
static void identify_and_add_sensor(HardwareContext *ctx, void *map, uint8_t id, uint8_t addr, bool is_spi) {
    SensorInstance *s = &ctx->sensors[ctx->active_sensor_count];
    s->axi_map = map;
    s->i2c_addr = addr;
    s->is_spi = is_spi;
    s->active = true;

    if (id == 0x68 && !is_spi) { // MPU6050
        strcpy(s->name, "MPU6050");
        s->split_read = true;
        s->num_channels = 6;
        s->data_reg_start = 0x3B;
        axi_iic_write_byte(map, addr, 0x6B, 0x80);
        usleep(10000); // 10ms Reset delay
        axi_iic_write_byte(map, addr, 0x6B, 0x01);
        usleep(5000);  // 5ms Wake delay
    }
    else if (id == 0x71) { // MPU9250
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
        
        // Disable I2C Master and Enable Bypass to see the Magnetometer (0x0C)
        axi_iic_write_byte(map, addr, 0x6A, 0x00); 
        axi_iic_write_byte(map, addr, 0x37, 0x02); 
        usleep(5000);

        // This call is safe here because we are actually on an I2C bus
        axi_iic_write_byte(map, 0x0C, 0x0A, 0x16); 
        printf("  -> MPU9250 (I2C) initialized (9-axis enabled).\n");
    }
}
    else if (id == 0xEA) { // ICM20948
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
    else if (id == 0xA0 && !is_spi) { // BNO055
        strcpy(s->name, "BNO055");
        s->split_read = false;
        s->num_channels = 9;
        s->data_reg_start = 0x08;
        axi_iic_write_byte(map, addr, 0x3D, 0x0C); // NDOF Mode
        usleep(20000);
    }

    ctx->total_channels += s->num_channels;
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
static int run_stream(int client_fd, HardwareContext *ctx, FILE *log_file, int base_channels) {
    uint32_t ticks_per_sample = CTR_CLK_RATE / DESIRED_SAMPLE_RATE_HZ;
    const int max_total_channels = base_channels + ctx->active_sensor_count * 4;
    const int max_bytes_per_frame = max_total_channels * 2;

    // --- Memory Allocations ---
    uint8_t *packet = (uint8_t *)malloc(HEADER_SIZE + max_bytes_per_frame);
    int16_t *frame_buffer = (int16_t *)malloc(max_bytes_per_frame);
    uint8_t *sd_write_buffer = (uint8_t *)malloc(max_bytes_per_frame * BUF_SAMPLES); // The massive RAM queue

    bool with_fusion = fusion_is_enabled();
    int current_channels = base_channels + ctx->active_sensor_count * 4;
    int bytes_per_frame = current_channels * 2;
    int buffered_bytes_per_frame = bytes_per_frame;
    char cmd[32];
    bool record = false;
    int buf_idx = 0;
    long long vqf_total_ns = 0;
    long long vqf_max_ns = 0;
    unsigned long long vqf_call_count = 0;

    if (packet == NULL || frame_buffer == NULL || sd_write_buffer == NULL) {
        free(packet);
        free(frame_buffer);
        free(sd_write_buffer);
        return -1;
    }

    // --- Header Setup ---
    int32_t ns = 1;
    ctx->total_channels = current_channels;
    write_stream_header(packet, bytes_per_frame, ctx->total_channels, ns);

    *ctx->gpio_reset = 1; usleep(1); *ctx->gpio_reset = 0;
    uint32_t last_counter = *ctx->gpio_counter;

    while (1) {
        uint32_t now = *ctx->gpio_counter;
        if ((now - last_counter) < ticks_per_sample) {
            int n = recv(client_fd, cmd, sizeof(cmd) - 1, MSG_DONTWAIT);
            if (n > 0) {
                cmd[n] = '\0';

                if (strstr(cmd, "STOP")) { 
                    // Flush any leftover data in RAM before stopping
                    if (record && log_file != NULL && buf_idx > 0) {
                        fwrite(sd_write_buffer, 1, buffered_bytes_per_frame * buf_idx, log_file);
                        fflush(log_file);
                    }
                    free(packet); free(frame_buffer); free(sd_write_buffer); 
                    return 0; 
                }

                if (strstr(cmd, "RECORD ON")) record = true;
                if (strstr(cmd, "FILTER ON")) {
                    fusion_set_enabled(true);
                    printf("Filter enabled.\n");
                }
                if (strstr(cmd, "FILTER OFF")) {
                    fusion_set_enabled(false);
                    printf("Filter disabled.\n");
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
                    int frequency_hz = atoi(strstr(cmd, "FREQ:") + 5);
                    printf("Requested sample frequency: %d Hz.\n", frequency_hz);
                }
                if (strstr(cmd, "FUSION ON")) fusion_set_enabled(true);
                if (strstr(cmd, "FUSION OFF")) fusion_set_enabled(false);
                if (strstr(cmd, "RECORD OFF")) {
                    record = false;
                    // Flush buffer when recording is toggled off manually
                    if (log_file != NULL && buf_idx > 0) {
                        fwrite(sd_write_buffer, 1, buffered_bytes_per_frame * buf_idx, log_file);
                        fflush(log_file);
                        buf_idx = 0;
                    }
                }
            } else if (n == 0) { 
                // Client disconnected suddenly. Flush and exit.
                if (record && log_file != NULL && buf_idx > 0) {
                    fwrite(sd_write_buffer, 1, buffered_bytes_per_frame * buf_idx, log_file);
                }
                free(packet); free(frame_buffer); free(sd_write_buffer); 
                return -1; 
            }
            usleep(100); continue;
        }
        last_counter += ticks_per_sample;

        {
            bool old_with_fusion = with_fusion;
            update_frame_layout(ctx, base_channels, &with_fusion, &current_channels, &bytes_per_frame);

            if (with_fusion != old_with_fusion) {
                printf("Fusion %s during stream.\n", with_fusion ? "enabled" : "disabled");
            }
        }
        acquire_sensor_samples(ctx, with_fusion, bytes_per_frame, frame_buffer,
                               &vqf_total_ns, &vqf_max_ns, &vqf_call_count);
        maybe_report_vqf_stats(with_fusion, ns, &vqf_total_ns, &vqf_max_ns, &vqf_call_count);

        // --- File Logging (The Unblocked Double-Buffer) ---
        if (record && log_file != NULL) {
            // Copy this single frame into our giant RAM block
            memcpy(sd_write_buffer + (buf_idx * buffered_bytes_per_frame), frame_buffer, bytes_per_frame);
            buf_idx++;

            // Only trigger the slow SD card write once the block is full
            if (buf_idx >= BUF_SAMPLES) {
                fwrite(sd_write_buffer, 1, buffered_bytes_per_frame * BUF_SAMPLES, log_file);
                fflush(log_file);
                buf_idx = 0;     
            }
        }

        // --- Network Send (Still fires instantly to keep GUI real-time) ---
        memcpy(packet + HEADER_SIZE, frame_buffer, bytes_per_frame);
        
        // Increment the ns counter so Open Ephys knows time is passing
        ns++; 
        write_stream_header(packet, bytes_per_frame, ctx->total_channels, ns);

        if (send(client_fd, packet, HEADER_SIZE + bytes_per_frame, 0) <= 0) break;
    }

    // Standard exit cleanup
    if (record && log_file != NULL && buf_idx > 0) {
        fwrite(sd_write_buffer, 1, buffered_bytes_per_frame * buf_idx, log_file);
    }
    free(packet); free(frame_buffer); free(sd_write_buffer);
    return 0;
}


int main(void) {
    HardwareContext ctx = {0};
    printf(" Starting Server \n");
    signal(SIGINT, stop_handler);
    if (init_hardware(&ctx) < 0) {
        fprintf(stderr, "Error: Hardware initialization failed!\n");
        return 1;
    }
    printf("Hardware initialized. Active Sensors: %d, Total Channels: %d\n",ctx.active_sensor_count, ctx.total_channels);

    int base_channels = ctx.total_channels;
    bool start_with_fusion = false;

    fusion_init(ctx.active_sensor_count, (float)DESIRED_SAMPLE_RATE_HZ);
    for (int i = 0; i < ctx.active_sensor_count; i++) {
        SensorInstance *s = &ctx.sensors[i];
        FusionSensorType ftype;
        if      (strcmp(s->name, "MPU6050")  == 0) ftype = FUSION_SENSOR_TYPE_MPU6050;
        else if (strcmp(s->name, "MPU9250")  == 0) ftype = FUSION_SENSOR_TYPE_MPU9250;
        else if (strcmp(s->name, "ICM20948") == 0) ftype = FUSION_SENSOR_TYPE_ICM20948;
        else if (strcmp(s->name, "BNO055")   == 0) ftype = FUSION_SENSOR_TYPE_BNO055;
        else                                        ftype = FUSION_SENSOR_TYPE_GENERIC;
        FusionSensorConfig cfg;
        {
            bool has_mag = false;

            if (strcmp(s->name, "MPU9250") == 0 && !s->is_spi) {
                has_mag = true;
            } else if (strcmp(s->name, "ICM20948") == 0) {
                has_mag = true;
            } else if (strcmp(s->name, "BNO055") == 0) {
                has_mag = true;
            }

            fusion_get_default_sensor_config(ftype, has_mag, &cfg);
        }
        fusion_register_sensor_ex(i, &cfg);
    }

    calibrate_gyro_biases(&ctx);

    // Plugin controls fusion state - default to OFF
    start_with_fusion = false;
    fusion_set_enabled(start_with_fusion);
    ctx.total_channels = base_channels + ctx.active_sensor_count * 4;
    printf("Sensor fusion disabled (controlled by plugin). Total channels: %d\n", ctx.total_channels);

    // Always use TCP streaming - plugin controls all operations, no prompts
    printf("TCP streaming enabled (plugin controlled). No user prompts.\n");

    int server_fd, client_fd, opt = 1;
    struct sockaddr_in servaddr = {0};
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(PORT);
    if (bind(server_fd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("Bind failed"); // This will tell you if Port 5000 is busy
        return 1;
    }

    listen(server_fd, 1);
    printf("Server is listening on Port %d. Waiting for client...\n", PORT);

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
            else if (strstr(buffer, "FREQ:")) {
                int frequency_hz = atoi(strstr(buffer, "FREQ:") + 5);
                printf("Requested sample frequency: %d Hz.\n", frequency_hz);
            }
            else if (strstr(buffer, "START")) {
                system("rw");
                time_t rawtime; struct tm *timeinfo;
                char time_str[20]; char filename[128];
                time(&rawtime);
                timeinfo = localtime(&rawtime);
                strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", timeinfo);
                sprintf(filename, "/root/Measurements/recording_%s.bin", time_str);

                FILE *fp = fopen(filename, "wb");
                if (fp == NULL) {
                    perror("Failed to open file");
                    write(client_fd, "ERROR_FILE\n", 11);
                    continue;
                }
                char started_msg[192];
                snprintf(started_msg, sizeof(started_msg), "STARTED %s\n", filename);
                write(client_fd, started_msg, strlen(started_msg));
                if (run_stream(client_fd, &ctx, fp, base_channels) < 0) { fclose(fp); break; }
                fclose(fp);
                system("sync");
                write(client_fd, "STOPPED\n", 8);
            }
        }
        close(client_fd);
    }
    fusion_shutdown();
    return 0;
}

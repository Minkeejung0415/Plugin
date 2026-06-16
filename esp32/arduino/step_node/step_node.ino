/*
 * STEP ESP32-S3 node — Arduino IDE entry point
 * Seeed XIAO ESP32S3: ICM20948 + DIO + SD + Open Ephys TCP (Red Pitaya parity)
 *
 * Guide: docs/arduino-ide-guide.md
 *
 * --- Wi-Fi connect timeout fallback ---
 * If STA join fails (45 s), firmware starts Soft AP: SSID STEP_ESP32, pass step1234.
 * On your PC: join Wi-Fi "STEP_ESP32" (password step1234), then Open Ephys / TCP host 192.168.4.1:5000.
 *
 * --- Phone hotspot: use 2.4 GHz band only (ESP32-S3 does not join 5 GHz-only APs). ---
 * Edit WIFI_SSID / WIFI_PASS below (was ubcvisitor open campus — change for your hotspot).
 *
 * --- WIRING_4WIRE_ICM + USB to PC (copy-paste preset) ---
 * #define ENABLE_TCP false
 * #define ENABLE_SERIAL_BENCH true
 * #define ENABLE_ESPNOW false
 * #define ENABLE_SD false
 * #define PIN_ICM_CS 44
 * --- end preset ---
 *
 * --- USB_OPEN_EPHYS_MODE (USB power + PC — Wi-Fi not required for Open Ephys) ---
 * Plugin Acq Board: host\run_usb_plugin_bridge.ps1 COM5  (or serial_tcp_bridge.py COM5 --plugin)
 *   → Open Ephys Node IP 127.0.0.1:5000 — NOT the ESP32 Wi-Fi IP; bridge speaks REDPITAYA/START.
 * Ephys Socket (no Plugin build): serial_tcp_bridge.py COM5 without --plugin.
 * Set USB_OPEN_EPHYS_MODE true below (or copy these defines):
 * #define ENABLE_TCP false
 * #define ENABLE_SERIAL_BENCH true
 * #define SERIAL_OUTPUT_BINARY true
 * #define ENABLE_ESPNOW false
 * #define ENABLE_SD false
 * --- end USB_OPEN_EPHYS_MODE ---
 */

#define ENABLE_ESPNOW true
#define ESPNOW_WIFI_CHANNEL 6   // Must match WIFI_AP_CHANNEL so slaves on STEP_ESP32 AP receive sync

#include <WiFi.h>
#include <WiFiClient.h>
#if ENABLE_ESPNOW
#include <esp_now.h>
#include "esp_wifi.h"
#endif
#include <esp_timer.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>

extern "C" {
#include "vqf.h"
}

#define FIRMWARE_VERSION "1.7.0"
#define WIFI_HOSTNAME "step-esp32"
#define BOOT_CSV_DELAY_MS 5000
#define REPEAT_STATUS_SEC 10
#define BOOT_DIAGNOSTICS true

#define DIO_DEBOUNCE_MS 15   // stable toggle within ~20 ms @ 100 Hz

// STA: join your phone/lab hotspot (2.4 GHz). Empty WIFI_PASS = open network (WiFi.begin SSID only).
#define WIFI_SSID "YOUR_HOTSPOT"
#define WIFI_PASS "yourpassword"

// Soft AP fallback after STA timeout (automatic — do not need to edit unless renaming lab AP)
#define WIFI_AP_SSID "STEP_ESP32"
#define WIFI_AP_PASS "step1234"
#define WIFI_AP_CHANNEL 6       // 2.4 GHz only — use 1, 6, or 11; explicit helps Windows join
#define WIFI_AP_MAX_CONN 4
#define WIFI_STA_TIMEOUT_MS 1
// XIAO boards: high TX can desense the onboard antenna — try lower if STA/AP both fail
#define WIFI_TX_POWER_STA WIFI_POWER_8_5dBm
#define WIFI_TX_POWER_AP WIFI_POWER_8_5dBm

#define TCP_PORT 5000
#define SAMPLE_HZ_DEFAULT 100
#define NUM_CHANNELS 14

#define ICM_BANK2_GYRO_SMPLRT_DIV 0x00
#define ICM_BANK2_GYRO_CONFIG_1 0x01
#define ICM_BANK2_ODR_ALIGN_EN 0x09
#define ICM_BANK2_ACCEL_SMPLRT_DIV_1 0x10
#define ICM_BANK2_ACCEL_SMPLRT_DIV_2 0x11
#define ICM_BANK2_ACCEL_CONFIG_1 0x14

#define PIN_SPI_SCK 7    // XIAO D8 / GPIO7
#define PIN_SPI_MISO 8   // XIAO D9 / GPIO8
#define PIN_SPI_MOSI 9   // XIAO D10 / GPIO9
#define PIN_ICM_CS 44    // XIAO D7 / GPIO44
#define PIN_DIO 1       // XIAO D0 / GPIO1 — change via #define if wired elsewhere

#define NODE_IS_MASTER true
#define ENABLE_SD true

// true = USB binary @100 Hz 8ch → serial_tcp_bridge.py [--plugin] → 127.0.0.1:5000 (no Wi-Fi for OE).
// false = Wi-Fi TCP :5000 on board; Plugin uses Serial Monitor IP, not 127.0.0.1.
#define USB_OPEN_EPHYS_MODE true

#if USB_OPEN_EPHYS_MODE
#define ENABLE_TCP false
#define ENABLE_SERIAL_BENCH true
#define SERIAL_OUTPUT_BINARY true
#else
#define ENABLE_TCP true
#define ENABLE_SERIAL_BENCH false
#define SERIAL_OUTPUT_BINARY false
#endif

#define PIN_SD_CS 21

#define ICM_REG_BANK_SEL 0x7F
#define ICM_WHO_AM_I 0x00
#define ICM_PWR_MGMT_1 0x06
#define ICM_USER_CTRL 0x03
#define ICM_INT_PIN_CFG 0x0F
#define ICM_ACCEL_XOUT_H 0x2D
#define ICM20948_WHOAMI_VAL 0xEA
#define ICM_EXT_SENS_DATA_00 0x3B
#define ICM_I2C_MST_CTRL 0x01
#define ICM_I2C_SLV0_ADDR 0x03
#define ICM_I2C_SLV0_REG 0x04
#define ICM_I2C_SLV0_CTRL 0x05
#define ICM_I2C_SLV0_DO 0x06

#define AK09916_ADDR 0x0C
#define AK09916_WIA2 0x01
#define AK09916_ST1 0x10
#define AK09916_HXL 0x11
#define AK09916_ST2 0x18
#define AK09916_CNTL2 0x31
#define AK09916_CNTL3 0x32
#define AK09916_WIA2_VAL 0x09
#define AK09916_MODE_CONT_100HZ 0x08

#pragma pack(push, 1)
struct OeHeader {
  int32_t offset;
  int32_t num_bytes;
  uint16_t bit_depth;
  int32_t element_size;
  int32_t num_channels;
  int32_t samples_per_channel;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct SdLogHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t record_size;
  uint16_t sample_hz;
  uint16_t channel_count;
  int64_t start_time_us;
};

struct SdLogRecord {
  uint32_t seq;
  int64_t time_us;
  int16_t ch[NUM_CHANNELS];
};
#pragma pack(pop)

#define SD_LOG_MAGIC 0x31505453UL  // "STP1" little-endian
#define SD_LOG_VERSION 1
#define REC_RECONNECT_GRACE_MS 90000UL
#define SD_QUEUE_DEPTH 1024
#define REC_MAX_CHUNK 1024UL
#define SDRF_HEADER_LEN 64
#define SDRF_TYPE_DATA 0x01
#define SDRF_TYPE_EOF 0x02

#pragma pack(push, 1)
struct SdrfHeader {
  char magic[4];
  uint8_t frame_version;
  uint8_t frame_type;
  uint16_t header_length;
  uint8_t session_id[16];
  uint32_t chunk_index;
  uint64_t byte_offset;
  uint32_t payload_length;
  uint64_t total_file_size;
  uint32_t header_crc32;
  uint32_t payload_checksum;
  uint32_t flags;
  uint32_t reserved;
};
#pragma pack(pop)

WiFiServer server(TCP_PORT);
WiFiClient client;
bool streaming = false;
bool wifi_up = false;
bool wifi_soft_ap = false;

uint32_t seq = 0;
int16_t channels[NUM_CHANNELS];
bool icm_ok = false;
bool mag_ok = false;
uint32_t boot_ms = 0;
bool csv_banner_sent = false;
uint32_t last_status_ms = 0;

// DIO input is kept for commands/sync, but it is not part of the 13-channel OE stream.
struct {
  bool stable_high;
  bool pending_raw;
  uint32_t pending_since_ms;
  uint16_t edge_count;
} dio_state = {true, true, 0, 0};

static uint16_t g_sample_hz = SAMPLE_HZ_DEFAULT;
static uint32_t g_sample_last_us = 0;
static uint8_t g_acc_preset = 0;
static uint8_t g_gyr_preset = 0;
static bool g_filter_on = false;

static bool g_sd_ready = false;
static bool g_sd_recording = false;
static File g_sd_file;
static uint64_t g_generated_samples = 0;
static uint64_t g_sd_saved_samples = 0;
static uint64_t g_sd_write_errors = 0;
static uint32_t g_max_sd_write_us = 0;
static uint32_t g_max_loop_us = 0;
static uint32_t g_loop_overruns = 0;
static uint64_t g_prof_samples = 0;
static uint64_t g_prof_imu_sum_us = 0;
static uint64_t g_prof_mag_sum_us = 0;
static uint64_t g_prof_vqf_sum_us = 0;
static uint64_t g_prof_vqf_mag_sum_us = 0;
static uint64_t g_prof_quat_sum_us = 0;
static uint64_t g_prof_serial_sum_us = 0;
static uint32_t g_prof_imu_max_us = 0;
static uint32_t g_prof_mag_max_us = 0;
static uint32_t g_prof_vqf_max_us = 0;
static uint32_t g_prof_vqf_mag_max_us = 0;
static uint32_t g_prof_quat_max_us = 0;
static uint32_t g_prof_serial_max_us = 0;
#if ENABLE_ESPNOW
static int64_t  g_clock_offset_us      = 0;
static bool     g_espnow_sync_received = false;
static uint32_t g_espnow_last_seq      = 0;
#endif
static char g_sd_path[48] = "/step_session.bin";
static char g_rec_session_id[33] = "none";
static char g_rec_state[32] = "idle";
static char g_transfer_state[16] = "none";
static char g_finalization_reason[32] = "none";
static char g_last_rec_error[32] = "none";
static uint64_t g_final_file_size = 0;
static uint32_t g_final_file_checksum = 0;
static uint32_t g_rec_grace_deadline_ms = 0;
static bool g_transfer_active = false;

static QueueHandle_t g_sd_queue = nullptr;
static SemaphoreHandle_t g_sd_mutex = nullptr;

static const float kAccLsbPerG[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
static const float kGyrLsbPerDps[4] = {131.072f, 65.536f, 32.768f, 16.384f};
static const float kStdGravityMps2 = 9.80665f;
static const float kMagUnitsPerLsb = 0.15f;  // AK09916, matches Plugin sensor_fusion ICM20948
static const float kMagRateHz = 100.0f;
static const uint32_t kMagPollPeriodUs = 10000UL;
static const uint32_t kIcmSpiHz = 4000000UL;
static const uint8_t kIcmGyroSmplrtDiv = 0;
static const uint16_t kIcmAccelSmplrtDiv = 0;
static const uint8_t kIcmDlpfCfg = 0;
static const bool kIcmDlpfEnabled = false;

static VQF g_vqf;
static bool g_vqf_inited = false;
static int16_t g_last_mag[3] = {0, 0, 0};
static uint32_t g_last_mag_poll_us = 0;
static bool g_have_mag = false;

static bool useWifi() { return ENABLE_TCP; }

static void profAdd(uint32_t elapsed_us, uint64_t *sum_us, uint32_t *max_us) {
  *sum_us += elapsed_us;
  if (elapsed_us > *max_us) *max_us = elapsed_us;
}

static uint32_t profAvg(uint64_t sum_us) {
  if (g_prof_samples == 0) return 0;
  return (uint32_t)(sum_us / g_prof_samples);
}

static void profReset() {
  g_prof_samples = 0;
  g_prof_imu_sum_us = 0;
  g_prof_mag_sum_us = 0;
  g_prof_vqf_sum_us = 0;
  g_prof_vqf_mag_sum_us = 0;
  g_prof_quat_sum_us = 0;
  g_prof_serial_sum_us = 0;
  g_prof_imu_max_us = 0;
  g_prof_mag_max_us = 0;
  g_prof_vqf_max_us = 0;
  g_prof_vqf_mag_max_us = 0;
  g_prof_quat_max_us = 0;
  g_prof_serial_max_us = 0;
}

static uint8_t icmConfig1(uint8_t fs_sel) {
  const uint8_t fchoice = kIcmDlpfEnabled ? 1u : 0u;
  return (uint8_t)(((kIcmDlpfCfg & 0x07u) << 3) | ((fs_sel & 0x03u) << 1) | fchoice);
}

static int16_t floatToQ15(float v) {
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  return (int16_t)(v * 32767.0f);
}

static void icmApplyRangePresets() {
  if (!icm_ok) return;
  const uint8_t acc_fs = g_acc_preset & 3u;
  const uint8_t gyr_fs = g_gyr_preset & 3u;
  icmSelectBank(0, 2);
  icmWrite(ICM_BANK2_ACCEL_CONFIG_1, icmConfig1(acc_fs));
  icmWrite(ICM_BANK2_GYRO_CONFIG_1, icmConfig1(gyr_fs));
  icmSelectBank(0, 0);
#if !SERIAL_OUTPUT_BINARY
  Serial.printf("ICM range: ACC preset %u  GYR preset %u\n", acc_fs, gyr_fs);
#endif
}

static void icmConfigureOutputRate() {
  icmSelectBank(0, 2);
  icmWrite(ICM_BANK2_GYRO_SMPLRT_DIV, kIcmGyroSmplrtDiv);
  icmWrite(ICM_BANK2_ACCEL_SMPLRT_DIV_1, (uint8_t)(kIcmAccelSmplrtDiv >> 8));
  icmWrite(ICM_BANK2_ACCEL_SMPLRT_DIV_2, (uint8_t)(kIcmAccelSmplrtDiv & 0xFF));
  icmWrite(ICM_BANK2_ODR_ALIGN_EN, 0x01);
  icmSelectBank(0, 0);
}

static void vqfReinitFilter() {
  const float hz = (float)g_sample_hz;
  if (hz < 1.0f) return;
  const float imu_ts = 1.0f / hz;
  const float mag_ts = 1.0f / kMagRateHz;
  vqf_init(&g_vqf, imu_ts, imu_ts, mag_ts);
  g_vqf_inited = true;
}

static void imuRawToVqfPhysical(const int16_t imu[6], vqf_real_t acc[3], vqf_real_t gyr[3]) {
  const float acc_mps2_per_lsb =
      kStdGravityMps2 / kAccLsbPerG[g_acc_preset & 3u];
  const float gyr_rads_per_lsb =
      (float)(M_PI / 180.0) / kGyrLsbPerDps[g_gyr_preset & 3u];
  acc[0] = (vqf_real_t)((float)imu[0] * acc_mps2_per_lsb);
  acc[1] = (vqf_real_t)((float)imu[1] * acc_mps2_per_lsb);
  acc[2] = (vqf_real_t)((float)imu[2] * acc_mps2_per_lsb);
  gyr[0] = (vqf_real_t)((float)imu[3] * gyr_rads_per_lsb);
  gyr[1] = (vqf_real_t)((float)imu[4] * gyr_rads_per_lsb);
  gyr[2] = (vqf_real_t)((float)imu[5] * gyr_rads_per_lsb);
}

static void vqfUpdateFromImu(const int16_t imu[6], const int16_t *mag_or_null,
                             bool mag_fresh) {
  if (!g_vqf_inited)
    vqfReinitFilter();

  vqf_real_t acc[3], gyr[3];
  imuRawToVqfPhysical(imu, acc, gyr);
  uint32_t prof_start_us = micros();
  vqf_update(&g_vqf, gyr, acc);
  profAdd((uint32_t)(micros() - prof_start_us), &g_prof_vqf_sum_us, &g_prof_vqf_max_us);

  if (mag_or_null != nullptr && mag_fresh) {
    vqf_real_t mag[3];
    mag[0] = (vqf_real_t)((float)mag_or_null[0] * kMagUnitsPerLsb);
    mag[1] = (vqf_real_t)((float)mag_or_null[1] * kMagUnitsPerLsb);
    mag[2] = (vqf_real_t)((float)mag_or_null[2] * kMagUnitsPerLsb);
    prof_start_us = micros();
    vqf_update_mag(&g_vqf, mag);
    profAdd((uint32_t)(micros() - prof_start_us), &g_prof_vqf_mag_sum_us,
            &g_prof_vqf_mag_max_us);
  }
}

static void vqfReadQuatQ15(int16_t out[4], bool use_9d) {
  vqf_real_t quat[4];
  uint32_t prof_start_us = micros();
  if (use_9d)
    vqf_get_quat_9d(&g_vqf, quat);
  else
    vqf_get_quat_6d(&g_vqf, quat);
  profAdd((uint32_t)(micros() - prof_start_us), &g_prof_quat_sum_us,
          &g_prof_quat_max_us);
  out[0] = floatToQ15((float)quat[0]);
  out[1] = floatToQ15((float)quat[1]);
  out[2] = floatToQ15((float)quat[2]);
  out[3] = floatToQ15((float)quat[3]);
}

static void packChannelsFromImu(const int16_t imu[6], const int16_t *mag_or_null,
                                bool mag_fresh) {
  channels[0] = imu[0];
  channels[1] = imu[1];
  channels[2] = imu[2];
  channels[3] = imu[3];
  channels[4] = imu[4];
  channels[5] = imu[5];
  channels[6] = mag_or_null != nullptr ? mag_or_null[0] : 0;
  channels[7] = mag_or_null != nullptr ? mag_or_null[1] : 0;
  channels[8] = mag_or_null != nullptr ? mag_or_null[2] : 0;
  channels[9] = 0;
  channels[10] = 0;
  channels[11] = 0;
  channels[12] = 0;
  channels[13] = packDioCh6();

  if (!g_filter_on)
    return;

  vqfUpdateFromImu(imu, mag_or_null, mag_fresh);
  vqfReadQuatQ15(&channels[9], mag_or_null != nullptr && g_have_mag);
}

static int parseFreqHz(const String &line) {
  int idx = line.indexOf(':');
  String tail = (idx >= 0) ? line.substring(idx + 1) : line.substring(4);
  tail.trim();
  return tail.toInt();
}

static bool sampleHzValid(int hz) {
  return hz >= 1 && hz <= 65535;
}

static bool handleCfgLine(const String &line) {
  if (!line.startsWith("CFG ")) return false;
  int si = -1, preset = -1;
  char kind[8] = {};
  if (sscanf(line.c_str(), "CFG %d %7s %d", &si, kind, &preset) < 3) return false;
  if (si != 0) {
    replyToHost("ERROR CFG: sensor index must be 0 on ESP32 node\n");
    return true;
  }
  if (strncmp(kind, "ACC", 3) == 0) {
    g_acc_preset = (uint8_t)constrain(preset, 0, 3);
    icmApplyRangePresets();
    replyToHost("OK CFG ACC\n");
    return true;
  }
  if (strncmp(kind, "GYR", 3) == 0) {
    g_gyr_preset = (uint8_t)constrain(preset, 0, 3);
    icmApplyRangePresets();
    replyToHost("OK CFG GYR\n");
    return true;
  }
  if (strncmp(kind, "SRATE", 5) == 0) {
    if (!sampleHzValid(preset)) {
      replyToHost("ERROR CFG: SRATE Hz must be >= 1\n");
      return true;
    }
    int hz = preset;
    g_sample_hz = (uint16_t)hz;
    g_sample_last_us = 0;
    if (g_filter_on)
      vqfReinitFilter();
    char buf[48];
    snprintf(buf, sizeof(buf), "OK FREQ:%d\n", hz);
    replyToHost(buf);
    return true;
  }
  replyToHost("ERROR CFG: unknown field\n");
  return true;
}

typedef struct {
  uint32_t seq;
  int64_t time_us;
} SyncPacket;

#if ENABLE_ESPNOW
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  if (NODE_IS_MASTER) return;
  if (len < (int)sizeof(SyncPacket)) return;
  const SyncPacket *pkt = (const SyncPacket *)data;
  int64_t recv_us = (int64_t)esp_timer_get_time();
  g_clock_offset_us      = (int64_t)pkt->time_us - recv_us;
  g_espnow_sync_received = true;
  g_espnow_last_seq      = pkt->seq;
}
void onEspNowSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  (void)tx_info; (void)status;
}
#endif

static void icmSpiBegin() {
  pinMode(PIN_ICM_CS, OUTPUT);
  digitalWrite(PIN_ICM_CS, HIGH);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
}

static bool icmWriteAddr(uint8_t addr, uint8_t reg, uint8_t val) {
  (void)addr;
  SPI.beginTransaction(SPISettings(kIcmSpiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_ICM_CS, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(PIN_ICM_CS, HIGH);
  SPI.endTransaction();
  return true;
}

static bool icmReadAddr(uint8_t addr, uint8_t reg, uint8_t *val) {
  (void)addr;
  SPI.beginTransaction(SPISettings(kIcmSpiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_ICM_CS, LOW);
  SPI.transfer(reg | 0x80);
  *val = SPI.transfer(0x00);
  digitalWrite(PIN_ICM_CS, HIGH);
  SPI.endTransaction();
  return true;
}

static bool icmReadBytes(uint8_t reg, uint8_t *buf, size_t len) {
  SPI.beginTransaction(SPISettings(kIcmSpiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_ICM_CS, LOW);
  SPI.transfer(reg | 0x80);
  for (size_t i = 0; i < len; i++) {
    buf[i] = SPI.transfer(0x00);
  }
  digitalWrite(PIN_ICM_CS, HIGH);
  SPI.endTransaction();
  return true;
}

static void icmSelectBank(uint8_t addr, uint8_t bank) {
  icmWriteAddr(addr, ICM_REG_BANK_SEL, (uint8_t)((bank & 0x03) << 4));
}

static bool icmReadWhoAmI(uint8_t addr, uint8_t *who) {
  icmSelectBank(addr, 0);
  return icmReadAddr(addr, ICM_WHO_AM_I, who);
}

static bool icmWrite(uint8_t reg, uint8_t val) {
  return icmWriteAddr(0, reg, val);
}

static bool icmReadReg(uint8_t reg, uint8_t *val) {
  return icmReadAddr(0, reg, val);
}

static void printBootDiagnostics() {
#if BOOT_DIAGNOSTICS
  Serial.println();
  Serial.println("========================================");
  Serial.println("  STEP ESP32-S3 NODE — BOOT DIAGNOSTICS");
  Serial.println("========================================");
  Serial.printf("Firmware: %s\n", FIRMWARE_VERSION);
  Serial.printf("Board target: XIAO_ESP32S3 (Sense)\n");
  Serial.printf("SPI SCK: GPIO%d (D8)  MISO: GPIO%d (D9)  MOSI: GPIO%d (D10)  ICM CS: GPIO%d (D7)\n",
                PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_ICM_CS);
  Serial.printf("ICM ODR: gyro_div=%u accel_div=%u odr_align=1 dlpf=%s dlpf_cfg=%u\n",
                kIcmGyroSmplrtDiv, kIcmAccelSmplrtDiv,
                kIcmDlpfEnabled ? "on" : "off", kIcmDlpfCfg);
  Serial.printf("Mag poll: %.0f Hz, cached between polls\n", kMagRateHz);
  Serial.printf("Sample rate: %d Hz  channels: %d\n", g_sample_hz, NUM_CHANNELS);
  Serial.println("--- ICM20948 SPI WHO_AM_I (expect 0xEA) ---");
  icmSpiBegin();
  delay(50);
  uint8_t spi_who = 0;
  icmReadWhoAmI(0, &spi_who);
  Serial.printf("  CS GPIO%d -> WHO_AM_I 0x%02X %s\n", PIN_ICM_CS, spi_who,
                spi_who == ICM20948_WHOAMI_VAL ? "OK" : "unexpected");
  Serial.println("========================================");
#endif
}

static void initDio() {
  pinMode(PIN_DIO, INPUT_PULLUP);
  bool level = digitalRead(PIN_DIO);
  dio_state.stable_high = level;
  dio_state.pending_raw = level;
  dio_state.pending_since_ms = millis();
  Serial.printf("DIO: GPIO%d (pad D0) pull-up — initial level=%d (1=idle, 0=GND)\n",
                PIN_DIO, level ? 1 : 0);
  Serial.println("DIO: input active for sync/commands; not included in 13-channel OE stream");
}

static void updateDio() {
  bool raw = digitalRead(PIN_DIO);
  uint32_t now = millis();
  if (raw != dio_state.pending_raw) {
    dio_state.pending_raw = raw;
    dio_state.pending_since_ms = now;
  }
  if ((now - dio_state.pending_since_ms) >= (uint32_t)DIO_DEBOUNCE_MS &&
      dio_state.pending_raw != dio_state.stable_high) {
    dio_state.stable_high = dio_state.pending_raw;
    if (dio_state.edge_count < 0x7FFF) {
      dio_state.edge_count++;
    }
  }
}

static int16_t packDioCh6() {
  uint16_t packed = (dio_state.stable_high ? 1u : 0u) |
                    ((uint32_t)(dio_state.edge_count & 0x7FFFu) << 1);
  return (int16_t)packed;
}

static void icmAuxWriteByte(uint8_t slave_addr, uint8_t reg, uint8_t val) {
  icmSelectBank(0, 3);
  icmWrite(ICM_I2C_SLV0_ADDR, slave_addr);
  icmWrite(ICM_I2C_SLV0_REG, reg);
  icmWrite(ICM_I2C_SLV0_DO, val);
  icmWrite(ICM_I2C_SLV0_CTRL, 0x81);
  delay(10);
  icmWrite(ICM_I2C_SLV0_CTRL, 0x00);
  icmSelectBank(0, 0);
}

static uint8_t icmAuxReadByte(uint8_t slave_addr, uint8_t reg) {
  icmSelectBank(0, 3);
  icmWrite(ICM_I2C_SLV0_ADDR, (uint8_t)(0x80 | slave_addr));
  icmWrite(ICM_I2C_SLV0_REG, reg);
  icmWrite(ICM_I2C_SLV0_CTRL, 0x81);
  delay(10);
  icmSelectBank(0, 0);
  uint8_t val = 0;
  icmReadReg(ICM_EXT_SENS_DATA_00, &val);
  icmSelectBank(0, 3);
  icmWrite(ICM_I2C_SLV0_CTRL, 0x00);
  icmSelectBank(0, 0);
  return val;
}

static bool initIcm20948() {
  icmSpiBegin();
  uint8_t who = 0;
  if (!icmReadWhoAmI(0, &who) || who != ICM20948_WHOAMI_VAL) {
    Serial.printf("ICM20948: synthetic fallback - SPI WHO_AM_I=0x%02X (expected 0xEA)\n", who);
    return false;
  }

  icmSelectBank(0, 0);
  icmWrite(ICM_PWR_MGMT_1, 0x01);
  delay(100);
  icmConfigureOutputRate();
  Serial.printf("ICM20948: OK on SPI CS GPIO%d WHO_AM_I=0xEA\n", PIN_ICM_CS);
  Serial.printf("ICM20948: ODR gyro_div=%u accel_div=%u odr_align=1 dlpf=%s cfg=%u\n",
                kIcmGyroSmplrtDiv, kIcmAccelSmplrtDiv,
                kIcmDlpfEnabled ? "on" : "off", kIcmDlpfCfg);
  icmApplyRangePresets();
  return true;
}

static bool readImuRaw(int16_t out[6]) {
  uint8_t raw[14];
  icmSelectBank(0, 0);
  if (!icmReadBytes(ICM_ACCEL_XOUT_H, raw, sizeof(raw))) return false;

  auto read16be = [](const uint8_t *p) {
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
  };
  out[0] = read16be(&raw[0]);
  out[1] = read16be(&raw[2]);
  out[2] = read16be(&raw[4]);
  out[3] = read16be(&raw[8]);
  out[4] = read16be(&raw[10]);
  out[5] = read16be(&raw[12]);
  return true;
}

static bool initAk09916() {
  if (!icm_ok) return false;

  icmSelectBank(0, 0);
  icmWrite(ICM_USER_CTRL, 0x20);
  delay(10);
  icmSelectBank(0, 3);
  icmWrite(ICM_I2C_MST_CTRL, 0x07);
  delay(10);
  icmSelectBank(0, 0);

  uint8_t who = icmAuxReadByte(AK09916_ADDR, AK09916_WIA2);
  if (who != AK09916_WIA2_VAL) {
    Serial.printf("AK09916: unavailable through ICM SPI aux bus WIA2=0x%02X\n", who);
    return false;
  }

  icmAuxWriteByte(AK09916_ADDR, AK09916_CNTL3, 0x01);
  delay(10);
  icmAuxWriteByte(AK09916_ADDR, AK09916_CNTL2, AK09916_MODE_CONT_100HZ);

  icmSelectBank(0, 3);
  icmWrite(ICM_I2C_SLV0_ADDR, (uint8_t)(0x80 | AK09916_ADDR));
  icmWrite(ICM_I2C_SLV0_REG, AK09916_ST1);
  icmWrite(ICM_I2C_SLV0_CTRL, 0x88);
  delay(10);
  icmSelectBank(0, 0);

  Serial.println("AK09916: OK through ICM SPI aux bus, continuous 100 Hz, cached between polls");
  return true;
}

static bool readMagRaw(int16_t out[3], bool *fresh) {
  if (fresh != nullptr) *fresh = false;
  if (!mag_ok) return false;

  uint8_t mag_raw[8];
  icmSelectBank(0, 0);
  if (!icmReadBytes(ICM_EXT_SENS_DATA_00, mag_raw, sizeof(mag_raw))) return false;

  if ((mag_raw[0] & 0x01) == 0) {
    if (g_have_mag) {
      out[0] = g_last_mag[0];
      out[1] = g_last_mag[1];
      out[2] = g_last_mag[2];
      return true;
    }
    return false;
  }

  uint8_t st2 = mag_raw[7];
  if ((st2 & 0x08) != 0) {
    if (g_have_mag) {
      out[0] = g_last_mag[0];
      out[1] = g_last_mag[1];
      out[2] = g_last_mag[2];
      return true;
    }
    return false;
  }

  g_last_mag[0] = out[0] = (int16_t)((uint16_t)mag_raw[1] | ((uint16_t)mag_raw[2] << 8));
  g_last_mag[1] = out[1] = (int16_t)((uint16_t)mag_raw[3] | ((uint16_t)mag_raw[4] << 8));
  g_last_mag[2] = out[2] = (int16_t)((uint16_t)mag_raw[5] | ((uint16_t)mag_raw[6] << 8));
  g_have_mag = true;
  if (fresh != nullptr) *fresh = true;
  return true;
}

static void readImu(int16_t out[6]) {
  if (icm_ok && readImuRaw(out)) return;

  float t = millis() * 0.01f;
  out[0] = (int16_t)(1000 * sinf(t));
  out[1] = (int16_t)(500 * cosf(t));
  out[2] = 16384;
  out[3] = out[4] = out[5] = 0;
}

static void readMag(int16_t out[3], bool *fresh) {
  if (fresh != nullptr) *fresh = false;

  const uint32_t now_us = micros();
  const bool poll_due = !g_have_mag || g_last_mag_poll_us == 0 ||
                        (uint32_t)(now_us - g_last_mag_poll_us) >= kMagPollPeriodUs;
  if (poll_due) {
    g_last_mag_poll_us = now_us;
    if (readMagRaw(out, fresh)) return;
  }

  out[0] = g_last_mag[0];
  out[1] = g_last_mag[1];
  out[2] = g_last_mag[2];
}

// Open Ephys header offset field (int32 LE): low 32 bits of esp_timer_get_time() µs since boot.
// offset==0 = legacy frames (host/Plugin use arrival-time pacing). Same clock on every slave;
// cross-board alignment needs START pulse or host merge — see docs/open-ephys-plugin.md.
static void fillOeHeader(OeHeader *hdr) {
  int64_t t_us = (int64_t)esp_timer_get_time();
#if ENABLE_ESPNOW
  if (!NODE_IS_MASTER && g_espnow_sync_received) t_us += g_clock_offset_us;
#endif
  hdr->offset = (int32_t)(uint32_t)t_us;
  hdr->num_channels = NUM_CHANNELS;
  hdr->samples_per_channel = 1;
  hdr->element_size = 2;
  hdr->bit_depth = 3;  // Open Ephys Ephys Socket: OpenCV S16 enum
  hdr->num_bytes = NUM_CHANNELS * 1 * 2;
}

#if ENABLE_ESPNOW
static void sendEspNowSync() {
  if (!NODE_IS_MASTER || !wifi_up) return;
  SyncPacket pkt = {seq, (int64_t)esp_timer_get_time()};
  uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_send(bcast, (uint8_t *)&pkt, sizeof(pkt));
}
#else
static void sendEspNowSync() {}
#endif

static void packAndSendTcp() {
  if (!client || !client.connected() || !streaming) return;
  OeHeader hdr;
  fillOeHeader(&hdr);
  client.write((uint8_t *)&hdr, sizeof(hdr));
  client.write((uint8_t *)channels, sizeof(channels));
}

static void sendSerialBench() {
#if ENABLE_SERIAL_BENCH
  if (g_transfer_active || !streaming) return;
  if (!csv_banner_sent) {
    Serial.printf("# STEP boot complete icm=%s spi_cs=%d mag=%s channels=ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz\n",
                  icm_ok ? "OK" : "FALLBACK", PIN_ICM_CS, mag_ok ? "OK" : "FALLBACK");
    csv_banner_sent = true;
  }
#if SERIAL_OUTPUT_BINARY
  OeHeader hdr;
  fillOeHeader(&hdr);
  Serial.write((uint8_t *)&hdr, sizeof(hdr));
  Serial.write((uint8_t *)channels, sizeof(channels));
#else
  Serial.printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                (unsigned long)seq, channels[0], channels[1], channels[2],
                channels[3], channels[4], channels[5], channels[6], channels[7],
                channels[8], channels[9], channels[10], channels[11], channels[12],
                channels[13]);
#endif
#endif
}

static void sdRecordStop();
static void recReplyToHost(const char *text);

static void controlPrintf(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  recReplyToHost(buf);
}

static uint32_t recCrc32Update(uint32_t crc, const uint8_t *data, size_t len) {
  crc = ~crc;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

static uint32_t checksumSdFile(const char *path, uint64_t *size_out) {
#if ENABLE_SD
  uint8_t buf[128];
  uint32_t crc = 0;
  uint64_t total = 0;
  File f = SD.open(path, FILE_READ);
  if (!f) {
    if (size_out) *size_out = 0;
    return 0;
  }
  while (f.available()) {
    size_t n = f.read(buf, sizeof(buf));
    if (n == 0) break;
    crc = recCrc32Update(crc, buf, n);
    total += n;
  }
  f.close();
  if (size_out) *size_out = total;
  return crc;
#else
  if (size_out) *size_out = 0;
  return 0;
#endif
}

static void makeSessionId() {
  snprintf(g_rec_session_id, sizeof(g_rec_session_id), "%08lx%08lx",
           (unsigned long)millis(), (unsigned long)seq);
}

static uint32_t recGraceRemainingMs() {
  if (strcmp(g_rec_state, "host-disconnected-grace") != 0 || g_rec_grace_deadline_ms == 0)
    return 0;
  uint32_t now = millis();
  return (int32_t)(g_rec_grace_deadline_ms - now) > 0 ? g_rec_grace_deadline_ms - now : 0;
}

static void recMarkControlConnected() {
  if (strcmp(g_rec_state, "host-disconnected-grace") == 0) {
    strncpy(g_rec_state, "recording", sizeof(g_rec_state) - 1);
    g_rec_grace_deadline_ms = 0;
  }
}

static void recMarkControlDisconnected() {
  if (g_sd_recording && strcmp(g_rec_state, "recording") == 0) {
    strncpy(g_rec_state, "host-disconnected-grace", sizeof(g_rec_state) - 1);
    g_rec_grace_deadline_ms = millis() + REC_RECONNECT_GRACE_MS;
  }
}

static void recMaybeFinalizeTimeout() {
  if (strcmp(g_rec_state, "host-disconnected-grace") == 0 && recGraceRemainingMs() == 0) {
    strncpy(g_finalization_reason, "disconnect_timeout", sizeof(g_finalization_reason) - 1);
    sdRecordStop();
  }
}

static void recWriteBytes(const uint8_t *data, size_t len) {
#if ENABLE_TCP && !ENABLE_SERIAL_BENCH
  if (client && client.connected()) client.write(data, len);
#else
  Serial.write(data, len);
#endif
}

static void writeSdrfFrame(const char *session_id, uint8_t type, uint32_t chunk_index,
                           uint64_t offset, const uint8_t *payload, uint32_t payload_len,
                           uint64_t total_size, uint32_t flags) {
  SdrfHeader hdr = {};
  hdr.magic[0] = 'S'; hdr.magic[1] = 'D'; hdr.magic[2] = 'R'; hdr.magic[3] = 'F';
  hdr.frame_version = 1;
  hdr.frame_type = type;
  hdr.header_length = SDRF_HEADER_LEN;
  size_t sid_len = strlen(session_id);
  if (sid_len > sizeof(hdr.session_id)) sid_len = sizeof(hdr.session_id);
  memcpy(hdr.session_id, session_id, sid_len);
  hdr.chunk_index = chunk_index;
  hdr.byte_offset = offset;
  hdr.payload_length = payload_len;
  hdr.total_file_size = total_size;
  hdr.payload_checksum = payload && payload_len ? recCrc32Update(0, payload, payload_len) : 0;
  hdr.flags = flags;
  hdr.header_crc32 = 0;
  hdr.header_crc32 = recCrc32Update(0, (const uint8_t *)&hdr, sizeof(hdr));
  recWriteBytes((uint8_t *)&hdr, sizeof(hdr));
  if (payload && payload_len) recWriteBytes(payload, payload_len);
}

#if ENABLE_SD
static void sdWriteTask(void *) {
  SdLogRecord rec;
  uint32_t last_flush_ms = 0;
  while (true) {
    if (g_sd_queue && xQueueReceive(g_sd_queue, &rec, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (g_sd_mutex && xSemaphoreTake(g_sd_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (g_sd_file) {
          uint32_t t0 = micros();
          size_t written = g_sd_file.write((uint8_t *)&rec, sizeof(rec));
          uint32_t wu = (uint32_t)(micros() - t0);
          if (wu > g_max_sd_write_us) g_max_sd_write_us = wu;
          if (written == sizeof(rec)) g_sd_saved_samples++;
          else g_sd_write_errors++;
        }
        xSemaphoreGive(g_sd_mutex);
      }
    }
    uint32_t now_ms = millis();
    if ((now_ms - last_flush_ms) >= 1000) {
      last_flush_ms = now_ms;
      if (g_sd_mutex && xSemaphoreTake(g_sd_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (g_sd_file) g_sd_file.flush();
        xSemaphoreGive(g_sd_mutex);
      }
    }
  }
}
#endif

static void logSd() {
#if ENABLE_SD
  if (!g_sd_recording || !g_sd_queue) return;
  SdLogRecord rec = {};
  rec.seq = seq;
  int64_t t_us = (int64_t)esp_timer_get_time();
#if ENABLE_ESPNOW
  if (!NODE_IS_MASTER && g_espnow_sync_received) t_us += g_clock_offset_us;
#endif
  rec.time_us = t_us;
  memcpy(rec.ch, channels, sizeof(channels));
  if (xQueueSend(g_sd_queue, &rec, 0) != pdTRUE) {
    g_sd_write_errors++;
  }
#endif
}

static bool sdEnsureReady() {
#if ENABLE_SD
  return g_sd_ready;
#else
  return false;
#endif
}

static bool sdRecordStart(const char *path_or_null) {
#if ENABLE_SD
  if (!sdEnsureReady()) {
    g_sd_write_errors++;
    strncpy(g_rec_state, "failed", sizeof(g_rec_state) - 1);
    strncpy(g_last_rec_error, "sd_not_ready", sizeof(g_last_rec_error) - 1);
    controlPrintf("SD_STATUS enabled=1 ready=0 recording=0 error=begin_failed\n");
    return false;
  }

  if (g_sd_recording && g_sd_file) {
    g_sd_recording = false;
    if (g_sd_queue) while (uxQueueMessagesWaiting(g_sd_queue) > 0) vTaskDelay(pdMS_TO_TICKS(10));
    vTaskDelay(pdMS_TO_TICKS(30));
    if (g_sd_mutex) xSemaphoreTake(g_sd_mutex, portMAX_DELAY);
    g_sd_file.flush();
    g_sd_file.close();
    if (g_sd_mutex) xSemaphoreGive(g_sd_mutex);
  }

  makeSessionId();
  if (path_or_null && path_or_null[0]) {
    strncpy(g_sd_path, path_or_null, sizeof(g_sd_path) - 1);
    g_sd_path[sizeof(g_sd_path) - 1] = '\0';
  } else {
    snprintf(g_sd_path, sizeof(g_sd_path), "/step_%s.bin", g_rec_session_id);
  }

  if (g_sd_mutex) xSemaphoreTake(g_sd_mutex, portMAX_DELAY);
  g_sd_file = SD.open(g_sd_path, FILE_WRITE);
  if (g_sd_mutex) xSemaphoreGive(g_sd_mutex);
  if (!g_sd_file) {
    g_sd_recording = false;
    g_sd_write_errors++;
    strncpy(g_rec_state, "failed", sizeof(g_rec_state) - 1);
    strncpy(g_last_rec_error, "open_failed", sizeof(g_last_rec_error) - 1);
    controlPrintf("SD_STATUS enabled=1 ready=1 recording=0 error=open_failed path=%s\n", g_sd_path);
    return false;
  }

  g_sd_saved_samples = 0;
  g_sd_write_errors = 0;
  g_max_sd_write_us = 0;
  g_final_file_size = 0;
  g_final_file_checksum = 0;
  strncpy(g_rec_state, "starting", sizeof(g_rec_state) - 1);
  strncpy(g_transfer_state, "none", sizeof(g_transfer_state) - 1);
  strncpy(g_finalization_reason, "none", sizeof(g_finalization_reason) - 1);
  strncpy(g_last_rec_error, "none", sizeof(g_last_rec_error) - 1);

  SdLogHeader hdr = {};
  hdr.magic = SD_LOG_MAGIC;
  hdr.version = SD_LOG_VERSION;
  hdr.record_size = sizeof(SdLogRecord);
  hdr.sample_hz = g_sample_hz;
  hdr.channel_count = NUM_CHANNELS;
  hdr.start_time_us = (int64_t)esp_timer_get_time();
  size_t written = g_sd_file.write((uint8_t *)&hdr, sizeof(hdr));
  if (written != sizeof(hdr)) {
    g_sd_write_errors++;
  }

  g_sd_recording = true;
  strncpy(g_rec_state, "recording", sizeof(g_rec_state) - 1);
  controlPrintf("SD_STATUS enabled=1 ready=1 recording=1 path=%s sample_hz=%u\n",
                g_sd_path, (unsigned)g_sample_hz);
  return true;
#else
  controlPrintf("SD_STATUS enabled=0 ready=0 recording=0 error=compile_disabled\n");
  return false;
#endif
}

static void sdRecordStop() {
#if ENABLE_SD
  strncpy(g_rec_state, "finalizing", sizeof(g_rec_state) - 1);
  g_sd_recording = false;
  if (g_sd_queue) while (uxQueueMessagesWaiting(g_sd_queue) > 0) vTaskDelay(pdMS_TO_TICKS(10));
  vTaskDelay(pdMS_TO_TICKS(30));
  if (g_sd_mutex) xSemaphoreTake(g_sd_mutex, portMAX_DELAY);
  if (g_sd_file) {
    g_sd_file.flush();
    g_sd_file.close();
  }
  if (g_sd_mutex) xSemaphoreGive(g_sd_mutex);
  controlPrintf("SD_FINAL ready=%d recording=0 path=%s saved=%llu errors=%llu max_sd_write_us=%lu overrun=%lu\n",
                g_sd_ready ? 1 : 0,
                g_sd_path,
                (unsigned long long)g_sd_saved_samples,
                (unsigned long long)g_sd_write_errors,
                (unsigned long)g_max_sd_write_us,
                (unsigned long)g_loop_overruns);
  g_final_file_checksum = checksumSdFile(g_sd_path, &g_final_file_size);
  if (strcmp(g_finalization_reason, "none") == 0) {
    strncpy(g_finalization_reason, "manual_stop", sizeof(g_finalization_reason) - 1);
  }
  strncpy(g_rec_state, "finalized", sizeof(g_rec_state) - 1);
  g_rec_grace_deadline_ms = 0;
  controlPrintf("SD_STATUS enabled=1 ready=%d recording=0 path=%s saved=%llu errors=%llu max_sd_write_us=%lu\n",
                g_sd_ready ? 1 : 0,
                g_sd_path,
                (unsigned long long)g_sd_saved_samples,
                (unsigned long long)g_sd_write_errors,
                (unsigned long)g_max_sd_write_us);
#else
  controlPrintf("SD_STATUS enabled=0 ready=0 recording=0 error=compile_disabled\n");
#endif
}

static void printAcqStatus() {
  Serial.printf("STATUS seq=%lu generated=%llu sample_hz=%u filter=%d streaming=%d "
                "icm_ok=%d mag_ok=%d "
                "sd_enabled=%d sd_ready=%d sd_recording=%d sd_saved=%llu sd_errors=%llu "
                "max_sd_write_us=%lu max_loop_us=%lu loop_overruns=%lu "
                "prof_n=%llu "
                "avg_imu_us=%lu max_imu_us=%lu "
                "avg_mag_us=%lu max_mag_us=%lu "
                "avg_vqf_us=%lu max_vqf_us=%lu "
                "avg_vqf_mag_us=%lu max_vqf_mag_us=%lu "
                "avg_quat_us=%lu max_quat_us=%lu "
                "avg_serial_us=%lu max_serial_us=%lu\n",
                (unsigned long)seq,
                (unsigned long long)g_generated_samples,
                (unsigned)g_sample_hz,
                g_filter_on ? 1 : 0,
                streaming ? 1 : 0,
                icm_ok ? 1 : 0,
                mag_ok ? 1 : 0,
#if ENABLE_SD
                1,
#else
                0,
#endif
                g_sd_ready ? 1 : 0,
                g_sd_recording ? 1 : 0,
                (unsigned long long)g_sd_saved_samples,
                (unsigned long long)g_sd_write_errors,
                (unsigned long)g_max_sd_write_us,
                (unsigned long)g_max_loop_us,
                (unsigned long)g_loop_overruns,
                (unsigned long long)g_prof_samples,
                (unsigned long)profAvg(g_prof_imu_sum_us),
                (unsigned long)g_prof_imu_max_us,
                (unsigned long)profAvg(g_prof_mag_sum_us),
                (unsigned long)g_prof_mag_max_us,
                (unsigned long)profAvg(g_prof_vqf_sum_us),
                (unsigned long)g_prof_vqf_max_us,
                (unsigned long)profAvg(g_prof_vqf_mag_sum_us),
                (unsigned long)g_prof_vqf_mag_max_us,
                (unsigned long)profAvg(g_prof_quat_sum_us),
                (unsigned long)g_prof_quat_max_us,
                (unsigned long)profAvg(g_prof_serial_sum_us),
                (unsigned long)g_prof_serial_max_us);
#if ENABLE_ESPNOW
  Serial.printf("ESPNOW role=%s ch=%d sync=%d offset_us=%lld last_seq=%lu\n",
                NODE_IS_MASTER ? "master" : "slave",
                ESPNOW_WIFI_CHANNEL,
                g_espnow_sync_received ? 1 : 0,
                (long long)g_clock_offset_us,
                (unsigned long)g_espnow_last_seq);
#endif
}

static void recReplyToHost(const char *text);

static void printRecStatus() {
  char buf[1024];
  snprintf(buf, sizeof(buf),
                "REC STATUS_OK protocol=rec-v1 capabilities=record_control,status_v1,finalized_metadata,chunk_transfer_v1,whole_file_checksum,reconnect_grace,transfer_isolation=paused_isolated_stream transport=usb_bridge sd_ready=%d sd_open=%d recording_state=%s transfer_state=%s session_id=%s sd_path_token=sd:%s generated_samples=%llu saved_samples=%llu queue_drops=0 write_errors=%llu max_write_latency_us=%lu overrun_count=%lu finalization_reason=%s file_byte_size=%llu file_checksum=%08lx checksum_type=crc32 last_error=%s grace_ms_remaining=%lu local_result_path=unknown local_analyzer_result=unknown\n",
                g_sd_ready ? 1 : 0,
                g_sd_recording ? 1 : 0,
                g_rec_state,
                g_transfer_state,
                g_rec_session_id,
                g_rec_session_id,
                (unsigned long long)g_generated_samples,
                (unsigned long long)g_sd_saved_samples,
                (unsigned long long)g_sd_write_errors,
                (unsigned long)g_max_sd_write_us,
                (unsigned long)g_loop_overruns,
                g_finalization_reason,
                (unsigned long long)g_final_file_size,
                (unsigned long)g_final_file_checksum,
                g_last_rec_error,
                (unsigned long)recGraceRemainingMs());
  recReplyToHost(buf);
}

static void replyToHost(const char *text) {
#if ENABLE_TCP && !ENABLE_SERIAL_BENCH
  if (client && client.connected())
    client.print(text);
#else
  (void)text;  // Plugin USB path: bridge answers legacy handshakes on TCP.
#endif
}

static void recReplyToHost(const char *text) {
#if ENABLE_TCP && !ENABLE_SERIAL_BENCH
  if (client && client.connected())
    client.print(text);
#else
  Serial.print(text);
#endif
}

static bool recFieldValue(const String &line, const char *key, char *buf, size_t len) {
  String needle = String(key) + "=";
  int start = line.indexOf(needle);
  if (start < 0 || len == 0) return false;
  start += needle.length();
  int end = line.indexOf(' ', start);
  if (end < 0) end = line.length();
  String value = line.substring(start, end);
  value.toCharArray(buf, len);
  return true;
}

#define replyToHost recReplyToHost
static void handleRecLine(const String &line) {
  if (line.startsWith("REC HELLO")) {
    if (line.indexOf("protocol_min=rec-v1") < 0) {
      replyToHost("REC ERR code=unsupported_protocol retryable=false detail=protocol_min\n");
      return;
    }
    recMarkControlConnected();
    char buf[256];
    snprintf(buf, sizeof(buf),
             "REC HELLO_OK protocol=rec-v1 firmware=arduino-step-%s transport=usb_bridge capabilities=record_control,status_v1,finalized_metadata,chunk_transfer_v1,whole_file_checksum,reconnect_grace,transfer_isolation=paused_isolated_stream max_chunk=%lu analyzer=sd-bin-v1 grace_ms=%lu\n",
             FIRMWARE_VERSION, (unsigned long)REC_MAX_CHUNK, (unsigned long)REC_RECONNECT_GRACE_MS);
    replyToHost(buf);
    return;
  }

  if (line.startsWith("REC START")) {
    if (g_sd_recording) {
      char err[128];
      snprintf(err, sizeof(err), "REC ERR code=already_recording session_id=%s retryable=false detail=active\n", g_rec_session_id);
      replyToHost(err);
      return;
    }
    bool ok = sdRecordStart(nullptr);
    if (!ok) {
      replyToHost("REC ERR code=sd_not_ready retryable=true detail=start_failed\n");
      return;
    }
    char buf[192];
    snprintf(buf, sizeof(buf),
             "REC STARTED session_id=%s sd_path_token=sd:%s recording_state=recording generated_samples=%llu saved_samples=%llu\n",
             g_rec_session_id, g_rec_session_id,
             (unsigned long long)g_generated_samples,
             (unsigned long long)g_sd_saved_samples);
    replyToHost(buf);
    return;
  }

  if (line.startsWith("REC STATUS")) {
    printRecStatus();
    return;
  }

  if (line.startsWith("REC STOP")) {
    if (!g_sd_recording) {
      char err[128];
      snprintf(err, sizeof(err), "REC ERR code=not_recording session_id=%s retryable=false detail=idle\n", g_rec_session_id);
      replyToHost(err);
      return;
    }
    strncpy(g_finalization_reason, "manual_stop", sizeof(g_finalization_reason) - 1);
    char buf[96];
    snprintf(buf, sizeof(buf), "REC FINALIZING session_id=%s\n", g_rec_session_id);
    replyToHost(buf);
    sdRecordStop();
    snprintf(buf, sizeof(buf), "REC FINALIZED session_id=%s\n", g_rec_session_id);
    replyToHost(buf);
    return;
  }

  if (line.startsWith("REC SESSION")) {
    if (g_sd_recording) {
      replyToHost("REC ERR code=busy_recording retryable=true detail=session\n");
      return;
    }
    if (strcmp(g_rec_state, "finalized") != 0 || strcmp(g_rec_session_id, "none") == 0) {
      replyToHost("REC ERR code=not_found retryable=false detail=session\n");
      return;
    }
    char buf[256];
    snprintf(buf, sizeof(buf),
             "REC SESSION_OK session_id=%s sd_path_token=sd:%s file_size=%llu file_checksum=%08lx checksum_type=crc32 sample_count=%llu finalized_at=unknown finalization_reason=%s analyzer_format=sd-bin-v1\n",
             g_rec_session_id, g_rec_session_id,
             (unsigned long long)g_final_file_size,
             (unsigned long)g_final_file_checksum,
             (unsigned long long)g_sd_saved_samples,
             g_finalization_reason);
    replyToHost(buf);
    return;
  }

  if (line.startsWith("REC GET")) {
    if (g_sd_recording) {
      replyToHost("REC ERR code=busy_recording retryable=true detail=active\n");
      return;
    }
    if (strcmp(g_rec_state, "finalized") != 0) {
      replyToHost("REC ERR code=not_finalized retryable=true detail=session\n");
      return;
    }
    char off_buf[24], len_buf[16], idx_buf[16];
    uint64_t offset = recFieldValue(line, "offset", off_buf, sizeof(off_buf)) ? strtoull(off_buf, nullptr, 10) : 0;
    uint32_t length = recFieldValue(line, "length", len_buf, sizeof(len_buf)) ? (uint32_t)strtoul(len_buf, nullptr, 10) : REC_MAX_CHUNK;
    uint32_t chunk_index = recFieldValue(line, "chunk_index", idx_buf, sizeof(idx_buf)) ? (uint32_t)strtoul(idx_buf, nullptr, 10) : 0;
    if (offset > g_final_file_size) {
      replyToHost("REC ERR code=offset_out_of_range retryable=false detail=offset\n");
      return;
    }
    if (length > REC_MAX_CHUNK) length = REC_MAX_CHUNK;
    if (offset + length > g_final_file_size) length = (uint32_t)(g_final_file_size - offset);
    File f = SD.open(g_sd_path, FILE_READ);
    if (!f) {
      replyToHost("REC ERR code=sd_error retryable=true detail=read\n");
      return;
    }
    if (!f.seek(offset)) {
      f.close();
      replyToHost("REC ERR code=offset_out_of_range retryable=false detail=seek\n");
      return;
    }
    uint8_t buf[REC_MAX_CHUNK];
    size_t got = f.read(buf, length);
    f.close();
    strncpy(g_transfer_state, "chunking", sizeof(g_transfer_state) - 1);
    g_transfer_active = true;
    writeSdrfFrame(g_rec_session_id, SDRF_TYPE_DATA, chunk_index, offset, buf, got,
                   g_final_file_size, offset + got >= g_final_file_size ? 0x04 : 0);
    if (offset + got >= g_final_file_size) {
      writeSdrfFrame(g_rec_session_id, SDRF_TYPE_EOF, chunk_index + 1, g_final_file_size,
                     nullptr, 0, g_final_file_size, 0);
    }
    g_transfer_active = false;
    return;
  }

  if (line.startsWith("REC COMPLETE")) {
    strncpy(g_transfer_state, "complete", sizeof(g_transfer_state) - 1);
    char buf[96];
    snprintf(buf, sizeof(buf), "REC COMPLETE_OK session_id=%s transfer_state=complete\n", g_rec_session_id);
    replyToHost(buf);
    return;
  }

  if (line.startsWith("REC ABORT")) {
    strncpy(g_transfer_state, "aborted", sizeof(g_transfer_state) - 1);
    char buf[96];
    snprintf(buf, sizeof(buf), "REC ABORTED session_id=%s transfer_state=aborted\n", g_rec_session_id);
    replyToHost(buf);
    return;
  }

  if (line.startsWith("REC CLEAR")) {
    char scope[24];
    if (recFieldValue(line, "scope", scope, sizeof(scope)) && strcmp(scope, "errors") == 0) {
      strncpy(g_last_rec_error, "none", sizeof(g_last_rec_error) - 1);
      replyToHost("REC CLEAR_OK scope=errors\n");
      return;
    }
    if (recFieldValue(line, "scope", scope, sizeof(scope)) && strcmp(scope, "transfer") == 0) {
      strncpy(g_transfer_state, "none", sizeof(g_transfer_state) - 1);
      replyToHost("REC CLEAR_OK scope=transfer\n");
      return;
    }
    replyToHost("REC ERR code=invalid_scope retryable=false detail=clear\n");
    return;
  }

  replyToHost("REC ERR code=unsupported retryable=false detail=command\n");
}
#undef replyToHost

static void handleLine(const String &line) {
  if (line.startsWith("REC ")) {
    handleRecLine(line);
  } else if (line.startsWith("REDPITAYA")) {
    char buf[96];
    snprintf(buf, sizeof(buf),
             "%d channels; sample_rate=%u; node=esp32s3_arduino; filter=%s\n",
             NUM_CHANNELS, (unsigned)g_sample_hz, g_filter_on ? "on" : "off");
    replyToHost(buf);
    char okCh[24];
    snprintf(okCh, sizeof(okCh), "OK CHANNELS:%d\n", NUM_CHANNELS);
    replyToHost(okCh);
  } else if (line.startsWith("FREQ:") || line.startsWith("FREQ ")) {
    int hz = parseFreqHz(line);
    if (!sampleHzValid(hz)) {
      replyToHost("ERROR FREQ: Hz must be >= 1\n");
    } else {
      g_sample_hz = (uint16_t)hz;
      g_sample_last_us = 0;
      g_loop_overruns = 0;
      g_max_loop_us = 0;
      profReset();
      if (g_filter_on)
        vqfReinitFilter();
      char ok[32];
      snprintf(ok, sizeof(ok), "OK FREQ:%d\n", hz);
      replyToHost(ok);
#if !SERIAL_OUTPUT_BINARY
      Serial.printf("Sample rate set to %d Hz\n", hz);
#endif
    }
  } else if (line.startsWith("FILTER ON")) {
    g_filter_on = true;
    vqfReinitFilter();
    g_loop_overruns = 0;
    g_max_loop_us = 0;
    profReset();
    replyToHost("OK FILTER ON\n");
  } else if (line.startsWith("FILTER OFF")) {
    g_filter_on = false;
    g_loop_overruns = 0;
    g_max_loop_us = 0;
    profReset();
    replyToHost("OK FILTER OFF\n");
  } else if (line.startsWith("RECORD ON")) {
    g_loop_overruns = 0;
    g_max_loop_us   = 0;
    profReset();
    String path = line.substring(9);
    path.trim();
    sdRecordStart(path.length() ? path.c_str() : nullptr);
  } else if (line.startsWith("RECORD OFF")) {
    sdRecordStop();
  } else if (handleCfgLine(line)) {
    // handled
  } else if (line.startsWith("START")) {
    g_loop_overruns = 0;
    g_max_loop_us = 0;
    profReset();
    streaming = true;
    replyToHost("STARTED BIN:esp32s3_arduino\n");
    replyToHost("SENSORS:0,ICM20948\n");
#if !SERIAL_OUTPUT_BINARY
    Serial.println("START accepted (USB: bridge streams; Wi-Fi: TCP binary)");
#endif
  } else if (line.startsWith("STOP")) {
    streaming = false;
    replyToHost("STOPPED\n");
  } else if (line.equalsIgnoreCase("AP?") || line.equalsIgnoreCase("WIFI?") ||
             line.equalsIgnoreCase("STATUS")) {
    printAcqStatus();
    printWifiStatus();
  }
}

static void pollSerialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length()) handleLine(line);
}

static const char *wifiStatusString(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL (SSID not found / wrong name / 5 GHz only?)";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED (wrong password?)";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED: return "WL_DISCONNECTED (auth timeout / AP rejected / incompatible security?)";
    default: return "unknown";
  }
}

static void trimInPlace(char *s) {
  if (!s || !*s) return;
  char *start = s;
  while (*start == ' ' || *start == '\t') start++;
  if (start != s) memmove(s, start, strlen(start) + 1);
  size_t n = strlen(s);
  while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = '\0';
}

static volatile int lastStaDisconnectReason = -1;

static const char *wifiDisconnectReasonString(int reason) {
  switch (reason) {
    case 2: return "auth expire";
    case 15: return "4-way handshake timeout (wrong password?)";
    case 39: return "timeout";
    case 201: return "no AP found (SSID / 5 GHz only / hidden?)";
    case 202: return "auth fail (wrong password / WPA3-only AP?)";
    case 204: return "handshake timeout";
    case 205: return "group key update timeout";
    default: return "see esp_wifi_types.h WIFI_REASON_*";
  }
}

static void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    lastStaDisconnectReason = info.wifi_sta_disconnected.reason;
    Serial.printf("\n[WiFi] STA disconnected reason=%d (%s)\n",
                  lastStaDisconnectReason,
                  wifiDisconnectReasonString(lastStaDisconnectReason));
  }
}

static void printWifiFailureHelp(wl_status_t status) {
  Serial.printf("Wi-Fi status=%d (%s)\n", (int)status, wifiStatusString(status));
  if (lastStaDisconnectReason >= 0) {
    Serial.printf("Last disconnect reason=%d (%s)\n",
                  lastStaDisconnectReason,
                  wifiDisconnectReasonString(lastStaDisconnectReason));
  }
  Serial.println("STA tips: 2.4 GHz hotspot band; correct SSID/password; PC and ESP32 same network;");
  Serial.println("  iPhone: Settings -> Personal Hotspot -> Maximize Compatibility ON");
}

static void printWifiStatus() {
  if (!wifi_up) {
    Serial.println("[WiFi] not up (USB mode or init failed)");
    return;
  }
  if (wifi_soft_ap) {
    Serial.println("--- Soft AP status ---");
    Serial.printf("SSID=%s  pass=%s  channel=%d  broadcast=ON  WPA2-PSK\n",
                  WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL);
    Serial.printf("AP MAC=%s  IP=%s  TCP :%d\n",
                  WiFi.softAPmacAddress().c_str(),
                  WiFi.softAPIP().toString().c_str(), TCP_PORT);
    Serial.printf("Stations connected: %u / %d\n",
                  WiFi.softAPgetStationNum(), WIFI_AP_MAX_CONN);
  } else {
    Serial.println("--- STA status ---");
    Serial.printf("hostname=%s  STA MAC=%s\n",
                  WiFi.getHostname(), WiFi.macAddress().c_str());
    Serial.printf("IP=%s  gateway=%s  subnet=%s  RSSI=%d dBm\n",
                  WiFi.localIP().toString().c_str(),
                  WiFi.gatewayIP().toString().c_str(),
                  WiFi.subnetMask().toString().c_str(), WiFi.RSSI());
    Serial.printf("TCP listen :%d  client=%s  streaming=%s\n",
                  TCP_PORT,
                  (client && client.connected()) ? "yes" : "no",
                  streaming ? "yes" : "no");
    Serial.println("PC: ping IP above; Plugin/Ephys Socket -> IP:5000; send REDPITAYA then START");
  }
  Serial.println("Serial command: STATUS  (repeat)");
}

static bool startSoftApFallback() {
  WiFi.disconnect(true);
  WiFi.softAPdisconnect(true);
  delay(200);
  WiFi.mode(WIFI_OFF);
  delay(300);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_TX_POWER_AP);
  WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK);
  // Explicit AP IP — some Windows builds fail DHCP on softAP without this
  if (!WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                         IPAddress(255, 255, 255, 0))) {
    Serial.println("[AP] softAPConfig failed (continuing)");
  }
  // hidden=0 → SSID broadcast ON; password ≥8 → WPA2-PSK
  bool ok = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL, 0, WIFI_AP_MAX_CONN);
  if (!ok) {
    Serial.println("Soft AP start failed");
    return false;
  }
  delay(1500);  // let beacon stabilize before Windows scan
  wifi_up = true;
  wifi_soft_ap = true;
  Serial.println("WiFi OK Soft AP started");
  printWifiStatus();
  Serial.println("PC: join Wi-Fi STEP_ESP32 (password step1234), then host 192.168.4.1:5000");
  return true;
}

static void setupWifi() {
  if (!useWifi()) {
    Serial.println("Wi-Fi skipped — USB serial bench mode");
    Serial.println("PC: host\\run_usb_plugin_bridge.ps1 COMx  (Plugin) or serial_tcp_bridge.py COMx");
    Serial.println("Open Ephys: 127.0.0.1:5000 — not ESP32 Wi-Fi IP");
    return;
  }

  char ssid[33];
  char pass[64];
  strncpy(ssid, WIFI_SSID, sizeof(ssid) - 1);
  ssid[sizeof(ssid) - 1] = '\0';
  strncpy(pass, WIFI_PASS, sizeof(pass) - 1);
  pass[sizeof(pass) - 1] = '\0';
  trimInPlace(ssid);
  trimInPlace(pass);

  wifi_soft_ap = false;
  lastStaDisconnectReason = -1;
  WiFi.persistent(false);  // do not load stale NVS credentials / corrupt join state
  WiFi.onEvent(onWifiEvent);
  WiFi.disconnect(true);
  WiFi.softAPdisconnect(true);
  delay(200);
  WiFi.mode(WIFI_OFF);
  delay(200);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // iPhone hotspot: avoid ESP light-sleep during join
  WiFi.setTxPower(WIFI_TX_POWER_STA);
#if defined(WIFI_AUTH_WPA2_WPA3_PSK)
  WiFi.setMinSecurity(WIFI_AUTH_WPA2_WPA3_PSK);  // WPA2 + WPA3-only hotspots
#else
  WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK);
#endif
  WiFi.setHostname(WIFI_HOSTNAME);

  if (strcmp(ssid, "YOUR_HOTSPOT") == 0) {
    Serial.println("WARNING: WIFI_SSID still \"YOUR_HOTSPOT\" — edit sketch before upload");
  }

  Serial.println("Scanning 2.4 GHz networks (3 s, hidden SSIDs included)...");
  int n = WiFi.scanNetworks(false, true);  // async=false, show_hidden=true
  bool ssid_seen = false;
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == ssid) {
      ssid_seen = true;
      Serial.printf("  target \"%s\" seen RSSI=%d ch=%d\n",
                    ssid, WiFi.RSSI(i), WiFi.channel(i));
    }
  }
  if (n == 0) {
    Serial.println("  (no networks — RF/antenna/power issue?)");
  } else if (!ssid_seen) {
    Serial.printf("  \"%s\" NOT in scan — typo, 5 GHz-only, or out of range\n", ssid);
  }
  WiFi.scanDelete();

  if (strlen(pass) == 0) {
    Serial.printf("Connecting to open network \"%s\" len=%u (2.4 GHz)\n",
                  ssid, (unsigned)strlen(ssid));
    WiFi.begin(ssid);
  } else {
    Serial.printf("Connecting to \"%s\" len=%u (2.4 GHz)\n",
                  ssid, (unsigned)strlen(ssid));
    WiFi.begin(ssid, pass);
  }

  uint32_t t0 = millis();
  uint32_t lastStatusLog = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    uint32_t now = millis();
    wl_status_t st = WiFi.status();
    Serial.printf(". status=%d (%s)", (int)st, wifiStatusString(st));
    if (lastStaDisconnectReason >= 0) {
      Serial.printf(" disc_reason=%d (%s)",
                    lastStaDisconnectReason,
                    wifiDisconnectReasonString(lastStaDisconnectReason));
    }
    Serial.println();
    if (now - lastStatusLog >= 10000) {
      lastStatusLog = now;
      Serial.printf("  elapsed=%lu ms\n", (unsigned long)(now - t0));
    }
    if (now - t0 > (uint32_t)WIFI_STA_TIMEOUT_MS) {
      Serial.println();
      wl_status_t st = WiFi.status();
      printWifiFailureHelp(st);
      Serial.printf("STA failed (status=%d) — starting Soft AP %s\n", (int)st, WIFI_AP_SSID);
      startSoftApFallback();
      return;
    }
  }

  wifi_up = true;
  wifi_soft_ap = false;
  Serial.println();
  Serial.println("========================================");
  Serial.println("  WiFi STA CONNECTED — use this IP on PC");
  Serial.println("========================================");
  printWifiStatus();
  Serial.println("========================================");
}

static void setupEspNow() {
#if ENABLE_ESPNOW
  // When TCP is not in use, WiFi was not started by setupWifi().
  // ESP-NOW requires the WiFi stack to be initialized (STA mode, no AP join needed).
  if (!wifi_up) {
    WiFi.persistent(false);
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    delay(100);
    wifi_up = true;
    Serial.printf("ESP-NOW WiFi: STA mode ch=%d (no AP join)\n", ESPNOW_WIFI_CHANNEL);
  }
  esp_err_t err = esp_now_init();
  if (err != ESP_OK) {
    Serial.printf("ESP-NOW init failed: %d\n", (int)err);
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);
  esp_now_peer_info_t peer = {};
  memset(peer.peer_addr, 0xFF, 6);
  peer.channel = 0;
  peer.encrypt = false;
  err = esp_now_add_peer(&peer);
  if (err != ESP_OK) {
    Serial.printf("ESP-NOW add peer failed: %d\n", (int)err);
    return;
  }
  Serial.printf("ESP-NOW ready (role=%s ch=%d)\n",
                NODE_IS_MASTER ? "master" : "slave", ESPNOW_WIFI_CHANNEL);
#else
  Serial.println("ESP-NOW disabled — single-node mode");
#endif
}

static void maybeRepeatStatus() {
#if REPEAT_STATUS_SEC > 0
  if (millis() - last_status_ms < (uint32_t)REPEAT_STATUS_SEC * 1000UL) return;
  last_status_ms = millis();
  if (wifi_up) {
    printWifiStatus();
    return;
  }
  if (!icm_ok) {
    Serial.println("ICM20948: synthetic fallback - check 3V3, GND, SCK->D8, MISO->D9, MOSI->D10, CS->D7");
  }
  if (!mag_ok) {
    Serial.println("MAG unavailable: ch6-8=0, VQF 6-DOF only (no heading). See boot log for cause.");
  }
#endif
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  while (!Serial && millis() < 5000) {
    delay(10);
  }

  Serial.println();
  Serial.println("STEP node (Arduino) starting");

  initDio();

  printBootDiagnostics();
  icm_ok = initIcm20948();
  mag_ok = initAk09916();
  if (!mag_ok) {
    Serial.println("WARNING: AK09916 magnetometer not found.");
    Serial.println("  ch[6..8] will be 0; VQF quaternion is 6-DOF only (no yaw/heading).");
    Serial.println("  Causes: XIAO non-Sense variant (no AK09916); ICM SPI aux bus not reading");
    Serial.println("  (USER_CTRL); AK09916 needs power-cycle; board wiring missing SPI pins to ICM.");
  }

  setupWifi();
  setupEspNow();

#if ENABLE_SD
  g_sd_mutex = xSemaphoreCreateMutex();
  g_sd_ready = SD.begin(PIN_SD_CS, SPI, 25000000);
  Serial.println(g_sd_ready ? "SD ready" : "SD init failed");
  g_sd_queue = xQueueCreate(SD_QUEUE_DEPTH, sizeof(SdLogRecord));
  if (g_sd_queue) {
    xTaskCreatePinnedToCore(sdWriteTask, "sd_write", 4096, NULL, 4, NULL, 0);
  }
#endif

#if ENABLE_TCP && !ENABLE_SERIAL_BENCH
  if (wifi_up) {
    server.begin();
    Serial.printf("TCP listen :%d\n", TCP_PORT);
  }
#elif ENABLE_SERIAL_BENCH
  Serial.println("Serial bench active @115200");
  Serial.println(SERIAL_OUTPUT_BINARY
                     ? "Format: Open Ephys binary on Serial"
                     : "Format: CSV seq,ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz");
#endif

  boot_ms = millis();
  Serial.printf("CSV/stream paused %d ms — read diagnostics above\n", BOOT_CSV_DELAY_MS);
  last_status_ms = millis();
}

void loop() {
  pollSerialCommands();
  recMaybeFinalizeTimeout();

#if ENABLE_TCP && !ENABLE_SERIAL_BENCH
  if (wifi_up) {
    if (!client || !client.connected()) {
      recMarkControlDisconnected();
      WiFiClient incoming = server.available();
      if (incoming && incoming.connected()) {
        client = incoming;
        streaming = false;
        recMarkControlConnected();
        Serial.printf("Client connected from %s\n",
                      client.remoteIP().toString().c_str());
      }
    }
    while (client && client.available()) {
      String line = client.readStringUntil('\n');
      line.trim();
      if (line.length()) handleLine(line);
    }
  }
#endif

  maybeRepeatStatus();

  if (millis() - boot_ms < (uint32_t)BOOT_CSV_DELAY_MS) {
    return;
  }

  uint32_t now = micros();
  uint32_t loop_start_us = now;
  if (g_sample_hz < 1)
    return;
  const uint32_t period_us = 1000000UL / (uint32_t)g_sample_hz;
  if (g_sample_last_us != 0 && (uint32_t)(now - g_sample_last_us) < period_us)
    return;
  g_sample_last_us = now;

  int16_t imu[6];
  int16_t mag[3];
  bool mag_fresh = false;
  uint32_t prof_start_us = micros();
  readImu(imu);
  profAdd((uint32_t)(micros() - prof_start_us), &g_prof_imu_sum_us, &g_prof_imu_max_us);

  prof_start_us = micros();
  readMag(mag, &mag_fresh);
  profAdd((uint32_t)(micros() - prof_start_us), &g_prof_mag_sum_us, &g_prof_mag_max_us);

  updateDio();
  packChannelsFromImu(imu, mag_ok ? mag : nullptr, mag_fresh);

  sendEspNowSync();
  packAndSendTcp();
  prof_start_us = micros();
  sendSerialBench();
  profAdd((uint32_t)(micros() - prof_start_us), &g_prof_serial_sum_us,
          &g_prof_serial_max_us);
  logSd();

  g_generated_samples++;
  g_prof_samples++;
  uint32_t loop_us = (uint32_t)(micros() - loop_start_us);
  if (loop_us > g_max_loop_us) g_max_loop_us = loop_us;
  if (loop_us > period_us) g_loop_overruns++;

  seq++;
}

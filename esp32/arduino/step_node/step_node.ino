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
 * #define ICM20948_ADDR 0x69
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

#define ENABLE_ESPNOW false

#include <WiFi.h>
#include <WiFiClient.h>
#if ENABLE_ESPNOW
#include <esp_now.h>
#endif
#include <esp_timer.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>
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
#define WIFI_STA_TIMEOUT_MS 45000
// XIAO boards: high TX can desense the onboard antenna — try lower if STA/AP both fail
#define WIFI_TX_POWER_STA WIFI_POWER_8_5dBm
#define WIFI_TX_POWER_AP WIFI_POWER_8_5dBm

#define TCP_PORT 5000
#define SAMPLE_HZ_DEFAULT 100
#define NUM_CHANNELS 11

#define ICM_BANK2_ACCEL_CONFIG_1 0x14
#define ICM_BANK2_GYRO_CONFIG_1 0x01

#define PIN_I2C_SDA 5   // XIAO D4 / GPIO5
#define PIN_I2C_SCL 6   // XIAO D5 / GPIO6
#define PIN_DIO 1       // XIAO D0 / GPIO1 — change via #define if wired elsewhere
#define ICM20948_ADDR 0x69

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
#define ICM_ACCEL_XOUT_H 0x2D
#define ICM20948_WHOAMI_VAL 0xEA

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

WiFiServer server(TCP_PORT);
WiFiClient client;
bool streaming = false;
bool wifi_up = false;
bool wifi_soft_ap = false;

uint32_t seq = 0;
int16_t channels[NUM_CHANNELS];
bool icm_ok = false;
uint8_t icm_addr = ICM20948_ADDR;
uint32_t boot_ms = 0;
bool csv_banner_sent = false;
uint32_t last_status_ms = 0;

// DIO ch6: bit0 = level (1 idle/high, 0 pressed to GND); bits1-15 = debounced edge count
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
static char g_sd_path[48] = "/step_session.bin";

static const float kAccLsbPerG[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
static const float kGyrLsbPerDps[4] = {131.072f, 65.536f, 32.768f, 16.384f};
static const float kStdGravityMps2 = 9.80665f;
static const float kMagUnitsPerLsb = 0.15f;  // AK09916, matches Plugin sensor_fusion ICM20948
static const float kMagRateHz = 100.0f;

static VQF g_vqf;
static bool g_vqf_inited = false;

static bool useWifi() { return ENABLE_TCP || ENABLE_ESPNOW; }

static int16_t floatToQ15(float v) {
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  return (int16_t)(v * 32767.0f);
}

static void icmApplyRangePresets() {
  if (!icm_ok) return;
  const uint8_t acc_fs = g_acc_preset & 3u;
  const uint8_t gyr_fs = g_gyr_preset & 3u;
  icmSelectBank(icm_addr, 2);
  icmWriteAddr(icm_addr, ICM_BANK2_ACCEL_CONFIG_1, (uint8_t)(acc_fs << 1));
  icmWriteAddr(icm_addr, ICM_BANK2_GYRO_CONFIG_1, (uint8_t)(gyr_fs << 1));
  icmSelectBank(icm_addr, 0);
#if !SERIAL_OUTPUT_BINARY
  Serial.printf("ICM range: ACC preset %u  GYR preset %u\n", acc_fs, gyr_fs);
#endif
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
  vqf_update(&g_vqf, gyr, acc);

  if (mag_or_null != nullptr && mag_fresh) {
    vqf_real_t mag[3];
    mag[0] = (vqf_real_t)((float)mag_or_null[0] * kMagUnitsPerLsb);
    mag[1] = (vqf_real_t)((float)mag_or_null[1] * kMagUnitsPerLsb);
    mag[2] = (vqf_real_t)((float)mag_or_null[2] * kMagUnitsPerLsb);
    vqf_update_mag(&g_vqf, mag);
  }
}

static void vqfReadQuatQ15(int16_t out[4], bool use_9d) {
  vqf_real_t quat[4];
  if (use_9d)
    vqf_get_quat_9d(&g_vqf, quat);
  else
    vqf_get_quat_6d(&g_vqf, quat);
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
  channels[6] = packDioCh6();
  channels[7] = 0;
  channels[8] = 0;
  channels[9] = 0;
  channels[10] = 0;

  if (!g_filter_on)
    return;

  vqfUpdateFromImu(imu, mag_or_null, mag_fresh);
  vqfReadQuatQ15(&channels[7], mag_or_null != nullptr && mag_fresh);
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
  (void)info; (void)data; (void)len;
}
void onEspNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info; (void)status;
}
#endif

static bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool icmWriteAddr(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool icmReadAddr(uint8_t addr, uint8_t reg, uint8_t *val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  *val = Wire.read();
  return true;
}

static void icmSelectBank(uint8_t addr, uint8_t bank) {
  icmWriteAddr(addr, ICM_REG_BANK_SEL, bank & 0x30);
}

static bool icmReadWhoAmI(uint8_t addr, uint8_t *who) {
  icmSelectBank(addr, 0);
  return icmReadAddr(addr, ICM_WHO_AM_I, who);
}

static bool icmWrite(uint8_t reg, uint8_t val) {
  return icmWriteAddr(icm_addr, reg, val);
}

static bool icmReadReg(uint8_t reg, uint8_t *val) {
  return icmReadAddr(icm_addr, reg, val);
}

static void printBootDiagnostics() {
#if BOOT_DIAGNOSTICS
  Serial.println();
  Serial.println("========================================");
  Serial.println("  STEP ESP32-S3 NODE — BOOT DIAGNOSTICS");
  Serial.println("========================================");
  Serial.printf("Firmware: %s\n", FIRMWARE_VERSION);
  Serial.printf("Board target: XIAO_ESP32S3 (Sense)\n");
  Serial.printf("I2C SDA: GPIO%d (pad D4)  SCL: GPIO%d (pad D5)\n", PIN_I2C_SDA, PIN_I2C_SCL);
  Serial.printf("ICM20948 config addr: 0x%02X (AD0 high=0x69, low=0x68)\n", ICM20948_ADDR);
  Serial.printf("Sample rate: %d Hz  channels: %d\n", g_sample_hz, NUM_CHANNELS);
  Serial.println("--- I2C scan 0x68-0x6B ---");
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  delay(50);
  int found = 0;
  for (uint8_t a = 0x68; a <= 0x6B; a++) {
    if (i2cProbe(a)) {
      Serial.printf("  device at 0x%02X\n", a);
      found++;
    }
  }
  if (found == 0) Serial.println("  (no devices — check VCC/GND/SDA/SCL on D4/D5)");
  Serial.println("--- ICM20948 WHO_AM_I (expect 0xEA) ---");
  for (uint8_t a : {0x68, 0x69}) {
    uint8_t who = 0;
    if (icmReadWhoAmI(a, &who)) {
      Serial.printf("  0x%02X -> WHO_AM_I 0x%02X %s\n", a, who,
                    who == ICM20948_WHOAMI_VAL ? "OK" : "unexpected");
    } else {
      Serial.printf("  0x%02X -> no ACK\n", a);
    }
  }
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
  Serial.println("DIO ch6: bit0=level, bits1-15=edge_count (Open Ephys int16)");
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

static bool initIcm20948() {
  const uint8_t candidates[] = {ICM20948_ADDR, 0x68, 0x69};
  for (uint8_t a : candidates) {
    uint8_t who = 0;
    if (!icmReadWhoAmI(a, &who)) continue;
    if (who != ICM20948_WHOAMI_VAL) {
      Serial.printf("ICM20948 at 0x%02X WHO_AM_I=0x%02X (expected 0xEA)\n", a, who);
      continue;
    }
    icm_addr = a;
    icmSelectBank(icm_addr, 0);
    icmWriteAddr(icm_addr, ICM_PWR_MGMT_1, 0x01);
    delay(100);
    Serial.printf("ICM20948: OK at I2C 0x%02X WHO_AM_I=0xEA\n", icm_addr);
    icmApplyRangePresets();
    return true;
  }
  Serial.println("ICM20948: synthetic fallback — no chip at 0x68/0x69 with WHO_AM_I 0xEA");
  return false;
}

static bool readImuRaw(int16_t out[6]) {
  icmSelectBank(icm_addr, 0);
  Wire.beginTransmission(icm_addr);
  Wire.write(ICM_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(icm_addr, (uint8_t)14) != 14) return false;

  auto read16be = []() {
    int16_t v = (int16_t)(Wire.read() << 8);
    v |= Wire.read();
    return v;
  };
  out[0] = read16be();
  out[1] = read16be();
  out[2] = read16be();
  (void)read16be();
  out[3] = read16be();
  out[4] = read16be();
  out[5] = read16be();
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

// Open Ephys header offset field (int32 LE): low 32 bits of esp_timer_get_time() µs since boot.
// offset==0 = legacy frames (host/Plugin use arrival-time pacing). Same clock on every slave;
// cross-board alignment needs START pulse or host merge — see docs/open-ephys-plugin.md.
static void fillOeHeader(OeHeader *hdr) {
  const uint32_t hw_us = (uint32_t)esp_timer_get_time();
  hdr->offset = (int32_t)hw_us;
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
  if (!csv_banner_sent) {
    Serial.printf("# STEP boot complete icm=%s addr=0x%02X dio_ch6=level|edges\n",
                  icm_ok ? "OK" : "FALLBACK", icm_addr);
    csv_banner_sent = true;
  }
#if SERIAL_OUTPUT_BINARY
  OeHeader hdr;
  fillOeHeader(&hdr);
  Serial.write((uint8_t *)&hdr, sizeof(hdr));
  Serial.write((uint8_t *)channels, sizeof(channels));
#else
  Serial.printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d\n",
                (unsigned long)seq, channels[0], channels[1], channels[2],
                channels[3], channels[4], channels[5], channels[6], channels[7]);
#endif
#endif
}

static void logSd() {
#if ENABLE_SD
  if (!g_sd_recording || !g_sd_file) return;

  SdLogRecord rec = {};
  rec.seq = seq;
  rec.time_us = (int64_t)esp_timer_get_time();
  memcpy(rec.ch, channels, sizeof(channels));

  uint32_t t0 = micros();
  size_t written = g_sd_file.write((uint8_t *)&rec, sizeof(rec));
  uint32_t write_us = (uint32_t)(micros() - t0);
  if (write_us > g_max_sd_write_us) g_max_sd_write_us = write_us;

  if (written == sizeof(rec)) {
    g_sd_saved_samples++;
  } else {
    g_sd_write_errors++;
  }
#endif
}

static bool sdEnsureReady() {
#if ENABLE_SD
  if (g_sd_ready) return true;
  g_sd_ready = SD.begin(PIN_SD_CS);
  return g_sd_ready;
#else
  return false;
#endif
}

static bool sdRecordStart(const char *path_or_null) {
#if ENABLE_SD
  if (!sdEnsureReady()) {
    g_sd_write_errors++;
    Serial.println("SD_STATUS enabled=1 ready=0 recording=0 error=begin_failed");
    return false;
  }

  if (g_sd_recording && g_sd_file) {
    g_sd_file.flush();
    g_sd_file.close();
  }

  if (path_or_null && path_or_null[0]) {
    strncpy(g_sd_path, path_or_null, sizeof(g_sd_path) - 1);
    g_sd_path[sizeof(g_sd_path) - 1] = '\0';
  } else {
    strncpy(g_sd_path, "/step_session.bin", sizeof(g_sd_path) - 1);
    g_sd_path[sizeof(g_sd_path) - 1] = '\0';
  }

  g_sd_file = SD.open(g_sd_path, FILE_WRITE);
  if (!g_sd_file) {
    g_sd_recording = false;
    g_sd_write_errors++;
    Serial.printf("SD_STATUS enabled=1 ready=1 recording=0 error=open_failed path=%s\n", g_sd_path);
    return false;
  }

  g_sd_saved_samples = 0;
  g_sd_write_errors = 0;
  g_max_sd_write_us = 0;

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
  Serial.printf("SD_STATUS enabled=1 ready=1 recording=1 path=%s sample_hz=%u\n",
                g_sd_path, (unsigned)g_sample_hz);
  return true;
#else
  Serial.println("SD_STATUS enabled=0 ready=0 recording=0 error=compile_disabled");
  return false;
#endif
}

static void sdRecordStop() {
#if ENABLE_SD
  if (g_sd_file) {
    uint32_t t0 = micros();
    g_sd_file.flush();
    uint32_t flush_us = (uint32_t)(micros() - t0);
    if (flush_us > g_max_sd_write_us) g_max_sd_write_us = flush_us;
    g_sd_file.close();
  }
  g_sd_recording = false;
  Serial.printf("SD_STATUS enabled=1 ready=%d recording=0 path=%s saved=%llu errors=%llu max_sd_write_us=%lu\n",
                g_sd_ready ? 1 : 0,
                g_sd_path,
                (unsigned long long)g_sd_saved_samples,
                (unsigned long long)g_sd_write_errors,
                (unsigned long)g_max_sd_write_us);
#else
  Serial.println("SD_STATUS enabled=0 ready=0 recording=0 error=compile_disabled");
#endif
}

static void printAcqStatus() {
  Serial.printf("STATUS seq=%lu generated=%llu sample_hz=%u filter=%d streaming=%d "
                "sd_enabled=%d sd_ready=%d sd_recording=%d sd_saved=%llu sd_errors=%llu "
                "max_sd_write_us=%lu max_loop_us=%lu loop_overruns=%lu\n",
                (unsigned long)seq,
                (unsigned long long)g_generated_samples,
                (unsigned)g_sample_hz,
                g_filter_on ? 1 : 0,
                streaming ? 1 : 0,
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
                (unsigned long)g_loop_overruns);
}

static void replyToHost(const char *text) {
#if ENABLE_TCP && !ENABLE_SERIAL_BENCH
  if (client && client.connected())
    client.print(text);
#else
  (void)text;  // Plugin USB path: bridge answers REDPITAYA/START on TCP; optional log only
#endif
}

static void handleLine(const String &line) {
  if (line.startsWith("REDPITAYA")) {
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
    replyToHost("OK FILTER ON\n");
  } else if (line.startsWith("FILTER OFF")) {
    g_filter_on = false;
    replyToHost("OK FILTER OFF\n");
  } else if (line.startsWith("RECORD ON")) {
    String path = line.substring(9);
    path.trim();
    sdRecordStart(path.length() ? path.c_str() : nullptr);
  } else if (line.startsWith("RECORD OFF")) {
    sdRecordStop();
  } else if (handleCfgLine(line)) {
    // handled
  } else if (line.startsWith("START")) {
    streaming = true;
    replyToHost("STARTED BIN:esp32s3_arduino\n");
    replyToHost("SENSORS:0,ICM20948\n");
#if !SERIAL_OUTPUT_BINARY
    Serial.println("START accepted (USB: bridge streams; Wi-Fi: TCP binary)");
#endif
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
  if (!wifi_up) {
    Serial.println("ESP-NOW skipped (Wi-Fi not connected)");
    return;
  }
  esp_now_init();
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);
  esp_now_peer_info_t peer = {};
  memset(&peer.peer_addr, 0xFF, 6);
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  Serial.println("ESP-NOW enabled (multi-node)");
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
    Serial.println("ICM20948: synthetic fallback — check 3V3, GND, SDA->D4, SCL->D5, addr 0x68/0x69");
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

  setupWifi();
  setupEspNow();

#if ENABLE_SD
  Serial.println(SD.begin(PIN_SD_CS) ? "SD ready" : "SD init failed");
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
                     : "Format: CSV seq,ax,ay,az,gx,gy,gz,dio,cam");
#endif

  boot_ms = millis();
  Serial.printf("CSV/stream paused %d ms — read diagnostics above\n", BOOT_CSV_DELAY_MS);
  last_status_ms = millis();
}

void loop() {
  pollSerialCommands();

#if ENABLE_TCP && !ENABLE_SERIAL_BENCH
  if (wifi_up) {
    if (!client || !client.connected()) {
      WiFiClient incoming = server.available();
      if (incoming && incoming.connected()) {
        client = incoming;
        streaming = false;
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
  readImu(imu);
  updateDio();
  packChannelsFromImu(imu, nullptr, false);

  sendEspNowSync();
  packAndSendTcp();
  sendSerialBench();
  logSd();

  g_generated_samples++;
  uint32_t loop_us = (uint32_t)(micros() - loop_start_us);
  if (loop_us > g_max_loop_us) g_max_loop_us = loop_us;
  if (loop_us > period_us) g_loop_overruns++;

  seq++;
}

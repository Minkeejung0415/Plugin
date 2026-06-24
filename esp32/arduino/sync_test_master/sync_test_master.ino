/*
 * sync_test_master.ino
 * Auto-test for ESP-NOW clock sync — MASTER role.
 *
 * Behaviour:
 *   1. Boots, initialises ICM20948 + AK09916 + ESP-NOW.
 *   2. Broadcasts a SyncPacket every sample (same as step_node master).
 *      The SyncPacket includes `start_at_us`: the master-domain time when
 *      acquisition will begin.  Both boards start at the same instant in
 *      master time.
 *   3. After AUTO_START_MS ms, starts printing CSV at SAMPLE_HZ.
 *   4. Prints SAMPLE_COUNT rows then stops.
 *
 * CSV columns: seq,master_us,ax,ay,az,gx,gy,gz,mx,my,mz
 *
 * Flash to the MASTER board.  Flash sync_test_slave.ino to the slave.
 * Open two Serial Monitors (115200).  Find the same tap event in both
 * streams — master_us and slave_corrected_us should match within ~1 ms.
 */

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <esp_timer.h>
#include <SPI.h>

// ── Config ────────────────────────────────────────────────────────────────
#define ESPNOW_CHANNEL   6
#define SAMPLE_HZ       100
#define AUTO_START_MS  3000   // ms from boot before streaming starts
#define SAMPLE_COUNT   3000   // rows to print, then stop

// ── Pins (same as step_node) ──────────────────────────────────────────────
#define PIN_SPI_SCK  D6
#define PIN_SPI_MISO D4
#define PIN_SPI_MOSI D5
#define PIN_ICM_CS   D3

// ── ICM20948 registers ────────────────────────────────────────────────────
#define ICM_REG_BANK_SEL    0x7F
#define ICM_WHO_AM_I        0x00
#define ICM_PWR_MGMT_1      0x06
#define ICM_USER_CTRL       0x03
#define ICM_ACCEL_XOUT_H    0x2D
#define ICM_EXT_SENS_DATA_00 0x3B
#define ICM_I2C_MST_CTRL    0x01
#define ICM_I2C_SLV0_ADDR   0x03
#define ICM_I2C_SLV0_REG    0x04
#define ICM_I2C_SLV0_CTRL   0x05
#define ICM_I2C_SLV0_DO     0x06
#define ICM_BANK2_GYRO_SMPLRT_DIV   0x00
#define ICM_BANK2_GYRO_CONFIG_1     0x01
#define ICM_BANK2_ODR_ALIGN_EN      0x09
#define ICM_BANK2_ACCEL_SMPLRT_DIV_1 0x10
#define ICM_BANK2_ACCEL_SMPLRT_DIV_2 0x11
#define ICM_BANK2_ACCEL_CONFIG_1    0x14
#define ICM20948_WHOAMI_VAL 0xEA

// ── AK09916 registers ─────────────────────────────────────────────────────
#define AK09916_ADDR         0x0C
#define AK09916_WIA2         0x01
#define AK09916_WIA2_VAL     0x09
#define AK09916_ST1          0x10
#define AK09916_CNTL2        0x31
#define AK09916_CNTL3        0x32
#define AK09916_MODE_100HZ   0x08

// ── ESP-NOW packets ───────────────────────────────────────────────────────
#pragma pack(push,1)
typedef struct {
    uint32_t seq;
    int64_t  time_us;
    int64_t  start_at_us;   // master-domain time to begin streaming (fixed after boot)
} SyncPacket;
typedef struct { uint8_t magic; } SlaveAckPacket;
#define SLAVE_ACK_MAGIC 0xAC
#pragma pack(pop)

// ── Globals ───────────────────────────────────────────────────────────────
static SPIClass ICM_SPI(HSPI);
static bool     g_icm_ok = false;
static bool     g_mag_ok = false;
static int16_t  g_last_mag[3] = {0,0,0};
static bool     g_have_mag = false;
static uint32_t g_last_mag_us = 0;
static uint32_t g_seq = 0;
static int64_t  g_start_at_us = 0;   // set once in setup()
static volatile bool g_slave_found = false;

// ── SPI helpers ───────────────────────────────────────────────────────────
static void icmWrite(uint8_t reg, uint8_t val) {
    ICM_SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_ICM_CS, LOW);
    ICM_SPI.transfer(reg & 0x7F);
    ICM_SPI.transfer(val);
    digitalWrite(PIN_ICM_CS, HIGH);
    ICM_SPI.endTransaction();
}
static uint8_t icmRead(uint8_t reg) {
    ICM_SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_ICM_CS, LOW);
    ICM_SPI.transfer(reg | 0x80);
    uint8_t v = ICM_SPI.transfer(0x00);
    digitalWrite(PIN_ICM_CS, HIGH);
    ICM_SPI.endTransaction();
    return v;
}
static void icmReadBytes(uint8_t reg, uint8_t *buf, size_t n) {
    ICM_SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_ICM_CS, LOW);
    ICM_SPI.transfer(reg | 0x80);
    for (size_t i = 0; i < n; i++) buf[i] = ICM_SPI.transfer(0x00);
    digitalWrite(PIN_ICM_CS, HIGH);
    ICM_SPI.endTransaction();
}
static void icmBank(uint8_t bank) { icmWrite(ICM_REG_BANK_SEL, (uint8_t)((bank & 3) << 4)); }

// ── Mag aux-bus helpers ───────────────────────────────────────────────────
static void auxWrite(uint8_t slave, uint8_t reg, uint8_t val) {
    icmBank(3);
    icmWrite(ICM_I2C_SLV0_ADDR, slave);
    icmWrite(ICM_I2C_SLV0_REG,  reg);
    icmWrite(ICM_I2C_SLV0_DO,   val);
    icmWrite(ICM_I2C_SLV0_CTRL, 0x81);
    delay(10);
    icmWrite(ICM_I2C_SLV0_CTRL, 0x00);
    icmBank(0);
}
static uint8_t auxRead(uint8_t slave, uint8_t reg) {
    icmBank(3);
    icmWrite(ICM_I2C_SLV0_ADDR, (uint8_t)(0x80 | slave));
    icmWrite(ICM_I2C_SLV0_REG,  reg);
    icmWrite(ICM_I2C_SLV0_CTRL, 0x81);
    delay(10);
    icmBank(0);
    uint8_t v = icmRead(ICM_EXT_SENS_DATA_00);
    icmBank(3);
    icmWrite(ICM_I2C_SLV0_CTRL, 0x00);
    icmBank(0);
    return v;
}

// ── ICM / Mag init ────────────────────────────────────────────────────────
static bool initIcm() {
    pinMode(PIN_ICM_CS, OUTPUT);
    digitalWrite(PIN_ICM_CS, HIGH);
    ICM_SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_ICM_CS);
    delay(50);
    icmBank(0);
    uint8_t who = icmRead(ICM_WHO_AM_I);
    if (who != ICM20948_WHOAMI_VAL) {
        Serial.printf("ICM WHO_AM_I=0x%02X (expected 0xEA) — FAIL\n", who);
        return false;
    }
    icmWrite(ICM_PWR_MGMT_1, 0x01);
    delay(100);
    // ODR: gyro_div=0, accel_div=0, odr_align=1
    icmBank(2);
    icmWrite(ICM_BANK2_GYRO_SMPLRT_DIV,    0x00);
    icmWrite(ICM_BANK2_ACCEL_SMPLRT_DIV_1, 0x00);
    icmWrite(ICM_BANK2_ACCEL_SMPLRT_DIV_2, 0x00);
    icmWrite(ICM_BANK2_ODR_ALIGN_EN,        0x01);
    icmWrite(ICM_BANK2_ACCEL_CONFIG_1,      0x01);  // ±4g
    icmWrite(ICM_BANK2_GYRO_CONFIG_1,       0x01);  // ±500 dps
    icmBank(0);
    Serial.printf("ICM20948: OK WHO_AM_I=0xEA\n");
    return true;
}

static bool initMag() {
    if (!g_icm_ok) return false;
    icmBank(0);
    icmWrite(ICM_USER_CTRL, 0x20);
    delay(10);
    icmBank(3);
    icmWrite(ICM_I2C_MST_CTRL, 0x07);
    delay(10);
    icmBank(0);
    uint8_t who = auxRead(AK09916_ADDR, AK09916_WIA2);
    if (who != AK09916_WIA2_VAL) {
        Serial.printf("AK09916: WIA2=0x%02X — FAIL\n", who);
        return false;
    }
    auxWrite(AK09916_ADDR, AK09916_CNTL3, 0x01);
    delay(10);
    auxWrite(AK09916_ADDR, AK09916_CNTL2, AK09916_MODE_100HZ);
    icmBank(3);
    icmWrite(ICM_I2C_SLV0_ADDR, (uint8_t)(0x80 | AK09916_ADDR));
    icmWrite(ICM_I2C_SLV0_REG,  AK09916_ST1);
    icmWrite(ICM_I2C_SLV0_CTRL, 0x89);  // read 9 bytes (ST1 + XYZ + TMPS + ST2)
    delay(10);
    icmBank(0);
    Serial.println("AK09916: OK, continuous 100 Hz");
    return true;
}

// ── Read helpers ──────────────────────────────────────────────────────────
static bool readImu(int16_t out[6]) {
    uint8_t raw[12];
    icmBank(0);
    icmReadBytes(ICM_ACCEL_XOUT_H, raw, 12);
    auto r16 = [](const uint8_t *p){ return (int16_t)(((uint16_t)p[0]<<8)|p[1]); };
    out[0]=r16(&raw[0]); out[1]=r16(&raw[2]); out[2]=r16(&raw[4]);
    out[3]=r16(&raw[6]); out[4]=r16(&raw[8]); out[5]=r16(&raw[10]);
    return true;
}

static bool readMag(int16_t out[3]) {
    uint8_t raw[9];
    icmBank(0);
    icmReadBytes(ICM_EXT_SENS_DATA_00, raw, 9);
    if ((raw[0] & 0x01) == 0 || (raw[8] & 0x08) != 0) {
        // not ready or overflow — return cached
        out[0]=g_last_mag[0]; out[1]=g_last_mag[1]; out[2]=g_last_mag[2];
        return g_have_mag;
    }
    g_last_mag[0] = out[0] = (int16_t)((uint16_t)raw[1] | ((uint16_t)raw[2]<<8));
    g_last_mag[1] = out[1] = (int16_t)((uint16_t)raw[3] | ((uint16_t)raw[4]<<8));
    g_last_mag[2] = out[2] = (int16_t)((uint16_t)raw[5] | ((uint16_t)raw[6]<<8));
    g_have_mag = true;
    return true;
}

// ── ESP-NOW ───────────────────────────────────────────────────────────────
void onRecvAck(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    (void)info;
    if (len < 1 || data[0] != SLAVE_ACK_MAGIC) return;
    g_slave_found = true;
}

static void espNowSendSync() {
    SyncPacket pkt;
    pkt.seq         = g_seq;
    pkt.time_us     = (int64_t)esp_timer_get_time();
    pkt.start_at_us = g_start_at_us;
    uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    esp_now_send(bcast, (uint8_t*)&pkt, sizeof(pkt));
}

static bool setupEspNow() {
    WiFi.persistent(false);
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    delay(100);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init FAIL");
        return false;
    }
    // Register broadcast peer
    esp_now_peer_info_t peer = {};
    memset(peer.peer_addr, 0xFF, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    esp_now_register_recv_cb(onRecvAck);
    Serial.printf("ESP-NOW ready (role=master ch=%d)\n", ESPNOW_CHANNEL);
    return true;
}

// ── Arduino entry points ──────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== SYNC_TEST_MASTER ===");

    g_icm_ok = initIcm();
    g_mag_ok = initMag();
    setupEspNow();

    // Broadcast sync packets and wait for at least one slave to ack
    Serial.println("# Waiting for slave ack (timeout 30 s)...");
    uint32_t wait_start = millis();
    while (!g_slave_found) {
        espNowSendSync();
        g_seq++;
        delay(50);  // 20 Hz while waiting
        if ((millis() - wait_start) > 30000) {
            Serial.println("# WARNING: no slave ack — starting anyway");
            break;
        }
    }
    if (g_slave_found) Serial.println("# Slave found! Starting in 3 s...");

    // Fix start_at_us NOW (after slave confirmed) so slave clock is already synced
    g_start_at_us = (int64_t)esp_timer_get_time() + (int64_t)AUTO_START_MS * 1000LL;
    Serial.printf("# start_at_us=%lld  (%.3f s from now)\n",
                  (long long)g_start_at_us, AUTO_START_MS / 1000.0f);
    Serial.println("# seq,master_us,ax,ay,az,gx,gy,gz,mx,my,mz");
}

void loop() {
    static uint32_t samples_printed = 0;
    static bool done = false;

    if (done) {
        delay(1000);
        return;
    }

    // 100 Hz pacing
    static uint32_t last_us = 0;
    uint32_t now_us = (uint32_t)micros();
    if (last_us != 0 && (uint32_t)(now_us - last_us) < (1000000UL / SAMPLE_HZ)) return;
    last_us = now_us;

    int16_t imu[6] = {}, mag[3] = {};
    readImu(imu);
    if (g_mag_ok) readMag(mag);

    int64_t t_us = (int64_t)esp_timer_get_time();

    // Always broadcast sync so slave can track us
    espNowSendSync();
    g_seq++;

    // Only print CSV after start time
    if (t_us < g_start_at_us) return;

    Serial.printf("%lu,%lld,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                  (unsigned long)samples_printed,
                  (long long)t_us,
                  imu[0],imu[1],imu[2],
                  imu[3],imu[4],imu[5],
                  mag[0],mag[1],mag[2]);

    samples_printed++;
    if (samples_printed >= SAMPLE_COUNT) {
        done = true;
        Serial.printf("# SYNC_TEST_MASTER DONE samples=%lu\n", (unsigned long)SAMPLE_COUNT);
    }
}

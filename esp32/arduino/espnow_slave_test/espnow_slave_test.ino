// espnow_slave_test.ino
// Minimal ESP-NOW sync receiver for testing clock sync without any sensor.
// Flash this to a bare ESP32. Open Serial Monitor at 115200.
// The master (step_node with NODE_IS_MASTER true) must be running nearby.

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <esp_timer.h>

#define ESPNOW_WIFI_CHANNEL 6   // Must match master

// Same packet layout as step_node.ino — do not change
typedef struct {
  uint32_t seq;
  int64_t  time_us;
} SyncPacket;

static volatile int64_t  g_offset_us   = 0;
static volatile uint32_t g_last_seq    = 0;
static volatile uint32_t g_recv_count  = 0;
static volatile bool     g_synced      = false;

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < (int)sizeof(SyncPacket)) return;
  const SyncPacket *pkt = (const SyncPacket *)data;
  int64_t now_us = esp_timer_get_time();
  g_offset_us  = (int64_t)pkt->time_us - now_us;
  g_last_seq   = pkt->seq;
  g_recv_count++;
  g_synced     = true;
  Serial.printf("[ESPNOW] seq=%-6lu  offset_us=%-10lld  master_us=%llu\n",
                (unsigned long)pkt->seq,
                (long long)g_offset_us,
                (unsigned long long)pkt->time_us);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESPNOW SLAVE TEST ===");
  Serial.printf("Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(100);

  Serial.printf("WiFi STA up, ch=%d, MAC=%s\n",
                ESPNOW_WIFI_CHANNEL, WiFi.macAddress().c_str());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: esp_now_init failed");
    return;
  }
  esp_now_register_recv_cb(onRecv);
  Serial.println("Waiting for master broadcasts...");
}

void loop() {
  // Print a heartbeat every 5 s so you know it's alive
  static uint32_t last_hb = 0;
  uint32_t now = millis();
  if (now - last_hb >= 5000) {
    last_hb = now;
    if (g_synced) {
      Serial.printf("[STATUS] synced=YES  total_recv=%lu  last_seq=%lu  offset_us=%lld\n",
                    (unsigned long)g_recv_count,
                    (unsigned long)g_last_seq,
                    (long long)g_offset_us);
    } else {
      Serial.println("[STATUS] synced=NO — waiting for master");
    }
  }
}

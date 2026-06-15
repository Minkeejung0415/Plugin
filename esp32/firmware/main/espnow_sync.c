#include "espnow_sync.h"

#include <string.h>

#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "espnow_sync";

typedef struct __attribute__((packed)) {
    uint32_t seq;
    int64_t time_us;
} sync_packet_t;

static sync_state_t s_state;
static bool s_master;

static void espnow_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    (void)info;
    if (len < (int)sizeof(sync_packet_t)) {
        return;
    }
    const sync_packet_t *p = (const sync_packet_t *)data;
    int64_t recv_us = esp_timer_get_time();
    s_state.master_seq     = p->seq;
    s_state.master_time_us = p->time_us;
    if (!s_master) {
        s_state.clock_offset_us = (int32_t)(p->time_us - recv_us);
    }
}

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "ESP-NOW send status %d", (int)status);
    }
}

void espnow_sync_init(bool is_master)
{
    s_master = is_master;
    memset(&s_state, 0, sizeof(s_state));

    esp_now_register_recv_cb(espnow_recv);
    esp_now_register_send_cb(espnow_send_cb);
    esp_now_init();

    uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, broadcast, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    ESP_LOGI(TAG, "ESP-NOW init role=%s", is_master ? "master" : "slave");
}

void espnow_sync_get_state(sync_state_t *out)
{
    *out = s_state;
}

void espnow_sync_on_local_frame(uint32_t seq, int64_t time_us)
{
    if (!s_master) {
        return;
    }
    sync_packet_t pkt = {.seq = seq, .time_us = time_us};
    uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_send(broadcast, (uint8_t *)&pkt, sizeof(pkt));
    s_state.master_seq = seq;
    s_state.master_time_us = time_us;
}

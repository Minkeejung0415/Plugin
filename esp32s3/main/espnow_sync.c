/* ESPNow-based multi-node synchronisation.
   Master broadcasts a timestamp every ESPNOW_SYNC_INTERVAL_MS ms.
   Slaves echo it back; master computes per-slave clock offset.
   All data flows slave→master; master aggregates and forwards over WiFi. */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "include/config.h"
#include "include/espnow_sync.h"

static const char *TAG = "espnow";

static uint8_t          s_node_id;
static bool             s_is_master;
static int64_t          s_time_offset_us = 0;
static espnow_data_cb_t s_data_cb        = NULL;
static QueueHandle_t    s_rx_queue;

/* Per-peer clock offset table (master only) */
static struct {
    uint8_t  id;
    uint8_t  mac[6];
    int64_t  offset_us;
    bool     valid;
} s_peers[ESPNOW_MAX_NODES];
static int s_peer_count = 0;

/* ── Receive callback (runs in WiFi task context) ─────────────────────── */
static void espnow_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len)
{
    if (len < 1) return;
    uint8_t *copy = malloc(len);
    if (!copy) return;
    memcpy(copy, data, len);
    /* Pass {len, data} pair as uintptr tuple via queue */
    uintptr_t pair[2] = {(uintptr_t)len, (uintptr_t)copy};
    if (xQueueSendFromISR(s_rx_queue, pair, NULL) != pdTRUE)
        free(copy);
}

/* ── Rx processing task ───────────────────────────────────────────────── */
static void espnow_rx_task(void *arg)
{
    uintptr_t pair[2];
    while (1) {
        if (xQueueReceive(s_rx_queue, pair, portMAX_DELAY) != pdTRUE)
            continue;
        int len = (int)pair[0];
        uint8_t *buf = (uint8_t *)pair[1];

        uint8_t type = buf[0];

        if (type == ESPNOW_PKT_SYNC && !s_is_master) {
            /* Slave: update clock offset and echo back */
            espnow_sync_pkt_t *pkt = (espnow_sync_pkt_t *)buf;
            int64_t now = esp_timer_get_time();
            s_time_offset_us = pkt->master_us - now;

            espnow_sync_pkt_t echo = {
                .type      = ESPNOW_PKT_SYNC,
                .node_id   = s_node_id,
                .master_us = pkt->master_us,
                .slave_us  = now,
            };
            /* Broadcast back — master will hear it */
            uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
            esp_now_send(bcast, (uint8_t *)&echo, sizeof(echo));
        }
        else if (type == ESPNOW_PKT_SYNC && s_is_master) {
            /* Master: receive echo from slave, calculate offset */
            espnow_sync_pkt_t *echo = (espnow_sync_pkt_t *)buf;
            int64_t now = esp_timer_get_time();
            int64_t rtt = now - echo->master_us;
            int64_t offset = echo->master_us + rtt/2 - echo->slave_us;
            for (int i = 0; i < s_peer_count; i++) {
                if (s_peers[i].id == echo->node_id) {
                    s_peers[i].offset_us = offset;
                    s_peers[i].valid = true;
                }
            }
        }
        else if (type == ESPNOW_PKT_DATA && s_is_master) {
            espnow_data_pkt_t *d = (espnow_data_pkt_t *)buf;
            /* Correct slave timestamp to master epoch */
            for (int i = 0; i < s_peer_count; i++) {
                if (s_peers[i].id == d->node_id && s_peers[i].valid) {
                    d->timestamp_us += s_peers[i].offset_us;
                    break;
                }
            }
            if (s_data_cb) s_data_cb(d);
        }
        else if (type == ESPNOW_PKT_CMD && !s_is_master) {
            /* Slaves handle start/stop/config commands */
            espnow_cmd_pkt_t *cmd = (espnow_cmd_pkt_t *)buf;
            if (cmd->node_id != 0xFF && cmd->node_id != s_node_id) {
                free(buf); continue;
            }
            /* Commands processed in main.c via a separate command queue;
               here we just log for now — wiring done in main.c          */
            ESP_LOGI(TAG, "CMD %c param=%ld", (char)cmd->cmd, (long)cmd->param);
        }

        free(buf);
    }
}

/* ── Master sync broadcast task ─────────────────────────────────────── */
static void espnow_sync_task(void *arg)
{
    uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    while (1) {
        espnow_sync_pkt_t pkt = {
            .type      = ESPNOW_PKT_SYNC,
            .node_id   = s_node_id,
            .master_us = esp_timer_get_time(),
            .slave_us  = 0,
        };
        esp_now_send(bcast, (uint8_t *)&pkt, sizeof(pkt));
        vTaskDelay(pdMS_TO_TICKS(ESPNOW_SYNC_INTERVAL_MS));
    }
}

/* ── Public API ──────────────────────────────────────────────────────── */
esp_err_t espnow_sync_init(uint8_t node_id, bool is_master)
{
    s_node_id  = node_id;
    s_is_master = is_master;
    s_rx_queue = xQueueCreate(32, sizeof(uintptr_t[2]));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    /* Add broadcast peer */
    esp_now_peer_info_t bcast_peer = {
        .channel = ESPNOW_CHANNEL,
        .ifidx   = WIFI_IF_STA,
        .encrypt = false,
    };
    memset(bcast_peer.peer_addr, 0xFF, 6);
    esp_now_add_peer(&bcast_peer);

    xTaskCreate(espnow_rx_task, "espnow_rx", 4096, NULL, 10, NULL);
    if (is_master)
        xTaskCreate(espnow_sync_task, "espnow_sync", 2048, NULL, 8, NULL);

    ESP_LOGI(TAG, "ESPNow init: node=%d role=%s",
             node_id, is_master ? "MASTER" : "SLAVE");
    return ESP_OK;
}

esp_err_t espnow_sync_add_peer(const uint8_t mac[6])
{
    if (s_peer_count >= ESPNOW_MAX_NODES) return ESP_ERR_NO_MEM;
    esp_now_peer_info_t pi = {
        .channel = ESPNOW_CHANNEL,
        .ifidx   = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(pi.peer_addr, mac, 6);
    esp_err_t ret = esp_now_add_peer(&pi);
    if (ret == ESP_OK) {
        s_peers[s_peer_count].id = s_peer_count;
        memcpy(s_peers[s_peer_count].mac, mac, 6);
        s_peers[s_peer_count].valid = false;
        s_peer_count++;
    }
    return ret;
}

esp_err_t espnow_master_send_sync(void)
{
    uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    espnow_sync_pkt_t pkt = {
        .type      = ESPNOW_PKT_SYNC,
        .node_id   = s_node_id,
        .master_us = esp_timer_get_time(),
        .slave_us  = 0,
    };
    return esp_now_send(bcast, (uint8_t *)&pkt, sizeof(pkt));
}

esp_err_t espnow_master_send_cmd(uint8_t target_id, uint8_t cmd, int32_t param)
{
    uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    espnow_cmd_pkt_t pkt = {
        .type    = ESPNOW_PKT_CMD,
        .node_id = target_id,
        .cmd     = cmd,
        .param   = param,
    };
    return esp_now_send(bcast, (uint8_t *)&pkt, sizeof(pkt));
}

esp_err_t espnow_slave_send_data(const espnow_data_pkt_t *pkt)
{
    uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    return esp_now_send(bcast, (const uint8_t *)pkt, sizeof(*pkt));
}

int64_t espnow_slave_time_offset_us(void)
{
    return s_time_offset_us;
}

void espnow_set_data_callback(espnow_data_cb_t cb)
{
    s_data_cb = cb;
}

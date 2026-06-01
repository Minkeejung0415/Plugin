/* Network streaming — protocol-compatible with RedPitaya_justin.c.
   TCP:5000  control channel  (Open Ephys sends START/STOP/FREQ/CFG…)
   UDP:55001 sample data      (same 22-byte header + int16 payload)
   UDP:5005  quaternion stream (v2 float format → OpenSim bridge)       */
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "include/config.h"
#include "include/network_stream.h"
#include "include/sdcard_log.h"

static const char *TAG = "net_stream";

/* ── Live state ──────────────────────────────────────────────────────── */
static volatile uint16_t s_rate_hz       = DEFAULT_SAMPLE_RATE_HZ;
static volatile bool     s_fusion_on     = true;
static volatile bool     s_recording     = false;
static volatile bool     s_streaming     = false;
static volatile uint16_t s_total_ch      = CHANNELS_PER_IMU + 1; /* +DIO */
static SemaphoreHandle_t s_cfg_mutex;

/* Connected Open Ephys clients (up to MAX_OPEN_EPHYS_CLIENTS) */
static struct {
    int  udp_sock;
    struct sockaddr_in addr;
    bool valid;
} s_clients[MAX_OPEN_EPHYS_CLIENTS];
static int s_client_count = 0;

static int  s_quat_sock = -1;
static struct sockaddr_in s_quat_addr;   /* destination filled by TCP handshake */

/* ── UDP data packet (same layout as RedPitaya) ───────────────────────── */
typedef struct __attribute__((packed)) {
    int32_t  offset;
    int32_t  bytes_per_frame;
    int16_t  dtype;
    int32_t  elements;
    int32_t  total_channels;
    int32_t  sequence;
} udp_header_t;

static void build_and_send_udp(const int16_t *ch, uint16_t nch, uint32_t seq)
{
    size_t payload = nch * sizeof(int16_t);
    size_t pkt_sz  = sizeof(udp_header_t) + payload;
    uint8_t *pkt = malloc(pkt_sz);
    if (!pkt) return;

    udp_header_t *hdr = (udp_header_t *)pkt;
    hdr->offset         = 0;
    hdr->bytes_per_frame = (int32_t)payload;
    hdr->dtype          = 3;
    hdr->elements       = 2;
    hdr->total_channels = nch;
    hdr->sequence       = (int32_t)seq;
    memcpy(pkt + sizeof(udp_header_t), ch, payload);

    for (int i = 0; i < s_client_count; i++) {
        if (s_clients[i].valid)
            sendto(s_clients[i].udp_sock, pkt, pkt_sz, 0,
                   (struct sockaddr *)&s_clients[i].addr,
                   sizeof(s_clients[i].addr));
    }
    free(pkt);
}

/* ── TCP control task ────────────────────────────────────────────────── */
static void handle_tcp_client(int sock, struct sockaddr_in *cli_addr)
{
    char buf[256];
    /* Handshake */
    int n = recv(sock, buf, sizeof(buf)-1, 0);
    if (n <= 0) { close(sock); return; }
    buf[n] = '\0';

    if (strncmp(buf, "REDPITAYA", 9) != 0) {
        close(sock); return;
    }

    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    char resp[64];
    snprintf(resp, sizeof(resp), "OK CHANNELS:%d\n", s_total_ch);
    send(sock, resp, strlen(resp), 0);
    xSemaphoreGive(s_cfg_mutex);

    /* Register this client's UDP endpoint */
    if (s_client_count < MAX_OPEN_EPHYS_CLIENTS) {
        int udp_s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (udp_s >= 0) {
            s_clients[s_client_count].udp_sock = udp_s;
            s_clients[s_client_count].addr = *cli_addr;
            s_clients[s_client_count].addr.sin_port = htons(UDP_DATA_PORT);
            s_clients[s_client_count].valid = true;
            s_client_count++;
        }
    }

    /* Command loop */
    while (1) {
        n = recv(sock, buf, sizeof(buf)-1, MSG_DONTWAIT);
        if (n <= 0 && errno != EWOULDBLOCK) break;
        if (n <= 0) { vTaskDelay(1); continue; }
        buf[n] = '\0';

        char *line = strtok(buf, "\n");
        while (line) {
            xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);

            if (strncmp(line, "START", 5) == 0) {
                s_streaming = true;
                char sr[128];
                snprintf(sr, sizeof(sr), "STARTED\nSENSORS:0,ICM20948\n");
                send(sock, sr, strlen(sr), 0);
                ESP_LOGI(TAG, "Streaming started → client");

            } else if (strncmp(line, "STOP", 4) == 0) {
                s_streaming = false;
                ESP_LOGI(TAG, "Streaming stopped");

            } else if (strncmp(line, "FREQ:", 5) == 0) {
                int hz = atoi(line + 5);
                if (hz >= 1 && hz <= MAX_SAMPLE_RATE_HZ)
                    s_rate_hz = (uint16_t)hz;
                ESP_LOGI(TAG, "Rate → %d Hz", s_rate_hz);

            } else if (strncmp(line, "FILTER ON", 9)  == 0 ||
                       strncmp(line, "FUSION ON", 9)  == 0) {
                s_fusion_on = true;

            } else if (strncmp(line, "FILTER OFF", 10) == 0 ||
                       strncmp(line, "FUSION OFF", 10) == 0) {
                s_fusion_on = false;

            } else if (strncmp(line, "RECORD ON", 9) == 0) {
                s_recording = true;
                char path[64];
                sdcard_open_session(path, sizeof(path));
                char rm[80];
                snprintf(rm, sizeof(rm), "STARTED BIN:%s.bin CSV:%s.csv\n", path, path);
                send(sock, rm, strlen(rm), 0);

            } else if (strncmp(line, "RECORD OFF", 10) == 0) {
                s_recording = false;
                sdcard_close_session();

            } else if (strncmp(line, "CFG ", 4) == 0) {
                /* CFG <sensor_idx> ACC/GYR/SRATE <value>
                   Single IMU node — idx must be 0                      */
                int idx, val; char sub[8];
                if (sscanf(line+4, "%d %7s %d", &idx, sub, &val) == 3 && idx == 0) {
                    extern esp_err_t icm20948_set_accel_range(uint8_t);
                    extern esp_err_t icm20948_set_gyro_range(uint8_t);
                    if      (strcmp(sub, "ACC")   == 0) icm20948_set_accel_range(val);
                    else if (strcmp(sub, "GYR")   == 0) icm20948_set_gyro_range(val);
                    else if (strcmp(sub, "SRATE") == 0)
                        if (val >= 1 && val <= MAX_SAMPLE_RATE_HZ)
                            s_rate_hz = val;
                }
                /* Send ACK so Open Ephys knows the command landed */
                char ack[32];
                snprintf(ack, sizeof(ack), "OK:%s\n", line);
                send(sock, ack, strlen(ack), 0);
            }

            xSemaphoreGive(s_cfg_mutex);
            line = strtok(NULL, "\n");
        }
    }

    /* Remove client */
    for (int i = 0; i < s_client_count; i++) {
        if (s_clients[i].valid) {
            close(s_clients[i].udp_sock);
            s_clients[i].valid = false;
        }
    }
    s_client_count = 0;
    close(sock);
}

static void tcp_server_task(void *arg)
{
    int srv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in sa = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port        = htons(TCP_CTRL_PORT),
    };
    bind(srv, (struct sockaddr *)&sa, sizeof(sa));
    listen(srv, 4);
    ESP_LOGI(TAG, "TCP listening on :%d", TCP_CTRL_PORT);

    while (1) {
        struct sockaddr_in cli; socklen_t cl = sizeof(cli);
        int c = accept(srv, (struct sockaddr *)&cli, &cl);
        if (c < 0) { vTaskDelay(10); continue; }
        ESP_LOGI(TAG, "TCP client connected");
        handle_tcp_client(c, &cli);
    }
}

/* ── Quaternion UDP socket (to OpenSim) ────────────────────────────── */
static void init_quat_socket(void)
{
    s_quat_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    /* Default destination: broadcast on LAN; Open Ephys plugin overrides
       via the same IP it uses for UDP_DATA_PORT.                        */
    s_quat_addr.sin_family      = AF_INET;
    s_quat_addr.sin_port        = htons(UDP_QUAT_PORT);
    s_quat_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    int bcast = 1;
    setsockopt(s_quat_sock, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));
}

/* ── Public API ──────────────────────────────────────────────────────── */
esp_err_t network_stream_init(void)
{
    s_cfg_mutex = xSemaphoreCreateMutex();
    init_quat_socket();
    xTaskCreate(tcp_server_task, "tcp_ctrl", 6144, NULL, 8, NULL);
    return ESP_OK;
}

void network_stream_push_sample(const int16_t *channels, uint16_t nch,
                                uint32_t seq)
{
    if (!s_streaming) return;
    build_and_send_udp(channels, nch, seq);
}

void network_stream_push_quats(const float *quats, uint8_t n_sensors,
                               float timestamp_s)
{
    if (s_quat_sock < 0) return;
    /* v2 format: timestamp(f) version(f) n_sensors(f) quats[n*4](f) */
    size_t sz = (3 + n_sensors * 4) * sizeof(float);
    float *pkt = malloc(sz);
    if (!pkt) return;
    pkt[0] = timestamp_s;
    pkt[1] = 2.0f;
    pkt[2] = (float)n_sensors;
    memcpy(&pkt[3], quats, n_sensors * 4 * sizeof(float));
    sendto(s_quat_sock, pkt, sz, 0,
           (struct sockaddr *)&s_quat_addr, sizeof(s_quat_addr));
    free(pkt);
}

uint16_t network_stream_get_rate(void)   { return s_rate_hz; }
bool     network_stream_fusion_enabled(void) { return s_fusion_on; }
bool     network_stream_recording(void)  { return s_recording; }

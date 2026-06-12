#include "open_ephys_stream.h"

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "sd_logger.h"

static const char *TAG = "oe_stream";

static oe_sample_t s_latest;
static SemaphoreHandle_t s_sample_mux;
static int s_client_fd = -1;
static int s_listen_fd = -1;
static volatile bool s_streaming = false;

/* Open Ephys Ephys-Socket header: little-endian iiHiii (22 bytes) */
typedef struct __attribute__((packed)) {
    int32_t offset;
    int32_t num_bytes;
    uint16_t bit_depth;
    int32_t element_size;
    int32_t num_channels;
    int32_t samples_per_channel;
} oe_header_t;

static void pack_packet(const oe_sample_t *s, uint8_t *out, size_t *out_len)
{
    const int num_ch = OE_STREAM_NUM_CHANNELS;
    const int n_per_ch = OE_STREAM_SAMPLES_PER_CHANNEL;
    const int element_size = 2;
    const int num_bytes = num_ch * n_per_ch * element_size;

    oe_header_t hdr = {
        .offset = 0,
        .num_bytes = num_bytes,
        .bit_depth = 3,
        .element_size = element_size,
        .num_channels = num_ch,
        .samples_per_channel = n_per_ch,
    };

    memcpy(out, &hdr, sizeof(hdr));
    int16_t *payload = (int16_t *)(out + sizeof(hdr));
    for (int c = 0; c < num_ch; c++) {
        payload[c] = s->ch[c];
    }
    *out_len = sizeof(hdr) + (size_t)num_bytes;
}

static void handle_handshake(int fd, const char *line)
{
    if (strncmp(line, "REDPITAYA", 9) == 0) {
        const char *resp = "8 channels; sample_rate=100; node=esp32s3\n";
        send(fd, resp, strlen(resp), 0);
        ESP_LOGI(TAG, "Handshake REDPITAYA");
    } else if (strncmp(line, "START", 5) == 0) {
        s_streaming = true;
        ESP_LOGI(TAG, "Streaming START");
    } else if (strncmp(line, "RECORD ON", 9) == 0) {
        const char *path = line + 9;
        while (*path == ' ') {
            path++;
        }
        sd_logger_start(*path ? path : NULL);
        ESP_LOGI(TAG, "Recording command ON");
    } else if (strncmp(line, "RECORD OFF", 10) == 0) {
        sd_logger_stop();
        ESP_LOGI(TAG, "Recording command OFF");
    }
}

static void stream_task(void *arg)
{
    uint8_t packet[64];
    size_t packet_len;
    TickType_t period = pdMS_TO_TICKS(1000 / OE_STREAM_SAMPLE_HZ);

    while (1) {
        if (s_client_fd < 0) {
            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            s_client_fd = accept(s_listen_fd, (struct sockaddr *)&client_addr, &addr_len);
            if (s_client_fd >= 0) {
                s_streaming = false;
                ESP_LOGI(TAG, "Client connected fd=%d", s_client_fd);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        char line[64];
        int n = recv(s_client_fd, line, sizeof(line) - 1, MSG_DONTWAIT);
        if (n > 0) {
            line[n] = '\0';
            char *nl = strchr(line, '\n');
            if (nl) {
                *nl = '\0';
                handle_handshake(s_client_fd, line);
            }
        }

        if (s_streaming) {
            oe_sample_t local;
            if (xSemaphoreTake(s_sample_mux, pdMS_TO_TICKS(10)) == pdTRUE) {
                local = s_latest;
                xSemaphoreGive(s_sample_mux);
            }
            pack_packet(&local, packet, &packet_len);
            int sent = send(s_client_fd, packet, packet_len, 0);
            if (sent < 0) {
                close(s_client_fd);
                s_client_fd = -1;
                s_streaming = false;
            }
        }
        vTaskDelay(period);
    }
}

void open_ephys_stream_init(void)
{
    s_sample_mux = xSemaphoreCreateMutex();
    memset(&s_latest, 0, sizeof(s_latest));
}

void open_ephys_stream_set_sample(const oe_sample_t *sample)
{
    if (xSemaphoreTake(s_sample_mux, pdMS_TO_TICKS(5)) == pdTRUE) {
        s_latest = *sample;
        xSemaphoreGive(s_sample_mux);
    }
}

void open_ephys_stream_start_server(uint16_t port)
{
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(port),
    };

    s_listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    bind(s_listen_fd, (struct sockaddr *)&addr, sizeof(addr));
    listen(s_listen_fd, 1);
    ESP_LOGI(TAG, "TCP listen port %u", (unsigned)port);
    xTaskCreate(stream_task, "oe_tcp", 4096, NULL, 5, NULL);
}

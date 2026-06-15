#include "open_ephys_stream.h"

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
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
#define REC_RECONNECT_GRACE_MS 90000u
#define REC_MAX_CHUNK 1024u
#define SDRF_HEADER_LEN 64u
#define SDRF_TYPE_DATA 0x01u
#define SDRF_TYPE_EOF 0x02u

typedef struct __attribute__((packed)) {
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
} sdrf_header_t;

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

static uint32_t rec_crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

static void send_linef(int fd, const char *fmt, ...)
{
    char out[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(out, sizeof(out), fmt, ap);
    va_end(ap);
    if (n > 0) {
        send(fd, out, (size_t)n < sizeof(out) ? (size_t)n : sizeof(out) - 1, 0);
    }
}

static const char *field_value(const char *line, const char *key, char *buf, size_t len)
{
    const char *p = strstr(line, key);
    if (!p) {
        return NULL;
    }
    p += strlen(key);
    if (*p != '=') {
        return NULL;
    }
    p++;
    size_t i = 0;
    while (*p && *p != ' ' && i + 1 < len) {
        buf[i++] = *p++;
    }
    buf[i] = '\0';
    return buf;
}

static void send_rec_status(int fd)
{
    sd_logger_stats_t st;
    sd_logger_get_stats(&st);
    send_linef(fd,
               "REC STATUS_OK protocol=rec-v1 capabilities=record_control,status_v1,finalized_metadata,chunk_transfer_v1,whole_file_checksum,reconnect_grace,transfer_isolation=paused_isolated_stream transport=direct_tcp sd_ready=%d sd_open=%d recording_state=%s transfer_state=%s session_id=%s sd_path_token=%s generated_samples=%llu saved_samples=%llu queue_drops=%llu write_errors=%llu max_write_latency_us=%lu overrun_count=%llu finalization_reason=%s file_byte_size=%llu file_checksum=%08lx checksum_type=crc32 last_error=%s grace_ms_remaining=%lu local_result_path=unknown local_analyzer_result=unknown\r\n",
               st.sd_ready ? 1 : 0,
               st.file_open ? 1 : 0,
               st.recording_state[0] ? st.recording_state : "idle",
               st.transfer_state[0] ? st.transfer_state : "none",
               st.session_id[0] ? st.session_id : "none",
               st.sd_path_token[0] ? st.sd_path_token : "none",
               (unsigned long long)st.generated_samples,
               (unsigned long long)st.saved_samples,
               (unsigned long long)st.sd_queue_drops,
               (unsigned long long)st.sd_write_errors,
               (unsigned long)st.max_sd_write_us,
               (unsigned long long)st.acq_loop_overruns,
               st.finalization_reason[0] ? st.finalization_reason : "none",
               (unsigned long long)st.file_byte_size,
               (unsigned long)st.file_checksum,
               st.last_error[0] ? st.last_error : "none",
               (unsigned long)st.grace_ms_remaining);
}

static void session_id_bytes(const char *session_id, uint8_t out[16])
{
    memset(out, 0, 16);
    size_t n = strlen(session_id);
    if (n > 16) {
        n = 16;
    }
    memcpy(out, session_id, n);
}

static void send_sdrf_frame(int fd, const char *session_id, uint8_t type,
                            uint32_t chunk_index, uint64_t offset,
                            const uint8_t *payload, uint32_t payload_len,
                            uint64_t total_size, uint32_t flags)
{
    sdrf_header_t hdr = {
        .magic = {'S', 'D', 'R', 'F'},
        .frame_version = 1,
        .frame_type = type,
        .header_length = SDRF_HEADER_LEN,
        .chunk_index = chunk_index,
        .byte_offset = offset,
        .payload_length = payload_len,
        .total_file_size = total_size,
        .payload_checksum = payload && payload_len ? rec_crc32_update(0, payload, payload_len) : 0,
        .flags = flags,
        .reserved = 0,
    };
    session_id_bytes(session_id, hdr.session_id);
    hdr.header_crc32 = 0;
    hdr.header_crc32 = rec_crc32_update(0, (const uint8_t *)&hdr, sizeof(hdr));
    send(fd, &hdr, sizeof(hdr), 0);
    if (payload && payload_len) {
        send(fd, payload, payload_len, 0);
    }
}

static void handle_rec_command(int fd, const char *line)
{
    if (strncmp(line, "REC HELLO", 9) == 0) {
        if (!strstr(line, "protocol_min=rec-v1")) {
            send_linef(fd, "REC ERR code=unsupported_protocol retryable=false detail=protocol_min\r\n");
            return;
        }
        sd_logger_mark_host_connected();
        send_linef(fd,
                   "REC HELLO_OK protocol=rec-v1 firmware=esp-idf-step transport=direct_tcp capabilities=record_control,status_v1,finalized_metadata,chunk_transfer_v1,whole_file_checksum,reconnect_grace,transfer_isolation=paused_isolated_stream max_chunk=%u analyzer=sd-bin-v1 grace_ms=%u\r\n",
                   REC_MAX_CHUNK, REC_RECONNECT_GRACE_MS);
        return;
    }

    if (strncmp(line, "REC START", 9) == 0) {
        sd_logger_stats_t st;
        sd_logger_get_stats(&st);
        if (st.recording_requested || st.file_open) {
            send_linef(fd, "REC ERR code=already_recording session_id=%s retryable=false detail=active\r\n",
                       st.session_id[0] ? st.session_id : "none");
            return;
        }
        char requested[40];
        const char *req = field_value(line, "requested_session", requested, sizeof(requested));
        bool ok = sd_logger_start_session(req, NULL);
        sd_logger_get_stats(&st);
        if (!ok) {
            send_linef(fd, "REC ERR code=sd_not_ready retryable=true detail=start_failed\r\n");
            return;
        }
        send_linef(fd,
                   "REC STARTED session_id=%s sd_path_token=%s recording_state=recording generated_samples=%llu saved_samples=%llu\r\n",
                   st.session_id, st.sd_path_token,
                   (unsigned long long)st.generated_samples,
                   (unsigned long long)st.saved_samples);
        return;
    }

    if (strncmp(line, "REC STATUS", 10) == 0) {
        send_rec_status(fd);
        return;
    }

    if (strncmp(line, "REC STOP", 8) == 0) {
        sd_logger_stats_t st;
        sd_logger_get_stats(&st);
        if (!st.recording_requested && !st.file_open) {
            send_linef(fd, "REC ERR code=not_recording session_id=%s retryable=false detail=idle\r\n",
                       st.session_id[0] ? st.session_id : "none");
            return;
        }
        sd_logger_stop_with_reason("manual_stop");
        send_linef(fd, "REC FINALIZING session_id=%s\r\n", st.session_id);
        return;
    }

    if (strncmp(line, "REC SESSION", 11) == 0) {
        char sid[40];
        const char *session_id = field_value(line, "session_id", sid, sizeof(sid));
        sd_logger_session_t session;
        if (!sd_logger_get_session(session_id ? session_id : "latest_finalized", &session)) {
            sd_logger_stats_t st;
            sd_logger_get_stats(&st);
            const char *code = (st.recording_requested || st.file_open) ? "busy_recording" : "not_found";
            send_linef(fd, "REC ERR code=%s retryable=%s detail=session\r\n",
                       code, strcmp(code, "busy_recording") == 0 ? "true" : "false");
            return;
        }
        send_linef(fd,
                   "REC SESSION_OK session_id=%s sd_path_token=%s file_size=%llu file_checksum=%08lx checksum_type=crc32 sample_count=%llu finalized_at=unknown finalization_reason=%s analyzer_format=sd-bin-v1\r\n",
                   session.session_id, session.sd_path_token,
                   (unsigned long long)session.file_byte_size,
                   (unsigned long)session.file_checksum,
                   (unsigned long long)session.sample_count,
                   session.finalization_reason);
        return;
    }

    if (strncmp(line, "REC GET", 7) == 0) {
        char sid[40], off_buf[24], len_buf[16], idx_buf[16];
        const char *session_id = field_value(line, "session_id", sid, sizeof(sid));
        uint64_t offset = field_value(line, "offset", off_buf, sizeof(off_buf)) ? strtoull(off_buf, NULL, 10) : 0;
        uint32_t length = field_value(line, "length", len_buf, sizeof(len_buf)) ? (uint32_t)strtoul(len_buf, NULL, 10) : REC_MAX_CHUNK;
        uint32_t chunk_index = field_value(line, "chunk_index", idx_buf, sizeof(idx_buf)) ? (uint32_t)strtoul(idx_buf, NULL, 10) : 0;
        if (length > REC_MAX_CHUNK) {
            length = REC_MAX_CHUNK;
        }
        sd_logger_session_t session;
        if (!sd_logger_get_session(session_id, &session)) {
            send_linef(fd, "REC ERR code=not_finalized retryable=true detail=session\r\n");
            return;
        }
        if (!sd_logger_transfer_begin(session.session_id)) {
            send_linef(fd, "REC ERR code=transfer_busy retryable=true detail=state\r\n");
            return;
        }
        uint8_t buf[REC_MAX_CHUNK];
        size_t got = 0;
        int rc = sd_logger_read_chunk(session.session_id, offset, buf, length, &got);
        if (rc == -2) {
            send_linef(fd, "REC ERR code=offset_out_of_range session_id=%s retryable=false detail=offset\r\n", session.session_id);
            return;
        }
        if (rc != 0) {
            send_linef(fd, "REC ERR code=sd_error session_id=%s retryable=true detail=read\r\n", session.session_id);
            return;
        }
        send_sdrf_frame(fd, session.session_id, SDRF_TYPE_DATA, chunk_index, offset, buf, (uint32_t)got, session.file_byte_size,
                        offset + got >= session.file_byte_size ? 0x04u : 0u);
        if (offset + got >= session.file_byte_size) {
            send_sdrf_frame(fd, session.session_id, SDRF_TYPE_EOF, chunk_index + 1, session.file_byte_size, NULL, 0, session.file_byte_size, 0);
        }
        return;
    }

    if (strncmp(line, "REC COMPLETE", 12) == 0) {
        char sid[40], size_buf[24], sum_buf[16];
        const char *session_id = field_value(line, "session_id", sid, sizeof(sid));
        uint64_t file_size = field_value(line, "file_size", size_buf, sizeof(size_buf)) ? strtoull(size_buf, NULL, 10) : 0;
        uint32_t checksum = field_value(line, "file_checksum", sum_buf, sizeof(sum_buf)) ? (uint32_t)strtoul(sum_buf, NULL, 16) : 0;
        if (sd_logger_transfer_complete(session_id, file_size, checksum)) {
            send_linef(fd, "REC COMPLETE_OK session_id=%s transfer_state=complete\r\n", session_id ? session_id : "latest_finalized");
        } else {
            sd_logger_stats_t st;
            sd_logger_get_stats(&st);
            send_linef(fd, "REC ERR code=%s session_id=%s retryable=true detail=complete\r\n",
                       st.last_error[0] ? st.last_error : "checksum_mismatch",
                       session_id ? session_id : "unknown");
        }
        return;
    }

    if (strncmp(line, "REC ABORT", 9) == 0) {
        char sid[40];
        const char *session_id = field_value(line, "session_id", sid, sizeof(sid));
        if (sd_logger_transfer_abort(session_id)) {
            send_linef(fd, "REC ABORTED session_id=%s transfer_state=aborted\r\n", session_id ? session_id : "latest_finalized");
        } else {
            send_linef(fd, "REC ERR code=not_active session_id=%s retryable=false detail=abort\r\n", session_id ? session_id : "unknown");
        }
        return;
    }

    if (strncmp(line, "REC CLEAR", 9) == 0) {
        char scope[24];
        const char *value = field_value(line, "scope", scope, sizeof(scope));
        if (sd_logger_clear(value)) {
            send_linef(fd, "REC CLEAR_OK scope=%s\r\n", value ? value : "unknown");
        } else {
            sd_logger_stats_t st;
            sd_logger_get_stats(&st);
            send_linef(fd, "REC ERR code=%s retryable=false detail=clear\r\n",
                       st.last_error[0] ? st.last_error : "invalid_scope");
        }
        return;
    }

    send_linef(fd, "REC ERR code=unsupported retryable=false detail=command\r\n");
}

static void handle_handshake(int fd, const char *line)
{
    if (strncmp(line, "REC ", 4) == 0) {
        handle_rec_command(fd, line);
    } else if (strncmp(line, "REDPITAYA", 9) == 0) {
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
                sd_logger_mark_host_connected();
                ESP_LOGI(TAG, "Client connected fd=%d", s_client_fd);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        sd_logger_poll();

        char line[256];
        int n = recv(s_client_fd, line, sizeof(line) - 1, MSG_DONTWAIT);
        if (n > 0) {
            line[n] = '\0';
            char *nl = strchr(line, '\n');
            if (nl) {
                *nl = '\0';
                handle_handshake(s_client_fd, line);
            }
        } else if (n == 0) {
            close(s_client_fd);
            s_client_fd = -1;
            s_streaming = false;
            sd_logger_mark_host_disconnected();
            continue;
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
                sd_logger_mark_host_disconnected();
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

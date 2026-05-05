/*
 * Standalone framing test for the Red Pitaya binary stream header.
 * Mirrors write_stream_header() in RedPitaya_justin.c and the resync logic
 * in AcqBoardRedPitaya::run() (bytes-per-frame from header, not channel count).
 *
 * Build and run (from repo root):
 *   cc -std=c99 -Wall -Wextra -o /tmp/rp_framing_test tests/redpitaya_stream_framing_test.c && /tmp/rp_framing_test
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define HEADER_SIZE 22
#define MAX_PAYLOAD (1024 - HEADER_SIZE)

static void write_stream_header(uint8_t *packet, int bytes_per_frame, int total_channels, int32_t ns)
{
    int32_t offset = 0;
    int32_t bpb = bytes_per_frame;
    int32_t elm = 2;
    int16_t dtype = 3;

    memcpy(packet + 0, &offset, 4);
    memcpy(packet + 4, &bpb, 4);
    memcpy(packet + 8, &dtype, 2);
    memcpy(packet + 10, &elm, 4);
    memcpy(packet + 14, &total_channels, 4);
    memcpy(packet + 18, &ns, 4);
}

static int parse_bpb(const uint8_t *hdr, int32_t *out_bpb)
{
    if (hdr[8] != 0x03 || hdr[9] != 0x00)
        return 0;
    memcpy(out_bpb, hdr + 4, 4);
    if (*out_bpb < 2 || *out_bpb > MAX_PAYLOAD || (*out_bpb & 1))
        return 0;
    return 1;
}

static int simulate_read(const uint8_t *stream, size_t len, size_t *pos, void *dst, size_t n)
{
    if (*pos + n > len)
        return 0;
    memcpy(dst, stream + *pos, n);
    *pos += n;
    return 1;
}

static int read_one_frame(const uint8_t *stream, size_t len, size_t *pos,
                            uint8_t *packet, int32_t *payload_bytes)
{
    const int headerSize = HEADER_SIZE;

    if (! simulate_read(stream, len, pos, packet, (size_t) headerSize))
        return 0;

    for (int guard = 0; guard < 64; ++guard) {
        if (parse_bpb(packet, payload_bytes))
            return simulate_read(stream, len, pos, packet + headerSize, (size_t) *payload_bytes);

        memmove(packet, packet + 1, (size_t) headerSize - 1);
        if (! simulate_read(stream, len, pos, packet + headerSize - 1, 1))
            return 0;
    }
    return 0;
}

static int test_samples_per_buffer(void)
{
    struct { float rate; int64_t expected_min; } cases[] = {
        {  100.0f, 1 },
        {  250.0f, 1 },
        {  500.0f, 1 },
        { 1000.0f, 1 },
        { 2000.0f, 2 },
    };
    int n = (int)(sizeof(cases) / sizeof(cases[0]));

    for (int i = 0; i < n; i++) {
        int64_t old_val = (int64_t)(cases[i].rate / 1000.0);
        int64_t new_val = old_val > 1 ? old_val : 1;

        if (new_val < cases[i].expected_min) {
            fprintf(stderr, "FAIL: samplesPerBuffer for %.0f Hz = %lld, expected >= %lld\n",
                    (double)cases[i].rate, (long long)new_val, (long long)cases[i].expected_min);
            return 1;
        }
    }

    printf("OK: samplesPerBuffer >= 1 for all supported rates\n");
    return 0;
}

int main(void)
{
    uint8_t blob[512];
    size_t w = 0;
    int32_t ch = 5;
    int32_t bpb1 = ch * 2;
    int32_t bpb2 = (ch + 4) * 2; /* e.g. fusion adds 4 int16 slots */

    /* garbage prefix to force resync */
    blob[w++] = 0xAB;

    uint8_t pkt[HEADER_SIZE + 256];
    memset(pkt, 0, sizeof(pkt));
    write_stream_header(pkt, bpb1, ch, 0);
    memset(pkt + HEADER_SIZE, 0x11, (size_t) bpb1);
    memcpy(blob + w, pkt, HEADER_SIZE + (size_t) bpb1);
    w += HEADER_SIZE + (size_t) bpb1;

    write_stream_header(pkt, bpb2, ch + 4, 1);
    memset(pkt + HEADER_SIZE, 0x22, (size_t) bpb2);
    memcpy(blob + w, pkt, HEADER_SIZE + (size_t) bpb2);
    w += HEADER_SIZE + (size_t) bpb2;

    size_t pos = 0;
    uint8_t frame[HEADER_SIZE + 256];
    int32_t pay = 0;

    if (! read_one_frame(blob, w, &pos, frame, &pay)) {
        fprintf(stderr, "FAIL: first frame\n");
        return 1;
    }
    if (pay != bpb1) {
        fprintf(stderr, "FAIL: expected payload %d got %d\n", bpb1, pay);
        return 1;
    }
    if (frame[HEADER_SIZE] != 0x11) {
        fprintf(stderr, "FAIL: first payload marker\n");
        return 1;
    }

    if (! read_one_frame(blob, w, &pos, frame, &pay)) {
        fprintf(stderr, "FAIL: second frame\n");
        return 1;
    }
    if (pay != bpb2) {
        fprintf(stderr, "FAIL: expected payload %d got %d\n", bpb2, pay);
        return 1;
    }
    if (frame[HEADER_SIZE] != 0x22) {
        fprintf(stderr, "FAIL: second payload marker\n");
        return 1;
    }

    if (pos != w) {
        fprintf(stderr, "FAIL: trailing bytes %zu != total %zu\n", pos, w);
        return 1;
    }

    printf("OK: variable payload framing and resync\n");

    if (test_samples_per_buffer() != 0)
        return 1;

    printf("ALL TESTS PASSED\n");
    return 0;
}

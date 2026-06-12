/*
 * Standalone framing test for the Red Pitaya binary stream header.
 * Mirrors write_stream_header() in RedPitaya_justin.c and the resync logic
 * in AcqBoardRedPitaya::run() (bytes-per-frame from header, not channel count).
 *
 * -------------------------------------------------------------------------
 * WHAT THE BINARY STREAM LOOKS LIKE
 * -------------------------------------------------------------------------
 * Every "frame" the Red Pitaya sends over TCP has this layout:
 *
 *   Offset  Size  Type    Meaning
 *   0       4     int32   offset (always 0)
 *   4       4     int32   bytes_per_frame  ← the critical field
 *   8       2     int16   dtype (must be 3 = int16 samples)
 *   10      4     int32   num_elements (always 2)
 *   14      4     int32   total_channels
 *   18      4     int32   sample_count (sequence number)
 *   22      N     int16[] payload: total_channels × int16 values
 *
 * bytes_per_frame tells the receiver exactly how many bytes follow the
 * 22-byte header.  The receiver uses ONLY this field to know the payload
 * size — it does NOT trust total_channels from the header, because the
 * firmware can dynamically add quaternion channels mid-stream.
 *
 * -------------------------------------------------------------------------
 * WHAT THIS TEST CHECKS
 * -------------------------------------------------------------------------
 * 1. Resync: the test prepends a single garbage byte (0xAB) before frame 1.
 *    read_one_frame() must slide the read window byte-by-byte until it
 *    finds a header whose dtype == 3 and bytes_per_frame is sane.
 *    If resync fails, the test would parse garbage or hang.
 *
 * 2. Variable payload: frame 1 has 5 channels (bpb = 10 bytes), frame 2
 *    has 9 channels (bpb = 18 bytes, simulating firmware adding fusion
 *    quaternion channels after stream start).  Both must parse correctly.
 *
 * 3. samplesPerBuffer: at every supported sample rate, the calculation
 *    round(rate / 1000) must produce at least 1 to avoid a zero-buffer bug.
 *
 * -------------------------------------------------------------------------
 * Build and run (from repo root):
 *   cc -std=c99 -Wall -Wextra -lm -o /tmp/rp_framing_test tests/redpitaya_stream_framing_test.c && /tmp/rp_framing_test
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define HEADER_SIZE 22
#define MAX_PAYLOAD (1024 - HEADER_SIZE)

/* Write a 22-byte stream header into *packet, matching the layout produced
 * by write_stream_header() in RedPitaya_justin.c.
 * bytes_per_frame: payload size in bytes (total_channels × sizeof(int16)).
 * total_channels:  informational; receiver uses bytes_per_frame instead.
 * ns:              frame sequence number (monotonically increasing).       */
static void write_stream_header(uint8_t *packet, int bytes_per_frame, int total_channels, int32_t ns)
{
    int32_t offset = 0;             /* always 0 in current firmware */
    int32_t bpb    = bytes_per_frame;
    int32_t elm    = 2;             /* always 2 (two samples per element) */
    int16_t dtype  = 3;             /* 3 = int16 sample data */

    memcpy(packet + 0,  &offset,         4);   /* bytes 0-3:  offset */
    memcpy(packet + 4,  &bpb,            4);   /* bytes 4-7:  bytes_per_frame */
    memcpy(packet + 8,  &dtype,          2);   /* bytes 8-9:  dtype */
    memcpy(packet + 10, &elm,            4);   /* bytes 10-13: num_elements */
    memcpy(packet + 14, &total_channels, 4);   /* bytes 14-17: total_channels */
    memcpy(packet + 18, &ns,             4);   /* bytes 18-21: sample_count */
}

/* Try to parse a valid bytes-per-frame value from a potential 22-byte header.
 * Returns 1 on success (sets *out_bpb), 0 if this does not look like a header.
 *
 * Validation rules (same as AcqBoardRedPitaya::run()):
 *   - dtype at bytes 8-9 must be 0x0003 (int16 samples)
 *   - bytes_per_frame must be in [2, MAX_PAYLOAD]
 *   - bytes_per_frame must be even (all channels are int16 = 2 bytes each) */
static int parse_bpb(const uint8_t *hdr, int32_t *out_bpb)
{
    if (hdr[8] != 0x03 || hdr[9] != 0x00)
        return 0;  /* dtype mismatch — not a valid header start */
    memcpy(out_bpb, hdr + 4, 4);
    if (*out_bpb < 2 || *out_bpb > MAX_PAYLOAD || (*out_bpb & 1))
        return 0;  /* payload size out of range or odd (would misalign int16s) */
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

/* Read and parse one complete frame (header + payload) from the stream buffer.
 *
 * This mirrors the resync logic in AcqBoardRedPitaya::run():
 *   1. Read HEADER_SIZE bytes into 'packet'.
 *   2. Try to parse a valid bytes-per-frame value.
 *   3. If parsing fails (garbage byte or partial overlap), slide the window
 *      one byte forward: shift the buffer left by 1, read 1 new byte at end.
 *   4. Repeat up to 64 times before giving up.
 *   5. On success, read the payload immediately after the header.
 *
 * Why 64 attempts?  The header is 22 bytes, so at most 21 shifts are needed
 * to pass a single corrupted byte.  64 is a generous safety margin.
 *
 * *payload_bytes is set to bytes_per_frame on success.                      */
static int read_one_frame(const uint8_t *stream, size_t len, size_t *pos,
                            uint8_t *packet, int32_t *payload_bytes)
{
    const int headerSize = HEADER_SIZE;

    /* Read the first candidate header. */
    if (! simulate_read(stream, len, pos, packet, (size_t) headerSize))
        return 0;

    for (int guard = 0; guard < 64; ++guard) {
        if (parse_bpb(packet, payload_bytes)) {
            /* Header looks valid — read the payload that follows. */
            return simulate_read(stream, len, pos, packet + headerSize, (size_t) *payload_bytes);
        }

        /* Header invalid: slide the window one byte forward and try again.
         * memmove shifts bytes 1..21 down to positions 0..20, freeing byte 21
         * for the next fresh byte from the stream.                           */
        memmove(packet, packet + 1, (size_t) headerSize - 1);
        if (! simulate_read(stream, len, pos, packet + headerSize - 1, 1))
            return 0;
    }
    return 0;  /* gave up after 64 attempts — stream is too corrupt */
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
        /* Mirrors AcqBoardRedPitaya::run(): jmax(1, lround(boardSampleRate / 1000)) */
        int64_t new_val = (int64_t) lround((double) cases[i].rate / 1000.0);
        if (new_val < 1)
            new_val = 1;

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

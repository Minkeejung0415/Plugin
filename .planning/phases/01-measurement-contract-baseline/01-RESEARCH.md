# Phase 1 Research: Measurement Contract & Baseline

**Phase:** 01 - Measurement Contract & Baseline
**Date:** 2026-06-12
**Status:** Complete

## Research Questions

1. What does the current firmware already record per sample?
2. Where can acquisition samples be lost or stalled today?
3. What should the Phase 1 SD/file/counter contract require before implementation?
4. What ESP-IDF SD/FatFS guidance matters for this phase?

## Findings

### Current Firmware Sample Contract

`esp32/firmware/main/open_ephys_stream.h` defines:

```c
typedef struct {
    int64_t timestamp_us;
    uint32_t seq;
    int16_t ch[OE_STREAM_NUM_CHANNELS];
} oe_sample_t;
```

This is a good seed for SD ground truth because every sample already has:

- `seq`: monotonic frame identity
- `timestamp_us`: device-side acquisition timestamp
- `ch[]`: channel payload

The missing pieces are session metadata and health counters. A saved SD file cannot be self-describing yet because it lacks a header that records version, sample rate, channel count/layout, firmware mode, filter mode, SD logging mode, and stream mode.

### Current Loss/Stall Risks

`esp32/firmware/main/main.c` currently performs sample acquisition and then calls both:

- `open_ephys_stream_set_sample(&sample)`
- `sd_logger_append(&sample)`

`sd_logger_append()` currently does:

```c
fwrite(sample, sizeof(*sample), 1, s_fp);
fflush(s_fp);
```

That makes SD-card latency part of the acquisition loop. At higher sample rates this can create loop overruns or jitter even if the sensor read and filter work are fast enough.

`open_ephys_stream_set_sample()` stores only one global latest sample:

```c
s_latest = *sample;
```

The stream task sends one current sample per period. This is fine for live display, but it is not a lossless queue. If acquisition advances faster than the stream task sends, older samples can be overwritten before the host sees them.

`esp32/host/stress_test_serial.py` and `analyze_sample_rate.py` can detect host-visible duplicates/gaps today, but they do not yet compare host-visible output against an SD-saved ground-truth file or firmware counters.

### Required Phase 1 Contract

Phase 1 should produce an implementation contract requiring:

- SD file header/session record:
  - magic/version
  - record size
  - firmware build or mode string
  - sample rate
  - channel count
  - channel layout
  - filter enabled/disabled
  - stream enabled/disabled
  - SD logging mode
- Per-sample record:
  - `seq`
  - `timestamp_us`
  - channel payload
- Runtime counters:
  - generated samples
  - enqueued samples
  - saved samples
  - stream snapshots/sent samples if available
  - SD queue drops/overflows
  - acquisition-loop overruns
  - max acquisition-loop duration
  - max SD enqueue latency
  - max SD write latency
  - max SD queue depth
- Mode controls:
  - frequency
  - SD on/off
  - filter on/off
  - stream on/off
  - counters/status dump

### ESP-IDF SD/FatFS Guidance

Official ESP-IDF docs say FatFS can be used through VFS with standard C/POSIX file APIs. For SD cards, ESP-IDF provides `esp_vfs_fat_sdmmc_mount()`, `esp_vfs_fat_sdspi_mount()`, and `esp_vfs_fat_sdcard_unmount()` convenience helpers. The same docs warn those helpers provide limited error handling and production applications should inspect/adapt the source for more advanced behavior.

Official SDMMC docs point to `storage/sd_card/sdmmc` and `storage/sd_card/sdspi` examples for SD-card access through FatFS.

Implication for this project: Phase 2 should not only open `/sdcard/step_session.bin`; it should explicitly mount/probe the card, report mount failures, and keep acquisition timing decoupled from potentially variable card writes.

References:

- ESP-IDF FatFS support: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/storage/fatfs.html
- ESP-IDF SD/SDIO/MMC driver examples: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/storage/sdmmc.html
- ESP-IDF file-system considerations: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/file-system-considerations.html

## Recommended Plan Shape

Use a single Phase 1 plan that creates the baseline/contract artifact. It should be mostly documentation and deterministic code inspection, with optional tiny diagnostics only if needed. The buffered logger itself belongs to Phase 2.

## Research Complete

Phase 1 can proceed to planning.

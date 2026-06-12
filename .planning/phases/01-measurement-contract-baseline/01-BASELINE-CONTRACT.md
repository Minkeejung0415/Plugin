# Phase 1 Baseline Contract: SD Ground Truth and Acquisition Integrity

**Phase:** 01 - Measurement Contract & Baseline
**Date:** 2026-06-12
**Status:** Contract defined

## Purpose

This contract defines what evidence is required before the project can claim acquisition data is saved without sample loss. For v1.1, the SD card is the authoritative acquisition record. Open Ephys, USB/TCP, serial, and UDP paths are comparison paths only; they can reveal downstream stalls, but they cannot prove device-side continuity by themselves.

## Ground Truth

The SD-card recording is the ground-truth source for acquisition integrity.

Accepted "no samples lost" evidence requires all of the following:

- SD sequence continuity has no missing `seq` values.
- SD sequence continuity has no duplicate `seq` values.
- Device timestamps are monotonic enough for the requested sample rate, allowing only explained scheduling jitter.
- Firmware counters agree with the SD file: generated samples equal saved samples plus explicitly reported drops.
- Any dropped/enqueued/overflowed samples are reported by firmware counters, not inferred only from host-side timing.

Host-side observations are still useful:

- Open Ephys/exported data can be compared against SD data.
- USB/TCP/serial output can reveal transport stalls.
- UDP output can reveal visualization path loss.
- None of those host-side paths are authoritative for acquisition integrity.

## SD File Contract

Phase 2 should implement a self-describing SD file format. The exact binary layout can change during implementation, but it must satisfy this contract.

### Session Header

Each SD recording must start with a session/header record containing:

| Field | Purpose |
|-------|---------|
| `magic` | Identifies the file as a STEP acquisition log |
| `version` | Allows future file format changes |
| `record_size` | Lets analyzers validate alignment |
| `sample_rate_hz` | Requested acquisition frequency |
| `channel_count` | Number of int16 payload channels |
| `channel_layout` | Channel labels/order, or a layout ID that maps to documented labels |
| `firmware_mode` | Build/mode string when available |
| `filter_enabled` | Whether filter processing was enabled |
| `sd_logging_enabled` | Whether SD logging was enabled for the session |
| `stream_enabled` | Whether host/Open Ephys streaming was enabled |
| `start_timestamp_us` | Device timestamp at session start when available |

### Sample Record

Every saved acquisition sample must include:

| Field | Purpose |
|-------|---------|
| `seq` | Monotonic generated sample number |
| `timestamp_us` | Device-side acquisition timestamp |
| `ch[]` | Int16 channel payload in the documented channel layout |

The current `oe_sample_t` already contains the per-sample minimum:

```c
typedef struct {
    int64_t timestamp_us;
    uint32_t seq;
    int16_t ch[OE_STREAM_NUM_CHANNELS];
} oe_sample_t;
```

Phase 2 can reuse this payload, but the SD file still needs session metadata so the file is decodable without relying on the host process that created it.

### File Lifecycle

The SD logger should make file state unambiguous:

- Report SD mount/open success or failure at startup.
- Report if logging is disabled because the card is absent or the mount failed.
- Write complete records only; analyzers should be able to detect truncated tail records.
- Flush at controlled intervals or session boundaries, not by blocking every acquisition sample.

## Firmware Counter Contract

Firmware must expose counters that distinguish acquisition loss from downstream transport loss.

Required counters:

| Counter | Meaning |
|---------|---------|
| `generated_samples` | Samples produced by the acquisition loop |
| `enqueued_samples` | Samples accepted by the SD logging queue or buffer |
| `saved_samples` | Samples fully written to the SD file |
| `sd_queue_drops` | Samples rejected because the SD queue/buffer was full |
| `sd_write_errors` | Failed file writes or card errors |
| `acq_loop_overruns` | Acquisition iterations exceeding the requested period |
| `max_acq_loop_us` | Worst observed acquisition-loop duration |
| `max_sd_enqueue_us` | Worst observed time spent enqueueing from the acquisition loop |
| `max_sd_write_us` | Worst observed SD writer task write duration |
| `max_sd_queue_depth` | High-water mark of buffered-but-not-yet-written samples |

Useful comparison counters:

| Counter | Meaning |
|---------|---------|
| `stream_snapshots` | Number of samples copied into the live stream handoff |
| `stream_sent` | Number of frames sent to the host/Open Ephys path |
| `stream_send_errors` | Socket/serial send errors |
| `status_reports` | Number of counter/status dumps emitted during tests |

Comparison counters are not substitutes for SD counters. They classify downstream behavior after SD ground truth exists.

## Mode Matrix

Stress testing must be able to isolate layers. Phase 1 requires these controls to exist or be specified for Phase 2/3 implementation.

| Mode Control | Required Behavior | Why |
|--------------|-------------------|-----|
| Frequency | Set requested sample rate | Finds max safe acquisition rate |
| SD on/off | Enable or disable SD logging | Isolates SD write cost |
| Filter on/off | Enable or disable filter processing | Isolates CPU/filter cost |
| Stream on/off | Enable or disable host/Open Ephys streaming | Isolates downstream transport cost |
| Status dump | Print or send counters | Lets stress tools compare generated/saved/streamed counts |

Minimum test matrix:

| Test | SD | Filter | Stream | Purpose |
|------|----|--------|--------|---------|
| Acquisition baseline | Off | Off | Off | Sensor/read loop ceiling |
| Filter cost | Off | On | Off | Filter CPU ceiling |
| SD-only | On | Off | Off | Card/logging ceiling |
| SD + filter | On | On | Off | Device-side recording ceiling |
| Stream-only | Off | Off/On | On | Host transport/Open Ephys ceiling |
| Full path | On | On | On | Real session envelope |

If runtime commands do not exist yet, the contract still applies: Phase 2/3 must add commands or equivalent Kconfig-controlled test builds.

## Baseline Diagnosis From Current Code

### SD Logger Blocks the Acquisition Loop

Current file: `esp32/firmware/main/sd_logger.c`

Current behavior:

```c
fwrite(sample, sizeof(*sample), 1, s_fp);
fflush(s_fp);
```

Risk: every sample waits for file write and flush behavior. SD cards can have variable write latency, so this can create acquisition-loop jitter, missed periods, or stalls. Phase 2 should replace this with a buffered writer task and explicit counters.

### Open Ephys Stream Is Latest-Sample Only

Current file: `esp32/firmware/main/open_ephys_stream.c`

Current behavior:

```c
s_latest = *sample;
```

Risk: the stream task sends one current sample per period. If acquisition produces samples faster than the stream path sends them, intermediate samples can be overwritten before the host receives them. This is acceptable for a live display path, but it is not a lossless recording queue.

### Host Stress Tools Lack SD Ground Truth

Current files:

- `esp32/host/stress_test_serial.py`
- `esp32/host/analyze_sample_rate.py`

Current behavior: the tools detect host-visible duplicate sequences, sequence gaps, identical rows, and timestamp gaps. That is useful for transport diagnosis, but it cannot prove device-side losslessness until SD files and firmware counters are analyzed too.

## Verification Checklist For Future Phases

Before claiming "no samples lost":

1. SD file parses successfully and record alignment is valid.
2. SD `seq` values have no duplicates.
3. SD `seq` values have no missing values.
4. SD timestamps are monotonic and match the requested sample rate within the accepted tolerance.
5. Firmware reports `generated_samples == saved_samples + sd_queue_drops`.
6. Firmware reports `sd_queue_drops == 0` for the chosen no-loss operating rate.
7. Firmware reports `acq_loop_overruns == 0` or documents every overrun as accepted jitter.
8. Host/Open Ephys output is compared against SD ground truth, not treated as ground truth.
9. The recommended operating rate is below the first observed failure rate, with the safety margin documented.

## Phase Handoff

Phase 2 should implement the buffered/measured SD logger and counter surface defined here.

Phase 3 should teach `stress_test_serial.py` and `analyze_sample_rate.py` to consume the SD/counter evidence defined here.

Phase 4 should compare Open Ephys behavior against SD ground truth to classify downstream stalls.

Phase 5 should use this checklist during hardware UAT.

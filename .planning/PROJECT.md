# Open Ephys ESP32 Acquisition Reliability

## What This Is

This project is an Open Ephys acquisition-board plugin and ESP32-S3 acquisition stack for streaming IMU/filter data into Open Ephys while also supporting direct on-device recording. The current live path can move samples through ESP32 firmware, a serial/TCP bridge, the Open Ephys buffer, and optional downstream consumers. That path is useful for visualization, but it is not enough for experimental data integrity because host transport, Open Ephys buffering, or UDP-style side channels can stall or drop data.

The v1.1 milestone pivots from OpenSim HUD work to acquisition fundamentals: record samples directly to SD card on the acquisition device, prove whether any samples are lost, and quantify the maximum reliable sample rate with filter processing and SD logging enabled.

## Core Value

The operator can run acquisition without trusting a lossy host path: samples are saved on SD card with sequence/timestamp continuity, and stress-test reports identify the highest safe frequency plus the exact layer responsible for stalls or drops.

## Requirements

### Validated

- ESP32-S3 firmware exists under `esp32/firmware/main`.
- Firmware sample structure already carries `timestamp_us`, monotonic `seq`, and 8 int16 channels in `open_ephys_stream.h`.
- Acquisition loop currently calls `icm20948_read_scaled()`, updates DIO/camera verification fields, then calls both `open_ephys_stream_set_sample()` and `sd_logger_append()` in `main.c`.
- SD logger scaffold exists in `sd_logger.c`, currently opening `/sdcard/step_session.bin`.
- Host stress harness exists at `esp32/host/stress_test_serial.py`.
- Host analyzer exists at `esp32/host/analyze_sample_rate.py` and already detects duplicate sequence values, sequence gaps, row duplicates, and timestamp gaps.
- Open Ephys/plugin streaming path exists and can be tested separately from SD logging.
- Prior OpenSim/HUD work is set aside for this milestone.

### Active

- [ ] Implement SD-card recording as a measured, buffered acquisition sink rather than a synchronous `fwrite()`/`fflush()` inside the acquisition loop.
- [ ] Store enough metadata and per-sample counters to prove generated, saved, and streamed sample counts.
- [ ] Improve `stress_test_serial.py` so it can sweep frequency with filter on/off, SD logging on/off, and streaming path on/off.
- [ ] Report max sustainable acquisition rate, SD write latency, loop latency, buffer high-water mark, sequence gaps, duplicates, and stalls.
- [ ] Isolate whether observed stalls come from sensor hardware/I2C, filter CPU, SD card writes, USB/TCP/Open Ephys buffering, or UDP/host-side transport.

### Out of Scope

- OpenSim HUD/window-title/live-angle display fixes.
- New OpenSim IK features.
- Cosmetic Open Ephys editor changes unrelated to acquisition integrity.
- Treating UDP delivery as ground truth for recorded data.
- Claiming "no samples lost" without sequence-continuity evidence from the device-side saved file.

## Context

| Layer | Technology | Key files |
|-------|------------|-----------|
| ESP32 acquisition firmware | ESP-IDF / FreeRTOS | `esp32/firmware/main/main.c` |
| SD logging | C stdio scaffold today; needs buffered writer | `esp32/firmware/main/sd_logger.c`, `sd_logger.h` |
| Open Ephys stream | TCP/Open Ephys-compatible packet writer | `esp32/firmware/main/open_ephys_stream.c`, `open_ephys_stream.h` |
| Host stress test | Python + pyserial | `esp32/host/stress_test_serial.py` |
| Host analysis | Python CSV analyzer | `esp32/host/analyze_sample_rate.py` |
| Open Ephys plugin | C++/JUCE acquisition-board plugin | `acqboard.ccp`, `devicethread.cpp`, `device editor.cpp` |

## Observed Code Facts

- `sd_logger_append()` currently performs `fwrite(sample, sizeof(*sample), 1, s_fp)` followed by `fflush(s_fp)` for every sample. That directly couples SD-card latency into the acquisition loop.
- `open_ephys_stream_set_sample()` stores only the latest sample in `s_latest`. If stream timing lags the acquisition loop, intermediate samples are overwritten before the stream task can send them.
- `stream_task()` sends one current sample per period. It is a live display/transport path, not a lossless queue.
- `stress_test_serial.py` currently evaluates host-visible serial/binary output, but it does not yet compare against an SD-saved ground-truth file or collect firmware-side latency counters.

## Constraints

- Keep the system practical for the current ESP32-S3 hardware and Open Ephys plugin setup.
- Do not use host UDP or Open Ephys export as the only proof of sample integrity.
- Avoid blocking the acquisition loop on slow SD-card operations.
- Keep test artifacts machine-readable so repeated sweeps can be compared.
- Preserve the existing Open Ephys streaming path while adding SD-first reliability.

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| SD card is the ground-truth recording path for v1.1 | Host transport can stall/drop and Open Ephys buffering is not proof of acquisition integrity | Locked 2026-06-12 |
| Sequence numbers and timestamps are required on every saved sample | They make loss, duplication, and timing gaps measurable | Locked 2026-06-12 |
| SD logging must be buffered/measured | Current per-sample `fflush()` can create acquisition-loop stalls | Pending implementation |
| Stress testing must compare modes | Need to separate hardware/filter/SD limits from Open Ephys/transport limits | Pending implementation |
| OpenSim is paused | Reliability must be solved before visualization polish matters | Locked 2026-06-12 |

## Evolution

This document now represents milestone v1.1. Older v1.0 planning artifacts remain in `.planning/phases` and `.planning/milestones` for historical context only.

---
*Last updated: 2026-06-12 - v1.1 SD card reliability and lossless acquisition milestone*

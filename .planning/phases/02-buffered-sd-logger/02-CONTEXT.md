# Phase 2 Context: Buffered SD Logger

**Phase:** 02 - Buffered SD Logger
**Date:** 2026-06-12
**Status:** Discussed

## Goal

Replace the ESP-IDF firmware's per-sample synchronous SD writes with a buffered writer path that protects acquisition timing and exposes enough counters to prove whether samples were generated, accepted by the SD queue, saved, or dropped.

## Decisions

- Primary target is the ESP-IDF firmware under `esp32/firmware/main`.
- Normal SD recording is triggered from the Open Ephys plugin over the board command socket.
- The Open Ephys plugin's ESP32 RECORD button sends `RECORD ON` / `RECORD OFF`; it should not silently switch to PC-side CSV recording for normal operation.
- Stress testing may still trigger SD recording through test scripts, but that is the exception.
- The Arduino sketch remains a parallel implementation risk because current serial stress tooling can exercise it, but Phase 2 does not update that sketch.
- SD card latency must move out of the acquisition task.
- The acquisition task must attempt a non-blocking enqueue and continue on the next sample period.
- If the SD queue is full, the sample is dropped from SD logging and `sd_queue_drops` is incremented.
- The logger keeps the existing `/sdcard` mount assumption for now. Full ESP-IDF SDSPI mount/pin work is deferred until the board-specific wiring is confirmed.

## Current Failure Point

`esp32/firmware/main/sd_logger.c` currently performs:

```c
fwrite(sample, sizeof(*sample), 1, s_fp);
fflush(s_fp);
```

from `sd_logger_append()`, which is called directly inside `acquisition_task()` in `main.c`. That means SD card write and flush latency can stall acquisition.

## Phase 2 Acceptance

- `sd_logger_append()` no longer performs file I/O or flushes from the acquisition task.
- A writer task drains a FreeRTOS queue and batches/periodically flushes writes.
- `RECORD ON` starts SD recording and `RECORD OFF` flushes/closes it.
- The SD file starts with a small session header before sample records.
- Startup logs clearly state whether SD logging is enabled, disabled, open, or failed.
- Firmware exposes counters for generated samples, enqueued samples, saved samples, queue drops, write errors, max queue depth, max enqueue latency, max write latency, acquisition-loop overruns, and max acquisition-loop duration.
- Acquisition timing counters are measured around the loop in `main.c`.

## Deferred

- Runtime serial commands for SD/filter/stream mode switching belong to Phase 3 unless needed to compile Phase 2.
- Full SD mount implementation with board-specific SDSPI pins is deferred; Phase 2 records mount/open failure explicitly instead of guessing pins.
- Open Ephys stream queueing remains out of scope until Phase 4.

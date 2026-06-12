---
gsd_state_version: 1.0
milestone: v1.1
milestone_name: SD Card Reliability and Lossless Acquisition
current_phase: 02
status: ready_to_plan
last_updated: 2026-06-12T22:03:54.834Z
progress:
  total_phases: 5
  completed_phases: 1
  total_plans: 1
  completed_plans: 1
  percent: 20
stopped_at: Phase 01 complete (1/1) — ready to discuss Phase 2
---

# Project State

**Project:** Open Ephys ESP32 Acquisition Reliability
**Current phase:** 2
**Status:** Ready to plan

## Progress

| Phase | Name | Status | Plans |
|-------|------|--------|-------|
| 1 | Measurement Contract & Baseline | Planned | 0 |
| 2 | Buffered SD Logger | Planned | 0 |
| 3 | Stress Harness & Analyzer | Planned | 0 |
| 4 | Open Ephys Stall Isolation | Planned | 0 |
| 5 | Hardware UAT & Operator Docs | Planned | 0 |

**Overall:** 0/5 phases complete. Next step is to plan Phase 1.

## Current Focus

OpenSim and HUD work are paused. The active goal is to make acquisition data reliable without relying on host transport:

- Save samples directly to SD card.
- Prove continuity with sequence and timestamp checks.
- Improve stress tests to find the maximum safe frequency with filter and SD logging enabled.
- Diagnose whether stalls come from hardware/sensor reads, filter CPU, SD writes, USB/TCP/Open Ephys buffering, or UDP/host-side paths.

## Code Facts From Milestone Start

- `esp32/firmware/main/sd_logger.c` currently writes and flushes every sample, which can stall acquisition.
- `esp32/firmware/main/open_ephys_stream.c` currently keeps only `s_latest`, so the live stream can miss intermediate samples if the stream task lags.
- `esp32/host/stress_test_serial.py` already sweeps rates and detects host-visible gaps/duplicates, but it does not yet validate SD ground truth.
- `esp32/host/analyze_sample_rate.py` already provides a foundation for sequence/timestamp analysis.

## Pending Manual Inputs

- Confirm the exact ESP32 board/SD hardware wiring if Phase 2 needs pin/mount changes.
- Run hardware stress tests once firmware and host tooling are ready.

## Active Risks

- Per-sample SD flush latency may be the current acquisition-loop stall source.
- Open Ephys stalls may be downstream even when device-side SD continuity is clean.
- USB/TCP/UDP observations cannot substitute for SD continuity proof.
- SD-card behavior depends on card model and filesystem state.

---
*State updated: 2026-06-12 - v1.1 milestone started*

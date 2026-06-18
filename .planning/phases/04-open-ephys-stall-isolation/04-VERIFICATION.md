---
phase: 04-open-ephys-stall-isolation
status: human_needed
verified: 2026-06-18
source:
  - 04-01-SUMMARY.md
  - 04-02-SUMMARY.md
---

# Phase 04 Verification: Open Ephys Stall Isolation

## Status

`human_needed`

Firmware, plugin, parser, and verdict automation pass. Device-side SD continuity under normal and stalled UDP receivers still requires the real ESP32, SD card, network, and Open Ephys GUI.

## Automated Evidence

| Must-have | Status | Evidence |
|-----------|--------|----------|
| Acquisition performs no blocking stream write | Passed | Static scan; only non-blocking queue offers remain in `loop()`. |
| SD is offered before live streaming | Passed | Firmware acquisition order is `logSd()` then `queueStreamRecord()`. |
| SD writer outranks UDP writer | Passed | Both use core 0; priorities are SD 4 and stream 1. |
| Congestion drops live packets only | Passed | Bounded queue drops its oldest stream item and updates stream counters. |
| TCP control is separate from UDP samples | Passed | Capability handshake advertises UDP; plugin binds port 55001 while retaining TCP control. |
| Malformed UDP packets do not desynchronize parsing | Passed | Each datagram is size/header/channel validated independently. |
| SD and stream verdicts are separate | Passed | Six focused stress-isolation tests pass. |
| Firmware and plugin compile | Passed | Arduino XIAO ESP32S3 and acquisition-board Debug builds pass. |
| Real stalled receiver preserves SD | Pending | Hardware UAT required. |
| Magnetometer contract works end to end | Pending | Hardware/Open Ephys UAT required. |

## Required Hardware Proof

1. Run stream-off baselines at 950 and 1000 Hz.
2. Run the same rates with Open Ephys receiving UDP 55001.
3. Stall or disconnect the receiver during a third run.
4. Finalize and analyze every SD file for sequence gaps, duplicates, timestamp gaps, and count mismatch.
5. Accept the operating point only when SD is perfect and UDP-on SD rate is at least 97% of its stream-off baseline. UDP drops are observations, not SD failures.

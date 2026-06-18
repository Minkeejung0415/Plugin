---
phase: 04-open-ephys-stall-isolation
plan: "02"
subsystem: esp32-sd-first-udp-streaming
tags: [esp32, open-ephys, udp, sd, freertos, stress-test]
requirements-completed: [LOSS-03, LOSS-04, STALL-03]
key-files:
  - esp32/arduino/step_node/step_node.ino
  - acqboard.ccp
  - Acqboardredpitaya.h
  - esp32/host/stress_test_serial.py
  - tests/test_esp32_stream_isolation.py
metrics:
  automated_checks_passed: 6
  human_checks_pending: 5
completed: 2026-06-18
---

# Plan 04-02 Summary: Isolate UDP Streaming From SD Acquisition

## What Changed

Moved live Open Ephys sample delivery out of the core-1 acquisition loop. Acquisition now offers each sample to SD first, then makes a non-blocking offer to a bounded live-stream queue. The core-0 SD writer runs at priority 4 while the UDP sender runs at priority 1. Congestion discards old live packets and increments stream-only counters; it cannot block SD persistence.

TCP remains responsible for commands and rec-v1 retrieval. UDP port 55001 carries complete, independently validated 14-channel sample datagrams to the Open Ephys plugin. Legacy TCP sample firmware remains supported by capability negotiation.

The stress harness now reports SD integrity separately from live-stream quality. A degraded UDP view can pass when finalized SD counters and rate are clean, while SD errors or disagreement still fail the run.

## Commit

| Task | Commit | Description |
|------|--------|-------------|
| 04-02 | `d95e70b` | Isolate lossy UDP streaming from SD acquisition. |

## Verification

Passed:

- XIAO ESP32S3 Arduino firmware compile: 996,344 bytes flash, 47,708 bytes globals.
- Open Ephys acquisition-board Debug plugin build produced `acquisition-board.dll`.
- `python tests\test_esp32_stream_isolation.py` (6 tests).
- `python tests\test_serial_tcp_bridge.py`.
- Python syntax compilation for the modified host harness and tests.
- Static acquisition-loop scan found no TCP, serial, or UDP transport write in `loop()`.

## Human Verification Pending

1. Flash the direct Wi-Fi UDP profile with the ICM on HSPI pins D3-D6.
2. Confirm Open Ephys negotiates `transport=udp`, receives port 55001 samples, and shows 14 channels.
3. Run 60-second filter+SD+UDP tests at 950 and 1000 Hz and analyze finalized SD files.
4. Stall or disconnect the UDP receiver and prove SD remains continuous and within 3% of baseline.
5. Confirm `mx`, `my`, and `mz` are present and responsive in Open Ephys and saved data.

## Deviations

- Added `--stream-transport udp` because UDP samples are intentionally absent from the serial command channel in the production profile.
- Activated the direct Wi-Fi profile so the new UDP path is the firmware-advertised production path.
- Hardware/Open Ephys verification was not run because it requires the connected device, SD card, LAN receiver, and GUI session.

## Self-Check

Automated implementation and verification passed. Phase 4 remains `human_needed` until the SD and receiver-stall bench checks pass.

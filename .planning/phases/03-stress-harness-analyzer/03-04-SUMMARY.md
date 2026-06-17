---
phase: 03-stress-harness-analyzer
plan: "04"
subsystem: esp32-sd-spi-reliability
tags: [sd-card, spi-bus, icm20948, stress-harness, lossless-acquisition]
dependency_graph:
  requires: [03-03]
  provides: [split-sd-counters, shared-spi-lock, batched-sd-writes]
  affects:
    - esp32/arduino/step_node/step_node.ino
    - esp32/host/stress_test_serial.py
    - esp32/docs/stress-test-sample-rate.md
metrics:
  compile: passed
  flash: blocked-no-serial-port
  hardware_smoke: blocked-no-serial-port
---

# Phase 03 Plan 04: SD-On SPI Contention Diagnosis

## Tasks Completed

| Task | Name | Status | Files |
|------|------|--------|-------|
| 1 | Split SD error counters | Done | `step_node.ino`, `stress_test_serial.py` |
| 2 | Add shared SPI bus mutex | Done | `step_node.ino` |
| 3 | Batch SD queue records | Done | `step_node.ino` |
| 4 | Document before/after diagnosis | Partial | `stress-test-sample-rate.md` |

## What Changed

The overloaded `sd_err` path was split into explicit counters:

- `queue_drops`
- `write_errors`
- `header_errors`
- `open_errors`
- `begin_errors`
- `mutex_timeouts`
- `max_queue_depth`
- `flush_count`
- `max_flush_us`
- `spi_mutex_timeouts`

The firmware now preserves the legacy `errors=` field as the total of the failure counters while also reporting the individual causes in `STATUS`, `SD_FINAL`, and `SD_STATUS`.

The ICM and SD paths now share one SPI bus mutex. Low-level ICM SPI reads/writes take this lock, and SD open/write/flush/close/checksum/read operations use the same lock. This tests the current leading hypothesis: today's SD-on regression appeared after moving ICM from I2C to the same SPI bus used by SD.

The SD writer task now batches up to 32 `SdLogRecord` entries into one contiguous `File.write(...)` call. The on-card format is unchanged because the batch is just consecutive records after the same header.

## Verification

Automated checks completed:

```text
python -m py_compile esp32\host\stress_test_serial.py
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 esp32\arduino\step_node
```

Both passed.

Hardware verification is blocked because Windows currently reports no serial ports:

```text
arduino-cli upload -p COM4 ... -> Could not open COM4, port busy or doesn't exist
python esp32\host\stress_test_serial.py --list-ports -> No serial ports found
```

## Next Hardware Step

Reconnect the ESP32 and run:

```powershell
arduino-cli upload -p COM4 --fqbn esp32:esp32:XIAO_ESP32S3 esp32\arduino\step_node
python esp32\host\stress_test_serial.py --port COM4 --hz 100 200 300 400 500 --capture-s 60 --filter on --sd on
```

Pass criteria for the SD-on proof:

- `queue_drops=0`
- `write_errors=0`
- `mutex_timeouts=0`
- `spi_mutex_timeouts=0`
- `dup_seq=0`
- `gap_seq=0`
- SD binary analyzer confirms continuous records at the chosen cap

## Self-Check: PARTIAL

Code and compile verification passed. Flash and SD-on hardware proof remain pending because the ESP32 serial port is not currently visible to the host.

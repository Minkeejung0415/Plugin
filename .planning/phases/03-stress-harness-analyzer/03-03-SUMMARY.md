---
phase: 03-stress-harness-analyzer
plan: "03"
subsystem: esp32-spi-stress
tags: [icm20948, spi, odr, no-sd-stress, duplicate-detection]
dependency_graph:
  requires: [03-02]
  provides: [verified-spi-pin-map, explicit-icm-odr-config, no-sd-spi-ceiling]
  affects:
    - esp32/arduino/step_node/step_node.ino
    - esp32/firmware/main/icm20948.h
    - esp32/docs/stress-test-sample-rate.md
    - .planning/phases/03-stress-harness-analyzer/03-03-PLAN.md
tech_stack:
  added: []
  patterns:
    - explicit ICM20948 bank-2 ODR/divider configuration
    - no-SD boundary sweeps split by filter mode
key_files:
  modified:
    - esp32/arduino/step_node/step_node.ino
    - esp32/firmware/main/icm20948.h
    - esp32/docs/stress-test-sample-rate.md
  added:
    - .planning/phases/03-stress-harness-analyzer/03-03-PLAN.md
    - .planning/phases/03-stress-harness-analyzer/03-03-SUMMARY.md
decisions:
  - "Use verified XIAO D7 / GPIO44 as the ICM20948 SPI CS pin."
  - "Keep no-SD characterization separate from SD lossless acquisition claims."
  - "Configure ICM gyro_div=0, accel_div=0, odr_align=1, dlpf=off for high-rate stress characterization."
metrics:
  completed: "2026-06-16"
  raw_no_sd_highest_pass_hz: 1400
  raw_no_sd_recommended_cap_hz: 1120
  filter_no_sd_highest_pass_hz: 1000
  filter_no_sd_recommended_cap_hz: 800
---

# Phase 03 Plan 03: ICM SPI ODR Tuning and No-SD Ceiling

## Tasks Completed

| Task | Name | Status | Files |
|------|------|--------|-------|
| 1 | Lock verified SPI wiring in both firmware paths | Done | `step_node.ino`, `icm20948.h` |
| 2 | Add explicit ICM ODR/DLPF configuration | Done | `step_node.ino` |
| 3 | Record no-SD SPI ceiling as stress artifact | Done | `stress-test-sample-rate.md` |

## What Was Built

The Arduino firmware now uses the verified ICM20948 SPI CS pin, XIAO D7 / GPIO44, and prints the active ICM ODR configuration at boot. The ESP-IDF header was kept consistent with the same CS pin.

The ICM init path now explicitly configures bank-2 output-rate settings:

- `gyro_div=0`
- `accel_div=0`
- `odr_align=1`
- `dlpf=off`
- `dlpf_cfg=0`

The stress-test docs now include a no-SD SPI characterization section with the exact raw and filter-on sweep commands.

## Hardware Verification

Flashed to `COM4` with `arduino-cli upload`.

Boot check:

```text
ICM CS: GPIO44 (D7)
ICM ODR: gyro_div=0 accel_div=0 odr_align=1 dlpf=off dlpf_cfg=0
CS GPIO44 -> WHO_AM_I 0xEA OK
ICM20948: OK on SPI CS GPIO44 WHO_AM_I=0xEA
ICM20948: ODR gyro_div=0 accel_div=0 odr_align=1 dlpf=off cfg=0
AK09916: OK through ICM SPI aux bus, continuous 100 Hz
```

Raw no-SD sweep after ODR setup:

| Hz | mean_hz | dup_seq | gap_seq | overrun | Result |
|----|---------|---------|---------|---------|--------|
| 1300 | 1297.0 | 0 | 0 | 3 | PASS |
| 1400 | 1396.6 | 0 | 0 | 4 | PASS |
| 1500 | 1496.9 | 3 | 0 | 4 | FAIL |
| 1600 | 1594.7 | 84 | 0 | 6 | FAIL |
| 1800 | 1794.5 | 2319 | 0 | 8 | FAIL |
| 2000 | 1991.0 | 5066 | 0 | 21 | FAIL |

Filter-on no-SD sweep after ODR setup:

| Hz | mean_hz | dup_seq | gap_seq | Result |
|----|---------|---------|---------|--------|
| 800 | 798.6 | 0 | 0 | PASS |
| 900 | 897.1 | 0 | 0 | PASS |
| 950 | 944.1 | 0 | 0 | PASS |
| 1000 | 963.1 | 0 | 0 | PASS |
| 1050 | 972.6 | 0 | 0 | FAIL |

## Interpretation

Explicit ODR setup reduced raw-mode duplicates at 1500 Hz compared with the earlier SPI run, but it did not move the pass boundary. The current no-SD envelope remains:

- raw/filter off: highest pass 1400 Hz, recommended cap 1120 Hz
- VQF/filter on: highest pass 1000 Hz, recommended cap 800 Hz

This does not prove SD reliability. SD was intentionally absent and still needs a later SD-on run plus SD-file continuity analysis.

## Self-Check: PASSED

- `arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 esp32\arduino\step_node` passed.
- `python -m py_compile esp32\host\stress_test_serial.py` passed.
- Firmware flashed to `COM4`.
- Boot diagnostics confirmed real ICM SPI and mag availability.
- Both planned no-SD sweeps completed.

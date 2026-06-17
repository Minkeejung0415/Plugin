---
phase: 04-open-ephys-stall-isolation
plan: "01"
subsystem: esp32-open-ephys-channel-contract
tags: [esp32, open-ephys, magnetometer, usb-bridge, csv]
key-files:
  - Acqboardredpitaya.h
  - acqboard.ccp
  - esp32/host/serial_tcp_bridge.py
  - esp32/arduino/step_node/step_node.ino
  - tests/test_serial_tcp_bridge.py
  - esp32/docs/open-ephys-plugin.md
  - esp32/docs/local-open-ephys-setup.md
  - esp32/docs/arduino-ide-guide.md
metrics:
  tests_added: 1
  automated_checks_passed: 3
  human_checks_pending: 4
---

# Plan 04-01 Summary: Restore ESP32 14-Channel Magnetometer Contract

## What Changed

Aligned the current ESP32 STEP channel contract across plugin fallback defaults, USB bridge defaults,
CSV bridge packing, firmware serial banners, and operator docs.

The current firmware contract is now consistently represented as:

`ax, ay, az, gx, gy, gz, mx, my, mz, qw, qx, qy, qz, dio`

## Commits

| Task | Commit | Description |
|------|--------|-------------|
| 04-01 | Not committed | Local working-tree execution only; no commit requested in this run. |

## Changes By File

| File | Change |
|------|--------|
| `Acqboardredpitaya.h` | Updated `ESP32_DEFAULT_CHANNELS` from 13 to 14 and documented `dio` as channel 13. |
| `acqboard.ccp` | Updated stale ESP32 handshake comments from 8-channel examples to 14-channel / `N` channel examples. |
| `esp32/host/serial_tcp_bridge.py` | Updated default `ESP32_NUM_CHANNELS` fallback from 11 to 14; made `pack_csv_row()` preserve configurable channel counts instead of hardcoding 8. |
| `esp32/arduino/step_node/step_node.ino` | Updated serial CSV/channel banners to include `dio`. |
| `tests/test_serial_tcp_bridge.py` | Added a no-dependency Python test for 14-channel CSV packing and legacy 8-channel override. |
| `esp32/docs/open-ephys-plugin.md` | Updated the primary ESP32 channel map and handshake/header expectations to 14 channels. |
| `esp32/docs/local-open-ephys-setup.md` | Updated USB/plugin setup expectations to 14 channels. |
| `esp32/docs/arduino-ide-guide.md` | Updated USB/Open Ephys guide, CSV header, handshake, payload size, and channel map to 14 channels. |

## Verification

Automated checks run:

- `python tests\test_serial_tcp_bridge.py` - passed
- `python -m py_compile esp32\host\serial_tcp_bridge.py tests\test_serial_tcp_bridge.py` - passed
- Static channel-contract search confirmed:
  - `NUM_CHANNELS 14`
  - `ESP32_DEFAULT_CHANNELS = 14`
  - `DEFAULT_NUM_CHANNELS` fallback `"14"`
  - firmware serial text includes `mx,my,mz,qw,qx,qy,qz,dio`
  - docs now describe 14-channel Open Ephys/plugin expectations in the main setup paths

`python -m pytest tests\test_serial_tcp_bridge.py -q` could not run because `pytest` is not installed in this Python environment. The same assertions were executed directly with `python tests\test_serial_tcp_bridge.py`.

## Human Verification Pending

The hardware/Open Ephys checks still need to be run:

1. Start the same bridge/path used in the failing run.
2. Confirm the handshake says `14 channels`.
3. Confirm Open Ephys shows `mx`, `my`, and `mz`.
4. Record/export or retrieve a short CSV and confirm it includes `mx,my,mz`.
5. Move the IMU or bring a magnet nearby and confirm magnetometer values change.

## Deviations

No source-scope deviations. The plan's Open Ephys bench verification was not run because it requires the device and GUI session.

## Self-Check

PASSED for automated implementation and static verification.

Human verification remains required before Phase 4 can be marked complete.


---
phase: 04-open-ephys-stall-isolation
status: human_needed
verified: 2026-06-17
source:
  - 04-01-PLAN.md
  - 04-01-SUMMARY.md
---

# Phase 04 Verification: Open Ephys Stall Isolation

## Status

`human_needed`

The automated portion of Plan 04-01 is complete, but Phase 4 cannot be marked passed until the
Open Ephys hardware/GUI checks prove the magnetometer channels appear in the real acquisition
and saved/exported CSV path.

## Automated Checks

Passed:

- `python tests\test_serial_tcp_bridge.py`
- `python -m py_compile esp32\host\serial_tcp_bridge.py tests\test_serial_tcp_bridge.py`
- Static channel-contract search for 14-channel firmware/plugin/bridge/doc alignment

Not available:

- `python -m pytest tests\test_serial_tcp_bridge.py -q` because `pytest` is not installed.

## Must-Haves

| Must-have | Status | Evidence |
|-----------|--------|----------|
| Plugin fallback exposes current 14-channel ESP32 contract | Passed | `Acqboardredpitaya.h` now sets `ESP32_DEFAULT_CHANNELS = 14`. |
| USB bridge defaults to current 14-channel contract | Passed | `serial_tcp_bridge.py` defaults `ESP32_NUM_CHANNELS` to 14. |
| CSV bridge preserves magnetometer slots | Passed | `tests/test_serial_tcp_bridge.py` verifies channels 6-8 survive as `mx,my,mz`. |
| Firmware serial text describes all 14 columns | Passed | `step_node.ino` banners include `mx,my,mz,qw,qx,qy,qz,dio`. |
| Operator docs describe current 14-channel layout | Passed | Main Open Ephys/plugin setup docs now reference 14 channels. |
| Open Ephys shows `mx`, `my`, `mz` | Pending human test | Requires GUI/device run. |
| Saved/exported CSV includes `mx,my,mz` | Pending human test | Requires recording/export or retrieval run. |
| Magnetometer values change under magnetic-field movement | Pending human test | Requires hardware movement/magnet test. |

## Human Verification

Run:

1. Flash or use current STEP firmware (`NUM_CHANNELS 14`).
2. Start the USB plugin bridge or direct Wi-Fi path used in the failing session.
3. Confirm the handshake reports `14 channels`.
4. In Open Ephys, confirm channels include `mx`, `my`, and `mz`.
5. Record a short session and export/retrieve CSV.
6. Confirm CSV columns or values include `mx,my,mz`.
7. Move the IMU or use a nearby magnet to confirm magnetometer readings are not flat zeros.

Expected:

- Open Ephys channel list includes `mx`, `my`, and `mz`.
- CSV output includes `mx,my,mz`.
- If `mx,my,mz` are present but flat, inspect firmware boot/status for `mag=FALLBACK` or `mag_ok=0`.


# Phase 4: Open Ephys Stall Isolation - Magnetometer Channel Research

**Researched:** 2026-06-17
**Domain:** ESP32/Open Ephys channel contract, USB bridge, plugin CSV/export path
**Confidence:** HIGH for static code facts; hardware behavior still needs a bench run.

---

## Summary

The immediate magnetometer symptom is consistent with a channel-contract drift between
firmware, bridge defaults, plugin defaults, and CSV conversion paths.

The current Arduino STEP node declares `NUM_CHANNELS 14` and packs:

1. `ax`, `ay`, `az`
2. `gx`, `gy`, `gz`
3. `mx`, `my`, `mz`
4. `qw`, `qx`, `qy`, `qz`
5. `dio`

Open Ephys naming in `devicethread.cpp` already has a 14-name ESP32 map including
`mx`, `my`, `mz`, and `dio`. The likely failure points are the older host/plugin
defaults that still advertise or fall back to 11 or 13 channels, plus the USB bridge
CSV parser that currently packs only 8 CSV values.

---

## Evidence

### Firmware Emits 14 Channels

- `esp32/arduino/step_node/step_node.ino` defines `NUM_CHANNELS 14`.
- `packChannelsFromImu()` assigns magnetometer values to channels 6, 7, and 8.
- Quaternion values are assigned to channels 9-12.
- DIO is assigned to channel 13.
- The Open Ephys binary header uses `NUM_CHANNELS` for `num_channels` and
  `NUM_CHANNELS * 2` for payload bytes.
- The firmware `REDPITAYA` handshake sends both:
  - `{NUM_CHANNELS} channels; sample_rate=...`
  - `OK CHANNELS:{NUM_CHANNELS}`

### Open Ephys Channel Names Can Represent 14 Channels

- `devicethread.cpp` has `esp32Names[]` with:
  `ax, ay, az, gx, gy, gz, mx, my, mz, qw, qx, qy, qz, dio`.
- This means if the acquisition board reports 14 ADC outputs in ESP32 mode, Open Ephys
  should expose magnetometer channels by name.

### Plugin Default Is Stale

- `Acqboardredpitaya.h` documents `ESP32_DEFAULT_CHANNELS = 13` and sets the constant to 13.
- This default is used when parsing an ESP32 handshake fails to extract a channel count.
- If the bridge/firmware handshake is incomplete, truncated, or stale, the plugin can expose
  13 channels instead of the firmware's current 14-channel contract.
- That does not remove `mx/my/mz` by itself, but it proves the host contract is not aligned
  with the active firmware and can mask or shift later channel expectations.

### USB Bridge Defaults Are Stale

- `esp32/host/serial_tcp_bridge.py` defaults `ESP32_NUM_CHANNELS` to 11.
- In plugin mode, the bridge advertises this `num_ch` during handshake.
- If running through the USB bridge without setting `ESP32_NUM_CHANNELS=14`, the plugin can
  expose the wrong channel count even though the serial binary frame may carry 14 channels.

### USB Bridge CSV Mode Drops Magnetometer/Quaternion/DIO Data

- `serial_tcp_bridge.py::pack_csv_row()` currently requires 9 fields and packs only 8 int16
  channels into the Open Ephys frame.
- If the firmware is in text CSV mode, this bridge path truncates the row to channels 1-8
  after `seq`. That preserves only `ax, ay, az, gx, gy, gz, mx, my` and drops `mz`,
  all quaternions, and `dio`.
- The plugin path normally expects binary mode, but this is still a real CSV-path bug for
  anyone using `--csv`.

### SD Download CSV Already Knows the 14-Channel Names

- `esp32/host/rec_download.py` has `CHANNEL_NAMES` including `mx`, `my`, `mz`, quaternion
  channels, and `dio`.
- If the SD log header reports 14 channels, this converter should write magnetometer columns.
- If the plugin-retrieved CSV lacks magnetometer columns, the missing piece is likely before
  or around retrieval/conversion rather than this static name list.

---

## Planning Implications

Phase 4 should not immediately patch code by assumption. It should first make the channel
contract observable at every boundary:

- Firmware boot/handshake says 14 channels.
- USB bridge advertises 14 channels.
- Open Ephys plugin reports 14 ADC outputs and names `mx/my/mz`.
- Live binary frame headers report 14 channels and 28 payload bytes.
- SD log header reports 14 channels.
- Retrieved/exported CSV headers include `mx,my,mz`.

Then the implementation can align stale defaults and CSV packing with the verified contract.

---

## Candidate Fix Scope

Likely files:

- `Acqboardredpitaya.h`
  - update ESP32 default channel contract from 13 to 14.
- `acqboard.ccp`
  - update stale ESP32 handshake comments and any fallback/status text.
  - optionally add debug/status logging for parsed ESP32 channel count.
- `esp32/host/serial_tcp_bridge.py`
  - update default `ESP32_NUM_CHANNELS` from 11 to 14.
  - update `pack_csv_row()` to pack all available channel values, up to the configured count.
- `esp32/arduino/step_node/step_node.ino`
  - update stale serial banner strings that omit `dio`.
- `esp32/docs/*`
  - update operator docs that still say 8 or 11 channels for current STEP firmware.

---

## Verification Needs

Automated:

- Unit or script-level check that CSV rows with 14 channel values become Open Ephys frames
  with `n_channels=14` and `num_bytes=28`.
- Static check that plugin and bridge defaults agree on 14.
- Static check that docs and boot banners describe the same 14-channel order.

Bench/manual:

- Run USB bridge in plugin mode and confirm handshake says 14 channels.
- In Open Ephys, confirm channels include `mx`, `my`, and `mz`.
- Record a short session and confirm exported/plugin-retrieved CSV includes `mx,my,mz`.
- Rotate or move the IMU near a magnet and confirm magnetometer values change from baseline.


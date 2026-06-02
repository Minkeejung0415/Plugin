# ESP32-S3 ↔ Open Ephys AcqBoard plugin — integration notes

This document captures the debugging and changes made to get the **ESP32-S3 STEP
node** streaming into the Open Ephys **Acquisition Board** plugin
(`AcqBoardRedPitaya`), and forwarding orientation to **OpenSim**.

It covers three layers that all had to agree:

1. **Plugin** (this repo): `acqboard.ccp` / `Acqboardredpitaya.h` / `devicethread.cpp`
2. **USB serial→TCP bridge** (`Minkeejung0415/ESP32-S3`): `host/serial_tcp_bridge.py`,
   `host/run_usb_plugin_bridge.ps1`
3. **ESP32-S3 firmware** (`Minkeejung0415/ESP32-S3`): the `.ino` sketch

---

## 1. Architecture / data path

```
ICM20948 (acc+gyro, I2C)
      │
   ESP32-S3 firmware (.ino)
      │  Open Ephys binary frames: [22-byte OeHeader][8 × int16]
      │
      ├── Wi-Fi mode: TCP :5000 on the board  ──────────────┐
      │                                                      │
      └── USB mode (default, USB_OPEN_EPHYS_MODE=true):      │
            USB serial (binary) → serial_tcp_bridge.py       │
                                  TCP 127.0.0.1:5000  ───────┤
                                                             ▼
                                            Open Ephys AcqBoard plugin
                                            (AcqBoardRedPitaya, "RedPitaya" board type)
                                                  │
                                  ┌───────────────┴───────────────┐
                                  ▼                               ▼
                        Open Ephys data stream         OpenSim UDP v2 (127.0.0.1:5000)
                        (8 ADC channels)               quaternion packets (when enabled)
```

The ESP32 path **reuses the Red Pitaya board class** (`AcqBoardRedPitaya`); an
`isEsp32Node` flag selects ESP32-specific behavior (TCP binary stream, fixed
8 ch @ 100 Hz) versus the Red Pitaya behavior (UDP :55001 stream).

### Wire formats

**Handshake (TCP, ASCII, line-delimited):**

| Direction | Message |
|---|---|
| plugin → node | `REDPITAYA\n` |
| node → plugin | `8 channels; sample_rate=100; node=esp32s3_arduino\n` then `OK CHANNELS:8\n` |
| plugin → node | `START\n` |
| node → plugin | `STARTED BIN:...\n` then `SENSORS:0,ICM20948\n` |
| node → plugin | *(binary frames follow immediately)* |

**Binary frame (`OeHeader`, packed, 22 bytes + payload):**

| Offset | Size | Field | Value |
|---|---|---|---|
| 0  | 4 | `offset` | 0 |
| 4  | 4 | `num_bytes` | 16 (= 8 ch × int16) |
| 8  | 2 | `bit_depth` | 3 (OpenCV S16 enum) |
| 10 | 4 | `element_size` | 2 |
| 14 | 4 | `num_channels` | 8 |
| 18 | 4 | `samples_per_channel` | 1 |
| 22 | 16 | payload | 8 × int16, little-endian |

The plugin's TCP parser validates `element_size == 2` at offset 10 and
`num_bytes` at offset 4, then reads `num_bytes/2` int16 channels. Full frame =
**38 bytes**.

**8-channel layout:**

| ch | meaning | scaling in plugin |
|---|---|---|
| 0 | AccX | ÷ 16384 → g (±2 g preset) |
| 1 | AccY | ÷ 16384 |
| 2 | AccZ | ÷ 16384 |
| 3 | GyrX | ÷ 131.072 → °/s (±250 °/s preset) |
| 4 | GyrY | ÷ 131.072 |
| 5 | GyrZ | ÷ 131.072 |
| 6 | DIO | bit0 = level |
| 7 | Reserved | 0 |

---

## 2. Problems found and fixed

### Pre-existing (before this work)
- **`open-ephys.exe` entry-point / mixed Debug-Release link error** — the JUCE
  `this_will_fail_to_link_if_some_of_your_compile_units_are_built_in_release_mode`
  trap. Cause: the plugin DLL and the host were built in different
  configurations. Fixed by the user with a clean rebuild in a single
  configuration (delete the CMake `Build/` cache, reconfigure, rebuild Debug).

### Fix 1 — ESP32 misdetected as Red Pitaya (commit `fbdd4ab`)
**Symptom:** plugin "detected a board" but no data; sometimes a hard start failure.

**Root cause:** in `performDetectionHandshake()` the generic Red Pitaya check
`response.contains("OK")` ran **before** the ESP32 marker check. Both the
firmware and the bridge reply with two lines, the second being `OK CHANNELS:8`.
Over a localhost/TCP connection the two writes coalesce into one `read()`, so
`OK` matched first → `isEsp32Node = false` → the node was treated as a Red
Pitaya → `run()` listened on **UDP :55001** while the node streamed binary over
**TCP** → zero samples.

**Fix:** check `isEsp32HandshakeResponse()` **before** the generic `OK` branch.
ESP32 markers (`esp32s3`, `node=esp32`, or `8 channels` + `sample_rate`) are
specific; a real Red Pitaya has none of them, so it is unaffected. Also drains
the `STARTED`/`SENSORS` lines and adds `127.0.0.1` as a final detection fallback
(see below).

### Fix 2 — no REDPITAYA on the acquisition socket (commit `567bb1b`)
**Symptom:** detection worked, but pressing **Play** produced no data
("acquisition starts, no progression").

**Root cause:** `startAcquisition()` always opens a **fresh** TCP session, but
the ESP32 path sent only `START` on it. The bridge (`--plugin` mode) requires a
`REDPITAYA` line on **every** new connection before it accepts `START`; lacking
it, the bridge logged `expected REDPITAYA, got START` and closed the socket, so
the reader thread read nothing.

**Fix:** re-handshake on the acquisition socket — send `REDPITAYA`, read the
handshake reply, **then** send `START`, then drain the `STARTED`/`SENSORS` reply
up to and including the terminal `SENSORS:` line so the binary parser is
byte-aligned from the first frame header. The Wi-Fi firmware also accepts
`REDPITAYA` harmlessly, so this is safe for both transports.

### Fix 5 — `REDPITAYA`+`START` swallowed by the bridge peek (commit pending)
**Symptom:** still no data after Fix 2 — acquisition starts, nothing streams.

**Root cause:** Fix 2 first sent `REDPITAYA\nSTART\n` in a **single** write. On
connect the bridge does `reader.read(256)` to peek; on localhost both lines land
in that one peek, so the bridge consumes `REDPITAYA` **and** `START` together,
replies to `REDPITAYA`, then blocks in `read_line()` waiting for a `START` it has
already swallowed → 120 s deadlock → no frames.

**Fix:** send `REDPITAYA` and `START` as **separate** writes, reading the
handshake reply in between (via a `readReplyLine` lambda). This forces `START`
into its own `read()` on the bridge after the peek. Fixes the bridge's latent
peek-coalescing bug from the plugin side without editing the bridge.

### Fix 3 — `127.0.0.1` detection fallback (commit `fbdd4ab`)
**Symptom:** the USB bridge runs on `127.0.0.1:5000`, which was never tried
unless the user manually set `ESP32_NODE_HOST` or the editor Node IP.

**Fix:** after the `rp-*.local` hosts and any configured/env host,
`detectBoard()` now also tries `127.0.0.1`. Detection still requires a valid
handshake reply, so an unrelated localhost service won't be mistaken for a
board.

### Fix 4 — OpenSim got nothing from the ESP32 (commit `aa90c89`)
**Symptom:** OpenSim (live/motion) showed no movement with the ESP32 node.

**Root cause (two parts):**
1. The ESP32 streams **raw acc + gyro only** — its firmware's `readImuRaw()`
   reads 14 bytes (accel + temp + gyro) and never reads the ICM20948
   magnetometer, and it computes no orientation. OpenSim IK needs
   **quaternions**.
2. The plugin's OpenSim UDP forwarding (`sendOpenSimQuaternionPacket`) was
   called **only** on the Red Pitaya UDP path, never in the ESP32 TCP branch.

**Fix:** added a self-contained 6-DOF Madgwick AHRS
(`madgwickUpdate6DOF`) and run it in the ESP32 branch of `run()`: convert gyro
to rad/s, fuse with accel, and forward the orientation quaternion on the
existing OpenSim UDP v2 path **when OpenSim is enabled** (after the editor's
**Gen Motion** / **OpenSim Live** button sets `openSimEnabled`).

---

## 3. Why Madgwick and not VQF

`vqf.c` / `vqf.h` are in this repo, but they are **not** linked into the plugin —
the plugin only *passes through* quaternions the Red Pitaya computes on-device.
Using VQF in the plugin would require adding `vqf.c` as a **new compile unit** to
the plugin's CMakeLists.

| | VQF | Madgwick (chosen) |
|---|---|---|
| Build change | New C compile unit in CMake; C/C++ linkage; risk of `undefined reference` errors | None — header math inside `acqboard.ccp` |
| State | ~1 KB struct, Butterworth filters, bias covariance | one 4-float quaternion |
| Strengths | gyro-bias estimation, rest detection, **magnetic** disturbance rejection | simple, fast, dependency-free |
| Benefit **here** | dormant — ESP32 sends **no magnetometer** and no rest calibration | same pitch/roll result |
| Yaw without mag | **drifts** | **drifts** |

For a magnetometer-less 6-DOF stream the two produce effectively the same
orientation (yaw drifts in both), but Madgwick adds **zero build risk** — which
mattered given the recent Debug/Release linker battle. VQF's sophistication only
pays off with a magnetometer (9-DOF) or careful rest-based bias calibration. If
the firmware is extended to read the AK09916 magnetometer, switching the plugin
to VQF (and wiring `vqf.c` into CMake) becomes worthwhile.

---

## 4. How to run

### USB path (default firmware preset `USB_OPEN_EPHYS_MODE = true`)
1. Build the plugin in **Debug**, matching the host configuration (single
   configuration; clean the CMake `Build/` cache if you hit the JUCE link trap).
2. Close the Arduino **Serial Monitor** (frees the COM port).
3. Start the bridge:
   ```powershell
   host\run_usb_plugin_bridge.ps1 COM5
   # equivalently: python host\serial_tcp_bridge.py COM5 --plugin
   ```
   You want to see `first Open Ephys frame from serial (38 bytes)`.
4. Open Ephys → **Acquisition Board** → detect (finds `127.0.0.1`) → **Play**.
   The 8 channels should update.
5. For OpenSim, click **OpenSim Live** (or **Gen Motion**) in the editor; the
   plugin then UDP-forwards fused quaternions to `127.0.0.1:5000`.

### Wi-Fi path (firmware `USB_OPEN_EPHYS_MODE = false`)
1. Set `WIFI_SSID` / `WIFI_PASS` in the `.ino` (2.4 GHz network).
2. Read the board's IP from the serial banner (or join the `STEP_ESP32` Soft AP
   fallback, host `192.168.4.1`).
3. Set the editor **Node IP** (or the `ESP32_NODE_HOST` env var) to that IP, then
   detect and Play.

### Diagnostics
- **Serial side:** `python host\serial_tcp_bridge.py COM5 --plugin --verbose` —
  confirms the board emits 38-byte frames. `no serial frames yet` means the board
  isn't streaming (wrong mode, Serial Monitor open, or the 5 s boot delay).
- **`diagnose_oe_udp.py` is the wrong tool for this pipeline** — it listens on
  **UDP** for a Fake-IMU float stream; the ESP32 path is **TCP**. Use the bridge's
  `--verbose` instead.

---

## 5. Known limitations / future work

- **Yaw drift.** No magnetometer reaches the plugin → heading is unobservable in
  6-DOF fusion. Pitch/roll are stable. Fix: extend firmware to read the AK09916
  magnetometer and add a 9-DOF update (then VQF/9-DOF in the plugin).
- **Single IMU.** One ESP32 node = one sensor; the Rajagopal OpenSim model
  expects several IMUs mapped to body segments. Check `opensim_sensor_map.json`
  and that `opensim_live_realtime.py` accepts `n_sensors = 1`.
- **Axis mapping.** `madgwickUpdate6DOF` uses the standard convention; the ICM
  frame → OpenSim model frame may need an axis/sign remap.
- **Filter gain.** `beta = 0.1` (in `run()`); lower = smoother/slower, higher =
  snappier/noisier.
- **Optional bridge cleanup.** The `OK CHANNELS:8\n` line in the ESP32 reply is
  now harmless (the plugin matches the esp32 markers first) but could be removed
  for an unambiguous handshake.

---

## 6. Commit reference (branch `claude/quirky-hypatia-2NwPv`)

| Commit | Summary |
|---|---|
| `fbdd4ab` | detect ESP32 before the generic `OK` branch; drain `STARTED`/`SENSORS`; add `127.0.0.1` fallback |
| `567bb1b` | re-handshake (`REDPITAYA`+`START`) on the fresh acquisition socket so data streams |
| `aa90c89` | self-contained 6-DOF Madgwick fusion → OpenSim quaternion UDP from the ESP32 path |

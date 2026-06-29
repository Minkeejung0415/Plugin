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
`isEsp32Node` flag selects ESP32-specific behavior (TCP binary stream, default
11 ch @ 100 Hz: raw IMU + DIO + filter quat slots) versus the Red Pitaya
behavior (UDP :55001 stream).

### Wire formats

**Handshake (TCP, ASCII, line-delimited):**

| Direction | Message |
|---|---|
| plugin → node | `REDPITAYA\n` |
| node → plugin | `11 channels; sample_rate=100; node=esp32s3_arduino\n` then `OK CHANNELS:11\n` |
| plugin → node | `START\n` |
| node → plugin | `STARTED BIN:...\n` then `SENSORS:0,ICM20948\n` |
| node → plugin | *(binary frames follow immediately)* |

**Binary frame (`OeHeader`, packed, 22 bytes + payload):**

| Offset | Size | Field | Value |
|---|---|---|---|
| 0  | 4 | `offset` | 0 |
| 4  | 4 | `num_bytes` | 22 (= 11 ch × int16) |
| 8  | 2 | `bit_depth` | 3 (OpenCV S16 enum) |
| 10 | 4 | `element_size` | 2 |
| 14 | 4 | `num_channels` | 11 |
| 18 | 4 | `samples_per_channel` | 1 |
| 22 | 22 | payload | 11 × int16, little-endian |

The plugin's TCP parser validates `element_size == 2` at offset 10 and
`num_bytes` at offset 4, then reads `num_bytes/2` int16 channels. Full frame =
**44 bytes** (11 ch).

**11-channel layout (Red Pitaya–like: raw + filter slots):**

| ch | meaning | scaling in plugin |
|---|---|---|
| 0 | AccX | ÷ 16384 → g (±2 g preset) |
| 1 | AccY | ÷ 16384 |
| 2 | AccZ | ÷ 16384 |
| 3 | GyrX | ÷ 131.072 → °/s (±250 °/s preset) |
| 4 | GyrY | ÷ 131.072 |
| 5 | GyrZ | ÷ 131.072 |
| 6 | DIO | bit0 = level |
| 7–10 | `qw`, `qx`, `qy`, `qz` (OE labels) | Q15 (×1/32767); **0 when FILTER OFF** |

Open Ephys channel names for ESP32 are set in `devicethread.cpp` (`ax`…`qz`). OpenSim UDP uses **`n_sensors=1`** per board; enable **Filter ON** so ch7–10 carry orientation (plugin skips OpenSim when filter is off).

Legacy 8-ch firmware (quat overwrote gyro) is still decoded if `channelsInPacket < 11`.

### Sample rate (no firmware/plugin cap)

ESP32 nodes accept any **Hz ≥ 1** (`FREQ:<Hz>`, `CFG 0 SRATE`, Plugin HW rate label). Values **≤ 0** are ignored/rejected. Red Pitaya boards keep the existing **1–2000 Hz** clamp in the editor. There is no 50–200 Hz limit — tune by testing:

- **Rate too high:** duplicate samples (repeated seq / frozen values) — firmware or USB/TCP cannot sustain the interval.
- **Rate too low or bus-limited:** missing samples (sequence gaps, uneven spacing).

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

## 6. SD Card Recording via rec-v1 Protocol (Phase 06, Plan 03)

### 6.1 Required firmware

The rec-v1 SD recording protocol requires ESP32 firmware that responds to
`REC HELLO protocol_min=rec-v1` with `REC HELLO_OK`.

- **rec-v1 capable firmware:** replies `REC HELLO_OK max_chunk=<n> ...`
- **Old firmware (no rec-v1):** no reply or `REC ERR code=unsupported_protocol`
  → plugin shows "recording protocol unsupported — update firmware for SD recording"
  → falls back to legacy host-PC CSV recording path

The plugin negotiates capability on every connect: `sendEsp32RecHello()` is
called right after the REDPITAYA detection handshake succeeds.

### 6.2 Operator workflow (normal session)

1. Insert an SD card into the ESP32-S3 carrier board.
2. Start the bridge (USB path) or join the Wi-Fi network.
3. In Open Ephys: detect the board (finds `127.0.0.1` or Node IP) → press **Play**.
4. Press **RECORD** (first press):
   - Plugin sends `REC START sample_rate_hz=<hz> channels=<n> format=sd-bin sd_required=true requested_session=<YYYYMMDD_HHMMSS>`
   - Status bar: "ESP32 SD recording: command sent — waiting for confirmation"
   - On `REC STARTED session_id=<id>`: status → "ESP32 SD recording confirmed (session=...)"
5. Conduct experiment.
6. Press **RECORD** again (second press — stop):
   - Plugin sends `REC STOP session_id=<id> reason=manual_stop`
   - Status bar: "ESP32 SD recording: stop sent — finalizing and retrieving in background"
   - Background thread takes over (audio/acquisition path is NOT blocked):
     - Polls `REC STATUS` every 500 ms until `recording_state=finalized` (max 30 s)
     - Status → "finalizing..."
     - Fetches `REC SESSION session_id=latest_finalized` for file_size + file_checksum
     - Status → "SD finalized (<n> bytes) — retrieving..."
     - Sends `REC GET` requests; receives SDRF binary frames; validates per-chunk CRC
     - Status → "ESP32 SD retrieving: <offset> / <total> bytes..."
     - Computes whole-file CRC32; compares with firmware's checksum
     - Status → "ESP32 SD: transfer checksum passed — running analyzer..."
     - Attempts `analyze_sample_rate.py --format sd-bin step_<session_id>.bin`
     - Status → "Recording saved and verified (session=...)"
7. Session files are in:
   `C:\Users\justi\Documents\Arduino\ESP32-S3-1\results\<sessionId>_<timestamp>\`

### 6.3 Output file structure

Each session produces a directory:

```
results\
  <sessionId>_<timestamp_ms>\
    step_<session_id>.bin     SD binary data (analyzer-compatible)
    step_<session_id>.bin.tmp staging file during transfer (deleted on success)
    metadata.json             session_id, protocol, file_size, file_checksum, timestamp_ms
    transfer_log.json         per-chunk CRC, byte offsets, whole_file_crc_match
    analyzer_handoff.json     {"command": "python esp32/host/analyze_sample_rate.py ...", ...}
    analyzer_result.json      {"passed": true/false, "output": "..."}
```

Verify with:
```powershell
python esp32\host\analyze_sample_rate.py --format sd-bin `
  "C:\Users\justi\Documents\Arduino\ESP32-S3-1\results\<dir>\step_<session_id>.bin"
```

### 6.4 Retry workflow (transfer failure)

If the transfer CRC fails (network glitch, disconnect mid-transfer):

1. Status bar shows: "ESP32 SD: transfer CRC mismatch! ... — press Retry Retrieval"
2. The session metadata is retained: `esp32PendingSessionId`, `esp32PendingFileSize`,
   `esp32PendingFileChecksum` are kept in the board object.
3. Call `rp->retryEsp32Retrieval()` (or wire a "Retry" button in the UI):
   - Restores `esp32SessionId` from pending.
   - Restarts the async retrieval thread from the REC GET phase.
4. A new timestamped subdirectory is created for the retry attempt.

### 6.5 Timeout-finalized session recovery (reconnect)

If the experiment is interrupted (ESP32 reboots, USB disconnect, PC crash),
the firmware may have stored a finalized SD session.

On reconnect (next `detectBoard()` → `performDetectionHandshake()` → `sendEsp32RecHello()`):

1. Plugin sends `REC SESSION session_id=latest_finalized`.
2. If the firmware returns `REC SESSION_OK` with non-zero `file_size`:
   - State → `SdFinalized`
   - Status bar: "Timeout-finalized session available (session=...) — press Retrieve"
   - `esp32RetrievalRetryAvailable = true`
3. Operator calls `retryEsp32Retrieval()` to begin the retrieval.

### 6.6 Unsupported firmware

When `sendEsp32RecHello()` receives no `REC HELLO_OK`:

- `esp32RecV1Supported = false`
- State → `UnsupportedProtocol`
- Status bar: "Recording protocol unsupported — update firmware for SD recording"
- RECORD button tooltip: "ESP32 SD recording requires firmware with rec-v1 support. Update firmware to enable."
- Plugin falls back to legacy PC-side CSV recording (if `esp32RecordStream` is in use).

### 6.7 Threading model

| Thread | Responsibility |
|--------|----------------|
| Main (JUCE message) | `sendEsp32RecHello`, `sendEsp32RecStart`, `sendEsp32RecStop` |
| `Esp32RetrievalThread` | All of: poll STATUS, fetch SESSION, REC GET chunks, write .bin, CRC, analyzer |
| `run()` (audio thread) | Frame decode, OpenSim UDP, legacy CSV write (guarded by `!esp32RecV1Supported`) |

`esp32CommandLock` (`CriticalSection`) must be held before any commandSocket
read/write on threads other than main. The retrieval thread acquires it for each
REC STATUS, REC SESSION, REC GET, and REC COMPLETE exchange.

`esp32RecStatusLock` guards `esp32RecStatusText`; the UI timer reads this every
~100 ms and forwards to the Open Ephys status bar.

---

## 8. Commit reference

| Commit | Summary |
|---|---|
| `fbdd4ab` | detect ESP32 before the generic `OK` branch; drain `STARTED`/`SENSORS`; add `127.0.0.1` fallback |
| `567bb1b` | re-handshake (`REDPITAYA`+`START`) on the fresh acquisition socket so data streams |
| `aa90c89` | self-contained 6-DOF Madgwick fusion → OpenSim quaternion UDP from the ESP32 path |
| `ed64c35` | rec-v1 header: Esp32RecState, fields, method declarations (Phase 06 plan 03) |
| `c0f93b4` | rec-v1 implementation: HELLO, START, STOP, async retrieval thread, CRC, analyzer |
| `e063f42` | DeviceEditor: truthful status, rec-v1 tooltip, openSimAngleTimer polling |

---

## 9. Slave SD workflow, CSV conversion, and DIO sync

### 9.1 Current master/slave topology

The Open Ephys plugin talks only to the master at `192.168.4.1:5000`. The
master starts the `STEP_ESP32` Soft AP and forwards START/STOP/REC commands to
slaves over ESP-NOW broadcast. The intended no-master-SD setup is valid:

- master: USB/Wi-Fi control and Open Ephys live stream, no SD required
- slave: joins `STEP_ESP32`, records the source-of-truth binary file to SD
- after acquisition: copy or mount the slave SD card on the PC and convert

Slaves use DHCP by default so several slaves can join the same master AP without
all claiming `192.168.4.2`. For fixed addresses, flash each slave with a unique
`SLAVE_STATIC_IP_OCTET`.

### 9.2 Convert copied slave SD files to CSV

Binary on the slave SD is the acquisition source of truth. CSV is generated
after the recording, not during acquisition.

Single file:

```powershell
python esp32\host\sd_bin_to_csv.py D:\step_000192b00000205a.bin --summary-json
```

Whole SD card or folder:

```powershell
python esp32\host\convert_sd_folder.py D:\
```

The folder converter writes:

- one `.csv` per `step_*.bin`
- one `*_summary.json` per file
- `conversion_summary.csv`
- `conversion_summary.json`

The summary reports record count, duration, sequence loss/duplicates,
quaternion nonzero percentage, DIO edge count, and whether the DIO data is
usable for sync estimation.

### 9.3 DIO sync test

`D0` is sampled into the `dio` CSV column. It has pull-up behavior:

- idle/open circuit: level `1`
- touch or drive D0 to GND: level `0`
- the CSV `dio` value packs level in bit 0 and edge count in bits 1-15

Single-board sanity test:

1. Start recording.
2. Briefly connect D0 to GND a few times.
3. Stop recording and convert the SD file.
4. Confirm `dio_edge_count` is greater than zero in `*_summary.json`.

Multi-board sync test:

1. Connect all ESP32 grounds together.
2. Feed the same 3.3 V-safe sync signal to every slave D0.
3. Record, convert all SD cards with `convert_sd_folder.py`.
4. Check `conversion_summary.json` `sync.pairs` for first offset and drift.

Do not drive D0 with 5 V.

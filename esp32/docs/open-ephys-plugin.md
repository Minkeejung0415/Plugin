# Open Ephys integration — ESP32-S3 vs Plugin repo

This document answers whether [Minkeejung0415/Plugin](https://github.com/Minkeejung0415/Plugin) must be edited for the ESP32-S3 STEP node, and how that relates to the built-in **Ephys Socket** plugin.

Firmware reference: `arduino/step_node/step_node.ino` v1.7.0 · Host test: `host/esp32_tcp_client.py` · Checklist: `.planning/PLUGIN-INTEGRATION.md` · Rate stress test: [stress-test-sample-rate.md](stress-test-sample-rate.md)

---

## Strategy (v1.0 milestone)

| Priority | Path | When to use |
|----------|------|-------------|
| **Primary** | **Plugin repo AcqBoardRedPitaya** | STEP production alignment; OpenSim via Plugin `ephys_to_opensim_*` scripts |
| **Alternate (lab)** | **Ephys Socket** + `host/serial_tcp_bridge.py` | Quick USB test without building Plugin; binary on `127.0.0.1:5000` |

**Bring-up order:** one ESP32 on **USB or Wi-Fi TCP** with Plugin handshake first → Wi-Fi TCP hardened → OpenSim (Plugin scripts) → SD → **ESP-NOW later**. **Camera** is out of scope for this milestone.

**No Plugin C++ in ESP32-S3 repo** — track patches and sign-off in `.planning/PLUGIN-INTEGRATION.md`.

### Firmware presets vs Plugin

| Target | `USB_OPEN_EPHYS_MODE` | `ENABLE_TCP` | Notes |
|--------|----------------------|--------------|--------|
| Plugin on **Wi-Fi** | `false` | `true` | Node IP:5000; send `REDPITAYA\n` then `START\n` (matches `esp32_tcp_client.py`) |
| Ephys Socket on **USB** | `true` | off (USB serial binary) | `serial_tcp_bridge.py COMx` → localhost:5000; GUI does **not** send handshake |
| Plugin on **USB** | `true` | `serial_tcp_bridge.py COMx --plugin` → `127.0.0.1:5000` | Bridge emits ESP32 + `OK CHANNELS`, `STARTED`, `SENSORS`; see [local-open-ephys-setup.md](local-open-ephys-setup.md) §8b |

Plugin AcqBoard expects **TCP control on port 5000** with `REDPITAYA`/`START`. ESP32 already implements those commands on Wi-Fi TCP; replies differ from Red Pitaya (`OK CHANNELS`, `STARTED`, `SENSORS`) and samples ride **TCP** not **UDP 55001** — see [Protocol comparison](#protocol-comparison) and [Path B](#path-b--plugin-repo-acqboard-primary).

---

## Short answer

| Open Ephys path | Edit Plugin repo? |
|-----------------|-------------------|
| **Plugin AcqBoardRedPitaya** (recommended) | **Yes — minimal to moderate** (handshake, START, transport, host IP, scaling) |
| **Built-in Ephys Socket** + USB bridge (alternate) | **No** Plugin build — `USB_OPEN_EPHYS_MODE true` + `serial_tcp_bridge.py` |

For serial CSV bench only, use Serial Monitor or `host/serial_bench_reader.py` (no Open Ephys).

---

## What is in Minkeejung0415/Plugin?

Not a standalone “Ephys Socket” plugin. It is a **custom Open Ephys GUI Source plugin** (DeviceThread + acquisition boards) with:

| Component | Role |
|-----------|------|
| `devicethread.cpp` | Detects OpalKelly → ONI → **Red Pitaya** → simulation |
| `acqboard.ccp` + `Acqboardredpitaya.h` | **AcqBoardRedPitaya** — TCP control + **UDP data** from Red Pitaya |
| `RedPitaya_justin.c` | Reference Red Pitaya firmware (multi-sensor, fusion, SD record) |
| `ephys_to_opensim_bridge.py`, `opensim_live_realtime.py` | OpenSim UDP bridge (localhost:5000) |

There is **no** separate Ephys Socket implementation in that repo; streaming logic is Red-Pitaya-specific.

---

## Protocol comparison

| Item | Red Pitaya (`RedPitaya_justin.c` + Plugin) | ESP32-S3 STEP (`step_node.ino`) |
|------|--------------------------------------------|----------------------------------|
| TCP port | 5000 (control only) | 5000 (control, status, rec-v1 retrieval) |
| Data transport | UDP 55001 per sample | UDP 55001 in direct Wi-Fi mode; queued serial/TCP bridge in USB mode |
| REDPITAYA reply | OK CHANNELS:N | Includes transport=udp for direct Wi-Fi or transport=tcp for USB bridge mode |
| START reply | STARTED then SENSORS | STARTED includes transport and UDP port, followed by SENSORS:0,ICM20948 |
| Sample rate | Configurable (`FREQ:`), default 100 Hz | **`FREQ:<Hz>`** any integer **≥ 1** (default 100 Hz; no firmware/plugin cap — user finds true min/max by testing) |
| `CFG` / `FILTER` | Yes | **`CFG 0 ACC|GYR <0-3>`**, **`FILTER ON|OFF`** |
| Channels | Dynamic (sensors × raw + quat + analog) | Fixed **14** int16 (firmware ≥1.5.0) |
| Packet header | 22-byte LE `iiHiii` | **Same** 22-byte layout (see [Binary frame header](#binary-frame-header-v170)) |
| Payload | int16 channel-major | int16 channel-major |
| Raw IMU | Always in first `num_raw` slots per sensor (e.g. MPU6050: ch0–2 acc, ch3–5 gyr) | **Always** ch0–2 acc, ch3–5 gyr (never replaced by quat) |
| Filter / quat | Next 4 slots after raw (`num_raw`…`num_raw+3`), Q15; **0 when `FILTER OFF`** | ch9-12 qw,qx,qy,qz Q15; **0 when `FILTER OFF`** |
| ch13 | Part of sensor layout / DIO varies | DIO packed (level + edge count) |
| Host discovery | Hardcoded `rp-f0f85a.local`, `rp-f0cd35.local` | Wi-Fi IP from Serial Monitor |

### Channel map (ESP32 firmware ≥1.5.0)

| Ch | Open Ephys label (Plugin) | Role | `FILTER OFF` | `FILTER ON` |
|----|---------------------------|------|--------------|-------------|
| 0–2 | `ax`, `ay`, `az` | Accel raw (ICM int16) | IMU | IMU |
| 3–5 | `gx`, `gy`, `gz` | Gyro raw (ICM int16) | IMU | IMU |
| 6-8 | `mx`, `my`, `mz` | Magnetometer raw (AK09916 int16) | Mag | Mag |
| 9-12 | `qw`, `qx`, `qy`, `qz` | Filter quaternion Q15 | **0** (flat) | VQF |
| 13 | `dio` | DIO (bit0 level, bits1-15 edge count) | DIO | DIO |

**Flat channels in the GUI:** With **Filter OFF**, ch9-12 are intentionally zero. With **Filter ON**, quat channels are Q15 (±32767) while accel/gyro are raw LSB — auto-scaled traces can make quats look “flat” if the Y axis is tuned for ~16k accel counts. Toggle filter or rescale the display.

**OpenSim (one IMU):** Plugin sends UDP v2 with `n_sensors=1` only. Enable **Filter ON** (firmware VQF on ch9-12, or plugin Madgwick fallback on 6–8 ch packets). Do not use the pre-1.5.0 layout that treated gyro slots as quaternion components.

**Before v1.5.0 (8 ch, deprecated):** ch3–5 and ch7 held gyro **or** quat (mutually exclusive on wire). Plugin ≥ current still decodes legacy 8-ch packets if `channelsInPacket < 11`.

**Red Pitaya reference (one MPU6050-class sensor):** 6 raw + 4 quat = 10 stream slots (+ analog tail). Current ESP32 uses 6 accel/gyro + 3 mag + 4 quat + DIO = 14.

**What already matches:** port 5000, `REDPITAYA` / `START` command names, 22-byte Open Ephys header, int16 channel-major samples, configurable rate via `FREQ:` (Hz ≥ 1 only).

### SD-first task isolation (2026-06-18)

The acquisition loop runs on ESP32 core 1. It enqueues the SD record first, then makes one
non-blocking offer to a 16-record live-stream queue. Core 0 runs two writers:

- SD writer: priority 4, deep 1024-record queue, persistence path.
- Stream writer: priority 1, shallow latest-data queue, UDP or queued USB serial output.

If the stream queue fills, the oldest live packet is discarded. Stream queue drops and
send errors may increase, but they do not contribute to SD errors or recording success.
STATUS reports stream offered, enqueued, sent, drop, error, depth, and latency counters.

UDP is a live view, not evidence of data integrity. Only finalized SD sequence continuity,
generated/saved agreement, and zero SD queue/write errors support a no-loss claim.

For direct Wi-Fi operation set USB_OPEN_EPHYS_MODE false. Open Ephys connects to TCP
port 5000 for handshake/control, then receives sample datagrams on UDP port 55001. Permit
inbound UDP 55001 through the Windows firewall.
### Binary frame header (v1.7.0)

**`HEADER_SIZE` = 22 bytes** — unchanged from Ephys Socket / Red Pitaya layout (`struct "<iiHiii"`). No extra bytes; backward compatible parsers that ignore unknown fields still work.

| Offset (bytes) | Field | Type | ESP32-S3 (≥1.7.0) | Legacy (≤1.6.0) |
|----------------|-------|------|-------------------|-----------------|
| 0 | `offset` | int32 LE | **Hardware time:** low 32 bits of `esp_timer_get_time()` µs since boot (monotonic per board; wraps ~71 min) | Always `0` |
| 4 | `num_bytes` | int32 LE | `NUM_CHANNELS × 2` (28 for 14 ch) | Same |
| 8 | `bit_depth` | uint16 LE | `3` (Open Ephys S16 enum) | Same |
| 10 | `element_size` | int32 LE | `2` (int16) | Same |
| 14 | `num_channels` | int32 LE | `14` | Same |
| 18 | `samples_per_channel` | int32 LE | `1` | Same |
| 22+ | payload | int16[] | Channel-major samples | Same |

**Clock choice:** `esp_timer_get_time()` (µs since boot, same domain as ESP-NOW `SyncPacket.time_us`). Not wall-clock / NTP — Wi-Fi time sync is a separate layer.

**Plugin / host:** When `offset != 0`, AcqBoard ESP32 path and `host/stress_test_serial.py` use **inter-frame Δoffset** (uint32 wrap-safe) for rate and timeline spacing. When `offset == 0`, fall back to host arrival time (USB bridge, old firmware).

### Multi-ESP32 timestamp sync (v1 minimal)

Each slave stamps frames with **its own** monotonic µs since boot. Slaves do **not** share an absolute epoch unless you add one.

| Approach | When to use |
|----------|-------------|
| **Per-frame `offset` (v1.7.0)** | Single node or post-hoc merge; compare Δt within one COM/TCP stream |
| **Shared START** | Master (or host) sends `START\n` / GPIO pulse to all nodes at once; align first frame after START |
| **ESP-NOW `SyncPacket`** | Master broadcasts `{seq, time_us}` (`esp_timer_get_time()`); slaves already use same struct when `ENABLE_ESPNOW` |
| **Host merge** | Record `host_wall` at START; map each slave’s `offset` + USB latency; or use Open Ephys event channel on DIO |
| **UTC / NTP** | Future: optional SNTP on STA — not required for sample pacing |

**Recommendation:** For multiple ESP32-S3 nodes, designate one **master** that issues START (or a DIO sync line into ch6 on all boards). Log each stream separately; align offline using first post-START `offset` delta or ESP-NOW sync packets. Plugin v1 uses hardware `offset` only for **one** TCP stream at a time.

### ESP32 TCP text commands (v2.0)

| Command | Notes |
|---------|--------|
| `REDPITAYA\n` | Returns `sample_rate=<Hz>` and `filter=on\|off` |
| `FREQ:<Hz>\n` | Any Hz **≥ 1** (invalid ≤ 0 rejected) |
| `CFG 0 ACC <0-3>\n` / `CFG 0 GYR <0-3>\n` | ICM full-scale presets |
| `FILTER ON\n` / `FILTER OFF\n` | VQF quaternion on ch7–10 (Q15); ch0–5 always raw |
| `START\n` | Binary stream on same TCP socket |

**OpenSim:** Plugin reads filter slots ch7–10 (Q15), same convention as Red Pitaya quat tail — flat zeros when `FILTER OFF`, live quat when `FILTER ON`.

**Current plugin behavior:** handshake negotiation selects ESP32 UDP or legacy TCP explicitly. Direct Wi-Fi uses UDP 55001 for samples and TCP 5000 for control/retrieval; USB bridge mode remains TCP-compatible.

### Sample rate (no fixed cap)

Firmware, USB bridge, and Plugin accept any **Hz ≥ 1** via `FREQ:<Hz>` / editor HW rate (Red Pitaya boards still use **1–2000 Hz** in the Plugin). There is no 50–200 Hz clamp — find the real minimum and maximum by experiment:

- **Too high:** duplicated samples (same seq/value repeated) — loop cannot keep up or USB/TCP cannot carry the stream.
- **Too low / USB-limited:** gaps or lost frames (sequence jumps, uneven timeline).

Full procedure: **[stress-test-sample-rate.md](stress-test-sample-rate.md)** · analyzer: `python host/analyze_sample_rate.py your_log.csv`

---

## Path A — Ephys Socket + USB bridge (alternate lab path)

Use the **built-in Open Ephys Ephys Socket** plugin when you are **not** building the Plugin repo:

1. On ESP32: `#define ENABLE_TCP true`, `#define ENABLE_SERIAL_BENCH false`, set Wi-Fi credentials.
2. Note node IP from Serial Monitor.
3. In Open Ephys GUI: add **Ephys Socket** → TCP client → `<node-ip>:5000`.
4. **Connect only** — built-in Ephys Socket is a TCP client that immediately reads **22-byte binary packet headers**; it does **not** send `REDPITAYA` / `START`. ESP32 firmware TCP mode still expects that text handshake today; for Open Ephys use **USB + `serial_tcp_bridge.py`** (see [arduino-ide-guide.md](arduino-ide-guide.md)) or adapt firmware to stream binary on connect.
5. Expect **14 channels @ 100 Hz**; scale ax–gz on the host (raw int16 ÷ sensitivity — see `host/esp32_tcp_client.py` env `ICM_ACCEL_SCALE` / `ICM_GYRO_SCALE`).

Verify with Python first:

```powershell
set ESP32_NODE_HOST=<node-ip>
python host\esp32_tcp_client.py
```

**Plugin repo:** no edits. **Not** the milestone primary path for STEP/OpenSim.

---

## Path B — Plugin repo AcqBoard (primary)

To use **AcqBoardRedPitaya** from [Minkeejung0415/Plugin](https://github.com/Minkeejung0415/Plugin) with ESP32-S3 **as-is**, the plugin will fail at detection and/or streaming. Required changes in **Plugin repo** (checklist: `.planning/PLUGIN-INTEGRATION.md`). Firmware parity in ESP32-S3 is an alternative — see [Alternative: align ESP32 firmware](#alternative-align-esp32-firmware-to-red-pitaya-plugin).

### 1. `acqboard.ccp` — `performDetectionHandshake()`

**Today:** Requires response containing `"OK"` and parses `CHANNELS:N`.

**Change:** Also accept ESP32 reply, e.g. parse `14 channels` or `sample_rate=100`, set `numAdcChannels` from the reply, `deviceFound = true` without requiring `"OK"`.

### 2. `acqboard.ccp` — `kRedPitayaHosts[]` / connect path

**Today:** Only `rp-f0f85a.local`, `rp-f0cd35.local`.

**Change:** Add configurable host (editor text field or env) for ESP32 IP, e.g. `192.168.x.x`, or try mDNS `esp32s3.local` if you add it on the board.

### 3. `acqboard.ccp` — `startAcquisition()`

**Today:** Blocks until `STARTED` / `STARTED BIN:…` and reads `SENSORS:…` line for channel layout and OpenSim mapping.

**Change:** For ESP32 mode: after `START\n`, if no `STARTED` within timeout, assume **fixed 8-channel ESP32 layout**; set `streamSensorNames = { "ICM20948" }` or synthetic single-sensor map; skip SENSORS parse.

### 4. `acqboard.ccp` — `run()`

**Today:** Binds **UDP 55001**, reads `headerSize + numAdcChannels*2` per datagram.

**Change (pick one):**

- **Option 4a (plugin-side):** ESP32 mode reads binary packets from **TCP `commandSocket`** after START (mirror `host/esp32_tcp_client.py` loop).
- **Option 4b (firmware-side):** Add UDP 55001 streaming to ESP32 to match Red Pitaya — then plugin `run()` stays UDP-only.

### 5. `acqboard.ccp` — scaling / channel names

**Today:** Per-sensor ACC/GYR presets from Red Pitaya `CFG` commands; quaternion slots after raw IMU.

**Change:** For legacy 8-ch ESP32 map, fixed scale factors for ch0–5 (ICM20948 ±2g / ±250°/s defaults or match sketch), ch6 = DIO (bit0 level), ch7 = 0; disable quaternion OpenSim path unless fusion added on ESP32.

### 6. `devices/redpitaya/AcqBoardRedPitaya.h` + `devicethread.cpp`

Optional: rename or add `AcqBoardEsp32S3` subclass; register in `detectBoard()` after Red Pitaya probe fails but TCP handshake to user IP succeeds.

### 7. `device editor.cpp`

Optional: IP/host field and “ESP32 fixed 8 ch” toggle in UI.

**Do not need to change:** `RedPitaya_justin.c` (Red Pitaya only), OpenSim bridge scripts (unless you want ESP32 quaternion layout).

---

## Alternative: align ESP32 firmware to Red Pitaya plugin

Instead of editing the Plugin, you could extend `step_node.ino` to:

- Reply `OK CHANNELS:14\n` to `REDPITAYA`
- Reply `STARTED\n` + `SENSORS:0,ICM20948\n` to `START`
- Stream samples on **UDP 55001** (keep TCP for commands)

That would be firmware work in **ESP32-S3**, not Plugin repo. The custom plugin would then need only **host IP** configuration and **8-channel scaling** tweaks.

---

## OpenSim (Plugin repo only)

After AcqBoard streams into Open Ephys, run Plugin-repo scripts (not in ESP32-S3):

- `ephys_to_opensim_bridge.py`
- `opensim_live_realtime.py`

Legacy note: older builds used the **fixed 8-channel ESP32 map** (ch0–5 ICM int16, ch6 DIO bit0, ch7 = 0). Red Pitaya quaternion tail slots do not apply unless firmware adds fusion.

---

## See also

- [.planning/PLUGIN-INTEGRATION.md](../.planning/PLUGIN-INTEGRATION.md) — operator checklist, presets, verification commands
- [.planning/ROADMAP.md](../.planning/ROADMAP.md) — phase order
- [arduino-ide-guide.md](arduino-ide-guide.md) — TCP / serial bench setup
- [stress-test-sample-rate.md](stress-test-sample-rate.md) — find max sustainable `FREQ:` / detect duplicates & gaps
- [wiring-diagram.md](wiring-diagram.md) — ICM dual-silk wiring
- Plugin change log: `docs/2026-04-27-change-documentation.md` in Plugin repo (UDP 55001, SENSORS line)

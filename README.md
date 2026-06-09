# Open Ephys IMU Acquisition Plugin

Real-time motion capture plugin for [Open Ephys GUI](https://open-ephys.org/) that streams inertial data from up to 8 IMU sensors, runs on-device VQF sensor fusion, and drives live musculoskeletal inverse kinematics (IK) in OpenSim 4.5.

---

## Highlights

- **8 concurrent IMU sensors** streamed over TCP/UDP at up to **2000 Hz** with sub-millisecond loop latency
- **125 MHz hardware counter polling** on Red Pitaya ARM Cortex-A9 — no OS scheduler, deterministic ~600 µs frame timing
- **VQF (Vectorial Quaternion Filter)** with motion-adaptive Kalman bias estimation; auto-switches 9D → 6D on magnetic disturbance
- **Live OpenSim IK at 20 Hz** — joint angles computed from quaternion stream, fed back to Open Ephys GUI in real time
- **Dual transport support**: wired Red Pitaya (FPGA board via UDP) and wireless ESP32-S3 node (Wi-Fi / USB-serial bridge via TCP)
- **4-board abstraction** (Red Pitaya, ESP32-S3, ONI, OpalKelly) behind a single virtual `AcquisitionBoard` interface
- **JSON hot-swap**: sensor-to-body-segment mapping and joint display selection reload every 25 frames without restarting

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  Hardware                                                       │
│  Red Pitaya (AXI GPIO / I2C)  ──or──  ESP32-S3 (Wi-Fi / USB)   │
└──────────────────┬──────────────────────────────────────────────┘
                   │  TCP binary frames  (11 ch: ax ay az gx gy gz
                   │                      dio qw qx qy qz)
┌──────────────────▼──────────────────────────────────────────────┐
│  Process 1 — C++ Open Ephys Plugin                              │
│  • AcqBoardRedPitaya / AcqBoardESP32 : DataThread               │
│  • JUCE editor: sample rate, sensor config, body segment picker │
│  • Writes opensim_sensor_map.json, opensim_display_joint.json   │
│  • Forwards quaternion packets → UDP :5000                      │
└──────────────────┬──────────────────────────────────────────────┘
                   │  UDP v2 quaternion packets
                   │  {t, version=2.0, N, [qw qx qy qz] × N}
┌──────────────────▼──────────────────────────────────────────────┐
│  Process 2 — Python AHRS Bridge  (ephys_to_opensim_bridge.py)   │
│  • imufusion AHRS on raw acc/gyro  ──or──  pass-through quats   │
│  • Detects session reset / stale fake packets                   │
│  • Forwards fused quaternions → OpenSim IK solver               │
└──────────────────┬──────────────────────────────────────────────┘
                   │  OrientationsReference  (OpenSim SDK)
┌──────────────────▼──────────────────────────────────────────────┐
│  Process 3 — OpenSim IK Solver  (opensim_live_realtime.py)      │
│  • Rajagopal2015 full-body model, Simbody 3D visualizer         │
│  • InverseKinematicsSolver.assemble() per frame @ 20 Hz         │
│  • Streams joint angle → UDP :5001 (v3.1 feedback packet)       │
│  • Plugin displays live angle in Open Ephys GUI overlay         │
└─────────────────────────────────────────────────────────────────┘
```

---

## Key Bugs Solved

### 1. Sample Rate Race Condition — UI lags behind hardware on acquisition start

**Root cause:** `syncRedPitayaBoardSampleRateFromLabel()` was called *after* `startAcquisition()` in the editor's start sequence. The board's `settings.boardSampleRate` was still stale when `startAcquisition()` calculated `ticks_per_sample` and sent the initial `FREQ:N` command. Open Ephys then expected e.g. 1000 Hz while the hardware streamed at 100 Hz, causing buffer overflow / progression stall.

**Fix:** Moved `syncRedPitayaBoardSampleRateFromLabel()` to execute *before* `startAcquisition()`, then re-sent `FREQ:N` post-START on the ESP32 path so the USB serial bridge also applied the correct rate. (`e934948`, `3fd571a`)

---

### 2. ESP32 TCP Handshake Mismatch — acquisition appears to start but no data arrives

**Root cause (detection):** The ESP32 firmware and USB-serial bridge respond to `REDPITAYA` with two lines that coalesce into one TCP read over localhost. The `detectBoard()` parser checked for `"OK"` before checking the `esp32s3_arduino` marker, so `"OK CHANNELS:8"` misclassified the node as a Red Pitaya. The plugin then waited for UDP on `:55001` while the board streamed binary over TCP — detection succeeded but zero samples arrived. (`fbdd4ab`)

**Root cause (streaming):** Even after detection was fixed, `startAcquisition()` opened a fresh TCP connection and sent only `START`. The USB-serial bridge requires a `REDPITAYA` line on *every* new connection before accepting `START`; without it the bridge logged `expected REDPITAYA, got START` and closed the socket silently. (`567bb1b`)

**Fix:** Re-ordered the detection branches so `isEsp32HandshakeResponse()` wins before the generic `"OK"` branch. In `startAcquisition()`, always send `"REDPITAYA\nSTART\n"` on the fresh socket and drain all ASCII reply lines (handshake markers → `SENSORS:`) before launching the binary frame reader. (`fbdd4ab`, `567bb1b`)

---

### 3. TCP Frame Parser Poisoning by Post-START ASCII Replies

**Root cause:** After sending `START`, the ESP32 Wi-Fi firmware ACKs subsequent `FREQ:N` and `FILTER ON/OFF` commands with ASCII `"OK FREQ:N"` / `"OK FILTER ON/OFF"` lines. These arrived in the TCP buffer before the reader thread started. The binary frame parser in `run()` had no ASCII-drain logic, so it byte-scanned through the ACK text to find a valid frame header, corrupting timestamps on the first several samples and occasionally causing a full desync.

**Fix:** After sending post-START commands, call `readReplyLine()` to drain each ACK before `startThread()` is called. The drain is a no-op on the first non-printable byte (firmware version without ACKs), making it safe for both Wi-Fi and USB-bridge transports. (`8a72bc2`)

---

## Hardware

| Board | Interface | Max Sample Rate | Notes |
|---|---|---|---|
| Red Pitaya STEMlab 125-14 | UDP (port 55001) | 2000 Hz | AXI GPIO/I2C, ARM Cortex-A9 |
| ESP32-S3 STEP node | TCP Wi-Fi / USB-serial bridge | 200 Hz | ICM20948, 11-channel binary frame |
| ONI host | USB (libonh) | board-dependent | Open Ephys ONI API |
| OpalKelly | USB | board-dependent | FPGA-based |
| Simulated | — | configurable | for UI / pipeline testing |

Supported IMUs: ICM20948, MPU9250, MPU6050, BNO055

---

## Setup

### Requirements

- Windows 10/11 (PowerShell 5.1+)
- [Open Ephys GUI](https://open-ephys.org/gui) with plugin SDK
- CMake 3.15+
- Python 3.8 (required for OpenSim SDK DLL compatibility)
- [OpenSim 4.5](https://simtk.org/projects/opensim)

### One-shot install

```powershell
.\setup-local.ps1
```

Clones Open Ephys plugin repos, patches `AcquisitionBoard.h`, detects/installs CMake and Python 3.8, and configures OpenSim paths.

### Build plugin

```powershell
cmake -B build -S .
cmake --build build --config Release
```

Copy the resulting `.dll` into your Open Ephys `plugins/` directory.

### Run OpenSim live bridge

```powershell
.\run_bridge.bat
```

Or launch from the plugin UI — the **OpenSim Live** button spawns the Python bridge and IK solver automatically.

### Fix OpenSim paths (if needed)

```powershell
.\fix-opensim-paths.ps1
```

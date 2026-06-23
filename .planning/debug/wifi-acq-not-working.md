---
status: awaiting_human_verify
slug: wifi-acq-not-working
trigger: "Open Ephys detects the ESP32 Soft AP node but no acquisition samples appear."
created: 2026-06-19T00:00:00Z
updated: 2026-06-19T15:12:00-07:00
---

## Current Focus

hypothesis: A stale acquisition-board DLL loaded by the Debug GUI ignored the firmware's negotiated UDP transport and waited for sample frames on TCP.

next_action: Reconnect Windows to STEP_ESP32, relaunch C:\Users\justi\dev\GUI\Build\Debug\open-ephys.exe, start acquisition, and verify the console says `(UDP samples + TCP control)` and live LFP samples move.

## Symptoms

- Firmware Soft AP is `STEP_ESP32`; node IP is `192.168.4.1`.
- Open Ephys successfully connected to TCP control port 5000.
- Firmware handshake reported `14 channels`, `sample_rate=100`, and `transport=udp`.
- The same runtime immediately logged `Detected ... (TCP stream)` and no received-sample evidence.

## Evidence

- timestamp: 2026-06-19T14:45:00-07:00
  source: attached Open Ephys log
  finding: TCP detection and handshake succeeded at `192.168.4.1:5000`; this eliminates node discovery and TCP reachability as root causes.

- timestamp: 2026-06-19T14:45:00-07:00
  source: attached Open Ephys log
  finding: The handshake contains `transport=udp`, but the loaded plugin reports `(TCP stream)`. These outcomes are mutually inconsistent with the current source, which sets `esp32UsesUdpStream` from that exact marker.

- timestamp: 2026-06-19T15:02:31-07:00
  source: latest Open Ephys activity log
  finding: Acquisition starts and stops about three seconds later, but the log has no UDP receive or sample-count evidence. Start/stop alone does not prove packets were received.

- timestamp: 2026-06-19T15:09:00-07:00
  source: Windows Firewall ActiveStore
  finding: Enabled inbound Allow rules cover the launched Debug `open-ephys.exe` for both UDP and TCP on Private and Public profiles. The earlier firewall conclusion is eliminated.

- timestamp: 2026-06-19T15:10:00-07:00
  source: runtime/build artifacts
  finding: The launched GUI's plugin at `C:\Users\justi\dev\GUI\Build\Debug\plugins\acquisition-board.dll` was dated 2026-06-17, older than the corrected source/build artifacts. This explains the transport-state contradiction.

- timestamp: 2026-06-19T15:11:15-07:00
  source: rebuilt and installed Debug plugin
  finding: `cmake --build Build --config Debug --target acquisition-board` succeeded. The rebuilt DLL was copied into the Debug GUI plugin folder; source and installed hashes match, and the binary contains `transport=udp`, `UDP samples + TCP control`, and the UDP bind diagnostic.

## Root Cause Analysis

### Confirmed root cause: stale runtime plugin DLL

The firmware sends samples as UDP datagrams to the TCP client's IP on local destination port 55001 after negotiating `transport=udp`. The stale plugin loaded by the Debug GUI treated the connection as a TCP sample stream. Thus firmware sent UDP while Open Ephys waited on TCP, yielding no samples despite successful discovery and acquisition start.

### Eliminated: Windows Firewall

The Open Ephys executable has enabled program-wide inbound UDP and TCP Allow rules on the relevant profiles. There is no evidence that Windows dropped port 55001, and the prior debug file's claim that this was confirmed was unsupported.

### Not implicated by current evidence

- ESP32/ICM chip health: handshake and streaming transport failure occur independently of sensor-value quality.
- TCP discovery: explicitly successful.
- UDP port bind conflict: no `Failed to bind UDP sample port 55001` appeared, but the stale DLL was not using UDP in the affected run.

## Resolution

root_cause: The Debug Open Ephys GUI loaded a stale June 17 acquisition-board DLL that selected TCP samples even when the ESP32 handshake negotiated UDP.

fix: Rebuilt the acquisition-board Debug target from current source and installed the resulting DLL into `C:\Users\justi\dev\GUI\Build\Debug\plugins\acquisition-board.dll`.

verification: Build and binary deployment are verified. End-to-end sample receipt is pending because Windows is currently disconnected from `STEP_ESP32`.

## Verification Checklist

1. Connect the PC to `STEP_ESP32`.
2. Relaunch the Debug GUI so it loads the new DLL.
3. Confirm detection logs `(UDP samples + TCP control)`, not `(TCP stream)`.
4. Start acquisition and confirm LFP traces/sample counters advance.
5. If the new label is correct but samples remain absent, capture UDP 55001 or add first-packet/timeout diagnostics; do not reassert firewall without packet evidence.

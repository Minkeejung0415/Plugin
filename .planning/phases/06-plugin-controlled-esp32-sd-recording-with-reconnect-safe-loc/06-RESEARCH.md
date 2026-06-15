# Phase 06 Research: Plugin-Controlled ESP32 SD Recording With Reconnect-Safe Local Retrieval

**Status:** Complete
**Date:** 2026-06-15

## Findings

### Current Recording Split

- `acqboard.ccp` already routes ESP32 `sendRecordOnCommand()` to `RECORD ON` and sets `lastRecordingPath` to `/sdcard/step_session.bin`.
- `device editor.cpp` currently reports recording status from local plugin path fields, so it can imply success without firmware confirmation.
- `Acqboardredpitaya.h` still carries `esp32RecordStream` as a legacy PC-side CSV recording path. Phase 2 context says normal ESP32 recording should be board-side SD, not silent PC-side CSV.

### USB Bridge Gap

- `esp32/host/serial_tcp_bridge.py` forwards selected plugin commands to serial through `_RELAY_PREFIXES`.
- Current relay prefixes are `FREQ`, `CFG`, `FILTER`, and `STOP`; `RECORD` is missing.
- In USB bridge mode, plugin RECORD may not reach Arduino firmware unless `RECORD` is added to the relay path and status replies are handled.

### Firmware Status Surface

- Arduino `step_node.ino` already exposes useful `SD_STATUS` and `STATUS` text including readiness, recording state, saved samples, errors, max SD write latency, loop timing, and overruns.
- ESP-IDF `sd_logger.c` has counters through `sd_logger_get_stats()` but `open_ephys_stream.c` only handles `RECORD ON/OFF`; it does not yet provide rich status or file-transfer commands.
- A reconnect-safe design needs firmware to distinguish `recording_requested`, `file_open`, host connected, host disconnected within grace, and finalized due to timeout.

### Retrieval Design Options

1. **Command-channel chunk transfer:** Reuse the control socket/bridge to request file metadata and stream chunks after `RECORD OFF`. Simple topology; requires framing so chunks cannot be confused with live binary frames.
2. **Sidecar transfer endpoint:** Firmware/bridge exposes a separate file-transfer TCP endpoint. Cleaner separation; more code and connection management.
3. **Manual SD copy plus plugin status:** Lower implementation risk, but does not satisfy the desired "flushed to local machine from ESP32" workflow.

Recommended planning direction: start with explicit command/status framing and choose the simplest transfer path that works in both direct TCP and USB bridge mode. Do not mix file bytes into the existing live Open Ephys sample stream without a clear mode boundary.

## Implementation Risks

- Transfer after stop can be slow over serial/115200, so the plan should include file size reporting, progress, and a way to abort transfer without corrupting the SD file.
- If connection loss occurs during transfer rather than recording, firmware should keep the finalized SD file available for a later retry.
- The plugin must avoid blocking the acquisition thread/UI thread while waiting for flush or transfer.
- Direct TCP firmware and Arduino USB bridge mode may need different transport mechanics, but they should expose the same operator-level status.

## Recommended Acceptance Tests

- No-SD dry run: RECORD command reaches firmware and reports SD failure truthfully.
- SD smoke run: plugin RECORD starts SD logging, stop finalizes it, `sd_saved > 0`, `sd_errors = 0`.
- USB bridge run: `RECORD ON/OFF` is relayed to serial firmware and status returns through the bridge.
- Disconnect run: while recording, kill the host connection; reconnect before timeout and verify same session continues.
- Timeout run: kill the host connection and wait past the grace period; verify firmware finalizes the file and reports timeout finalization on next connection.
- Retrieval run: after stop/finalize, retrieve the file locally and run `esp32/host/analyze_sample_rate.py --format sd-bin`.

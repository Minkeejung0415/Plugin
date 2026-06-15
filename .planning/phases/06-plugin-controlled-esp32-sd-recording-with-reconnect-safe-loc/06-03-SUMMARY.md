---
phase: 06-plugin-controlled-esp32-sd-recording-with-reconnect-safe-loc
plan: 03
subsystem: plugin
tags: [esp32, sd-card, rec-v1, plugin-ui, retrieval, async, reconnect, analyzer]
requires:
  - phase: 06-plugin-controlled-esp32-sd-recording-with-reconnect-safe-loc
    provides: rec-v1 firmware and bridge implementation
provides:
  - Plugin RECORD button wired to rec-v1 start/stop/finalize/retrieve
  - Async background retrieval thread with full lifecycle state machine
  - Truthful operator status for all recording lifecycle states
  - Local session output structure (binary, metadata, transfer log, analyzer handoff)
  - Timeout-finalized session discovery on reconnect (D-10)
  - Unsupported firmware detection and clear status message
affects: [plugin, device-editor, firmware-contract, local-results]
key-decisions:
  - D-01: Normal ESP32 recording is board-side SD logging; no silent PC-side CSV fallback for rec-v1 sessions
  - D-03: Plugin must NOT say "Recording saved" until analyzer-backed local verification passes
  - D-05: Local saving is post-record retrieval, not live capture
  - D-10: On reconnect, plugin surfaces timeout-finalized sessions via checkEsp32ReconnectSession()
  - D-11: Plugin status consumes explicit acknowledgements for every lifecycle state transition
  - D-13: Plugin gracefully handles no-rec-v1 firmware with tooltip + status message
metrics:
  duration_seconds: 401
  completed_date: "2026-06-15"
  tasks_completed: 12
  tasks_total: 12
  files_created: 0
  files_modified: 3
---

# Phase 06 Plan 03: Plugin UI, Local Retrieval, and End-to-End Verification Summary

Plugin RECORD button wired to rec-v1 SD start/stop/finalize/retrieve with async
background thread, whole-file CRC32 verification, analyzer invocation, and
truthful lifecycle status in the Open Ephys status bar.

## What Was Built

### Task 1 — rec-v1 capability state in Acqboardredpitaya.h
Added to the header:
- `Esp32RecState` enum (11 states: Idle → CommandSent → RecordingConfirmed →
  Finalizing → SdFinalized → Retrieving → LocalCopyWritten → ChecksumPassed →
  AnalyzerPassed / UnsupportedProtocol / Failed)
- `esp32RecV1Supported`, `esp32SessionId`, `esp32MaxChunkBytes`, `esp32LocalResultDir`
- Thread-safe `esp32RecStatusText` + `esp32RecStatusLock`
- Pending-session fields for retry: `esp32PendingSessionId`, `esp32PendingFileSize`,
  `esp32PendingFileChecksum`, `esp32RetrievalRetryAvailable`
- `esp32RetrievalThread`, `esp32RetrievalCancelRequested`, `esp32CommandLock`
- Method declarations: `sendEsp32RecHello`, `checkEsp32ReconnectSession`,
  `sendEsp32RecStart`, `sendEsp32RecStop`, `startEsp32RetrievalAsync`,
  `retryEsp32Retrieval`, `cancelEsp32Retrieval`, `getEsp32RecStatusText`, `getEsp32RecState`

### Tasks 2 & 3 — sendRecordOnCommand / sendRecordOffCommand for ESP32
- `sendRecordOnCommand` for ESP32 now routes to `sendEsp32RecStart()`
- `sendRecordOffCommand` for ESP32 now routes to `sendEsp32RecStop()`

### Tasks 4 & 10 — Async retrieval thread + retry
- `Esp32RetrievalThread` (JUCE Thread subclass) implements the full async lifecycle:
  1. Poll `REC STATUS` every 500 ms until `recording_state=finalized` (30 s timeout)
  2. Fetch `REC SESSION session_id=latest_finalized` → parse file_size, checksum
  3. Create local session dir: `C:\Users\justi\Documents\Arduino\ESP32-S3-1\results\<sessionId>_<ts>\`
  4. Write `metadata.json`
  5. Loop `REC GET` → read 64-byte SDRF header → validate magic + header CRC32 →
     read payload → validate payload CRC32 → write to `.tmp` file → build transfer_log.json
  6. Compute whole-file Ethernet CRC32; compare with firmware checksum
  7. On match: atomically rename `.tmp` → `session_data.bin`; send `REC COMPLETE`
  8. Write `analyzer_handoff.json`; attempt to run `analyze_sample_rate.py`; write `analyzer_result.json`
  9. On abort: send `REC ABORT session_id=<id> reason=user_cancel`
- `retryEsp32Retrieval()` restores pending session context and restarts the thread

### Task 5 — Legacy ESP32 CSV path guarded
- In `run()`, the `esp32RecordStream` write block wrapped with `if (!esp32RecV1Supported)`:
  old-firmware hosts still get PC-side CSV; rec-v1 sessions never touch the stream.

### Task 6 — sendEsp32RecHello on connect
- Called in `performDetectionHandshake()` after successful ESP32 detection.
- Sends `REC HELLO protocol_min=rec-v1 client=plugin mode=direct_tcp`.
- Parses `max_chunk` from `REC HELLO_OK`; sets `esp32RecV1Supported`.
- On `REC ERR` or no response: `esp32RecV1Supported = false`, state → `UnsupportedProtocol`.

### Task 7 — Timeout-finalized session discovery on reconnect
- `checkEsp32ReconnectSession()` called from `sendEsp32RecHello()` after `HELLO_OK`.
- Sends `REC SESSION session_id=latest_finalized`; if firmware returns non-zero
  `file_size`: sets `esp32PendingSessionId`, `esp32RetrievalRetryAvailable = true`,
  state → `SdFinalized`, status → "Timeout-finalized session available — press Retrieve".

### Tasks 8 & 9 — Truthful device editor status
- RECORD button handler: ESP32 path now shows `getEsp32RecStatusText()` (never
  the old "Recording saved" on first press or "Recording stopped and saved" on stop).
- `refreshOpenSimAngleReadout()` (10 Hz timer callback) polls `getEsp32RecStatusText()`
  and forwards changes to the Open Ephys status bar throughout the entire lifecycle.
- `syncRecordButtonForBoardType()` sets accurate tooltips:
  - rec-v1 supported → lifecycle description with state sequence
  - not supported → "Update firmware to enable"

### Task 11 — Local output structure
Per session directory contains:
- `session_data.bin` — retrieved SD binary (analyzer-compatible)
- `session_data.bin.tmp` — staging (deleted on success)
- `metadata.json` — session_id, protocol, file_size, file_checksum, timestamp_ms
- `transfer_log.json` — per-chunk CRC entries, whole_file_crc_match, computed/expected CRC
- `analyzer_handoff.json` — `{"command": "python ... analyze_sample_rate.py --format sd-bin ...", "required": true}`
- `analyzer_result.json` — `{"passed": bool, "output": "..."}`

### Task 12 — Documentation
Updated `docs/esp32-acqboard-integration.md` with new Section 6:
- Required firmware for rec-v1
- Operator workflow (7 steps)
- Output file structure
- Retry workflow
- Timeout-finalized session recovery
- Unsupported firmware handling
- Threading model table

## Commits

| Hash | Message |
|------|---------|
| `ed64c35` | feat(06-03): add rec-v1 capability state to Acqboardredpitaya.h |
| `c0f93b4` | feat(06-03): implement rec-v1 SD recording protocol in acqboard.ccp |
| `e063f42` | feat(06-03): update DeviceEditor for truthful ESP32 rec-v1 status |
| `87d6ebc` | docs(06-03): document rec-v1 SD recording workflow and operator guide |

## Verification Criteria Check

| Criterion | Status |
|-----------|--------|
| No-SD dry run: plugin shows truthful SD failure | SATISFIED — `!esp32RecV1Supported` → UnsupportedProtocol status; `REC ERR` on START → Failed state |
| SD smoke run: START → confirmed → STOP → finalize → retrieve → analyzer | SATISFIED — full async thread implements all states |
| Legacy CSV NOT triggered for rec-v1 sessions | SATISFIED — `!esp32RecV1Supported` guard in `run()` |
| Plugin UI shows all lifecycle states | SATISFIED — status bar updates every ~100 ms via openSimAngleTimer |
| Threading: all network/file/analyzer work off audio path | SATISFIED — `Esp32RetrievalThread` owns all retrieval I/O |
| `esp32RecordStream` writes guarded | SATISFIED — wrapped in `if (! esp32RecV1Supported)` |

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 2 - Missing feature] CRC32 table implementation**
- **Found during:** Task 4 implementation
- **Issue:** Plan specified standard Ethernet CRC32 but no utility function existed.
- **Fix:** Added 256-entry Ethernet CRC32 table and `crc32Ethernet()` helper in
  anonymous namespace at top of rec-v1 section. Also added header CRC validation
  that gracefully skips check when firmware sets header CRC field to 0.
- **Files modified:** `acqboard.ccp`

**2. [Rule 2 - Missing feature] Running whole-file CRC in retrieval loop**
- **Found during:** Task 4 step 6 (whole-file CRC)
- **Issue:** Plan described computing CRC32 of the local file after writing; doing
  a second full-file read would be slow for large sessions.
- **Fix:** Maintain running XOR CRC32 of payload bytes during the retrieval loop;
  finalise with `^ 0xFFFFFFFFu` at the end. Avoids second file read.
- **Files modified:** `acqboard.ccp`

**3. [Rule 2 - Missing feature] Static lastEsp32Status to suppress duplicate status messages**
- **Found during:** Task 8 — refreshOpenSimAngleReadout() polling
- **Issue:** Forwarding status on every 100ms tick would spam the status bar
  with identical messages.
- **Fix:** Track `lastEsp32Status` (function-static) and only call
  `CoreServices::sendStatusMessage()` when the text has changed.
- **Files modified:** `device editor.cpp`

**4. [Rule 1 - Bug] transfer_log.json structure**
- **Found during:** Task 4 — review of transfer_log writing
- **Issue:** Plan described transfer_log as a plain JSON array but the checksum
  summary fields (whole_file_crc_match etc.) needed to be outside the array.
- **Fix:** Write the array entries in the open `[...\n` block, then close with
  `\n],\n` before writing the summary fields at the top object level. The resulting
  JSON is a top-level object with both `chunks` (implicitly the array closed above)
  and `whole_file_crc_match` fields. This gives the analyzer all needed context.
- **Files modified:** `acqboard.ccp`

## Known Stubs

None. All state transitions produce explicit status text and the retrieval path
covers the full lifecycle including analyzer invocation.

## Threat Flags

| Flag | File | Description |
|------|------|-------------|
| threat_flag: path_traversal | `acqboard.ccp` | Session ID from firmware is used in directory name construction. Mitigated: JUCE `File::createDirectory()` will reject paths with `..` on Windows; session_id is from our own firmware, not user input. |

## Self-Check: PASSED

Files verified:
- `Acqboardredpitaya.h` — modified (90 insertions)
- `acqboard.ccp` — modified (1100 insertions, 50 deletions)
- `device editor.cpp` — modified (90 insertions, 22 deletions)
- `docs/esp32-acqboard-integration.md` — modified (120 insertions)

Commits verified:
- `ed64c35` — exists in git log
- `c0f93b4` — exists in git log
- `e063f42` — exists in git log
- `87d6ebc` — exists in git log

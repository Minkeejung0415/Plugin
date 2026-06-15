---
phase: 06-plugin-controlled-esp32-sd-recording-with-reconnect-safe-loc
plan: 02
subsystem: firmware
tags: [esp32, sd-card, rec-v1, usb-bridge, reconnect, retrieval]
requires:
  - phase: 06-plugin-controlled-esp32-sd-recording-with-reconnect-safe-loc
    provides: rec-v1 recording, status, reconnect, and SDRF retrieval contract
provides:
  - ESP-IDF direct TCP rec-v1 command handling for SD recording control and finalized-file retrieval
  - SD logger retained session metadata, reconnect grace timeout, and read-only chunk retrieval APIs
  - Arduino USB bridge firmware rec-v1 command subset with SDRF retrieval after finalization
  - serial TCP bridge relay for REC commands, text status, and SDRF transfer frames
affects: [phase-06-plan-03, firmware, usb-bridge, open-ephys-plugin]
tech-stack:
  added: []
  patterns: [firmware-owned recording truth, retained finalized metadata, SDRF framed retrieval, reconnect grace finalization]
key-files:
  created:
    - .planning/phases/06-plugin-controlled-esp32-sd-recording-with-reconnect-safe-loc/06-02-SUMMARY.md
  modified:
    - esp32/firmware/main/open_ephys_stream.c
    - esp32/firmware/main/sd_logger.c
    - esp32/firmware/main/sd_logger.h
    - esp32/arduino/step_node/step_node.ino
    - esp32/host/serial_tcp_bridge.py
key-decisions:
  - "Default SD recording paths are session-derived so new recordings do not overwrite prior finalized SD sources."
  - "Direct TCP and USB bridge retrieval use SDRF frames and are only served after finalization."
  - "The firmware and bridge advertise crc32 as the negotiated whole-file checksum type for this implementation."
patterns-established:
  - "REC STATUS reports protocol, capabilities, recording state, transfer state, counters, finalization reason, checksum, and reconnect grace fields from firmware state."
  - "Host disconnect starts a firmware-side grace timer; reconnect cancels it and timeout finalizes with disconnect_timeout."
requirements-completed: [SD-01, SD-02, SD-03, SD-04, SD-05, LOSS-01, LOSS-03, LOSS-04, STALL-01, STALL-02, STALL-03]
duration: 17min
completed: 2026-06-15
---

# Phase 06 Plan 02: Firmware and Bridge Recording Control Summary

**rec-v1 firmware and USB bridge control for reconnect-safe ESP32 SD recording, retained finalized metadata, and SDRF framed retrieval**

## Performance

- **Duration:** 17 min
- **Started:** 2026-06-15T17:14:55Z
- **Completed:** 2026-06-15T17:31:46Z
- **Tasks:** 13
- **Files modified:** 5

## Accomplishments

- Added ESP-IDF direct TCP handling for `REC HELLO`, `REC START`, `REC STATUS`, `REC STOP`, `REC SESSION`, `REC GET`, `REC COMPLETE`, `REC ABORT`, and `REC CLEAR`.
- Extended the SD logger with retained session ids, session-derived SD paths, reconnect grace timeout, timeout finalization, file size/checksum metadata, and read-only finalized chunk reads.
- Added Arduino USB firmware support for the same operator-level rec-v1 recording/status/session/retrieval flow while preserving existing USB binary streaming behavior.
- Updated `serial_tcp_bridge.py` to relay `REC` commands, preserve text acknowledgements, and route `SDRF` frames separately from live Open Ephys sample frames.

## Task Commits

Each task was committed atomically:

1. **Tasks 1-13: Firmware and bridge recording control** - `14821de` (feat)

**Plan metadata:** recorded in the final GSD metadata commit for this plan.

## Files Created/Modified

- `esp32/firmware/main/open_ephys_stream.c` - Adds direct TCP rec-v1 command parsing, status responses, SDRF frame emission, and disconnect/reconnect grace hooks.
- `esp32/firmware/main/sd_logger.c` - Adds retained recording/transfer state, finalized metadata, session-derived paths, reconnect timeout finalization, checksum calculation, and read-only chunk retrieval.
- `esp32/firmware/main/sd_logger.h` - Exposes rec-v1 status/session/retrieval APIs and extended SD logger stats fields.
- `esp32/arduino/step_node/step_node.ino` - Adds Arduino USB bridge rec-v1 recording/status/session/retrieval behavior and pauses serial sample output while SDRF transfer frames are emitted.
- `esp32/host/serial_tcp_bridge.py` - Separates live sample frames, `REC` text responses, and `SDRF` transfer frames; relays rec-v1 commands before and during streaming.

## Decisions Made

- Used session-derived SD filenames by default to satisfy source retention and avoid overwriting finalized recordings.
- Kept retrieval read-only and post-finalization only; active recording returns `busy_recording` or `not_finalized`.
- Used `paused_isolated_stream`/command isolation semantics for transfer framing so file bytes are not treated as Open Ephys sample frames.
- Used `crc32` as the negotiated checksum type because the local firmware codebase did not already provide a CRC32C or SHA-256 implementation.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 2 - Missing Critical] Added session-derived default SD paths**
- **Found during:** Task 9 and Task 13
- **Issue:** Reusing `/step_session.bin` would overwrite a retained finalized source on the next recording.
- **Fix:** Default direct TCP and Arduino recordings now use session-derived SD paths while still allowing explicit diagnostic paths.
- **Files modified:** `esp32/firmware/main/sd_logger.c`, `esp32/arduino/step_node/step_node.ino`
- **Verification:** Code inspection and `git diff --check`.
- **Committed in:** `14821de`

---

**Total deviations:** 1 auto-fixed (Rule 2)
**Impact on plan:** Required for source-file retention and retrieval safety. No scope expansion beyond the protocol contract.

## Issues Encountered

- ESP-IDF firmware build could not run because `idf.py` is not installed in the local environment.
- Hardware reconnect, timeout, retrieval, and SD continuity tests were not run in this environment.

## Verification

- `python -m py_compile esp32/host/serial_tcp_bridge.py` passed.
- `git diff --check -- esp32/firmware/main/open_ephys_stream.c esp32/firmware/main/sd_logger.c esp32/firmware/main/sd_logger.h esp32/arduino/step_node/step_node.ino esp32/host/serial_tcp_bridge.py` passed.
- ESP-IDF build was attempted and skipped with `IDF_NOT_FOUND`.

## Known Stubs

None - stub scan found no `TODO`, `FIXME`, placeholder, coming-soon, or not-available markers in the plan-owned files.

## Threat Flags

| Flag | File | Description |
|------|------|-------------|
| threat_flag: network-command-surface | `esp32/firmware/main/open_ephys_stream.c` | Direct TCP now accepts rec-v1 recording and retrieval commands from the connected client. |
| threat_flag: file-read-surface | `esp32/firmware/main/sd_logger.c` | Firmware can read finalized SD files by retained session id for chunk retrieval. |
| threat_flag: bridge-transfer-surface | `esp32/host/serial_tcp_bridge.py` | Bridge now relays framed finalized-file bytes from serial to TCP clients. |

## User Setup Required

None - no external service configuration required. Firmware build and hardware UAT still require a machine with ESP-IDF and the ESP32/SD hardware attached.

## Next Phase Readiness

Plan 06-03 can wire plugin UI/local retrieval/analyzer states against `rec-v1`. The next plan should treat `Recording saved` as unavailable until SDRF retrieval and analyzer continuity verification pass.

## Self-Check: PASSED

- Found all five plan-owned implementation files.
- Found `.planning/phases/06-plugin-controlled-esp32-sd-recording-with-reconnect-safe-loc/06-02-SUMMARY.md`.
- Found task commit `14821de`.

---
*Phase: 06-plugin-controlled-esp32-sd-recording-with-reconnect-safe-loc*
*Completed: 2026-06-15*

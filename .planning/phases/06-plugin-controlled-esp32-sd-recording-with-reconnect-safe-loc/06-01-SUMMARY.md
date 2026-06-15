---
phase: 06-plugin-controlled-esp32-sd-recording-with-reconnect-safe-loc
plan: 01
subsystem: protocol
tags: [esp32, sd-card, open-ephys, recording, retrieval, reconnect, protocol]
requires:
  - phase: 01-measurement-contract-baseline
    provides: SD card data as acquisition ground truth
  - phase: 02-buffered-sd-logger
    provides: board-side SD logger counters and binary SD file foundation
provides:
  - rec-v1 command, status, reconnect, and file-transfer contract
  - binary-safe SDRF retrieval frame definition
  - direct TCP and USB bridge behavioral expectations
  - truthful local verification state model
affects: [phase-06-plan-02, phase-06-plan-03, firmware, usb-bridge, open-ephys-plugin]
tech-stack:
  added: []
  patterns: [protocol-first contract, analyzer-backed saved state, opaque session-token retrieval]
key-files:
  created:
    - .planning/phases/06-plugin-controlled-esp32-sd-recording-with-reconnect-safe-loc/06-PROTOCOL.md
  modified: []
key-decisions:
  - "rec-v1 separates firmware recording state, transfer state, and plugin/local verification state."
  - "Finalized-file retrieval uses binary-safe SDRF frames and cannot share raw bytes with live Open Ephys sample parsing."
  - "The plugin may report Recording saved only after analyzer continuity verification passes."
  - "Retrieval is limited to firmware-issued session ids or latest finalized tokens, never arbitrary SD paths."
patterns-established:
  - "Firmware status must preserve SD finalized, local retrieval, checksum, and analyzer states as separate truth levels."
  - "Direct TCP and USB bridge paths must expose equivalent record/status/retrieval semantics."
requirements-completed: [SD-01, SD-03, SD-04, SD-05, LOSS-03, LOSS-04, STALL-03, OPS-01]
duration: 5min
completed: 2026-06-15
---

# Phase 06 Plan 01: Recording Protocol and Status Contract Summary

**rec-v1 protocol contract for plugin-controlled ESP32 SD recording, reconnect-safe finalization, binary-safe local retrieval, and analyzer-backed saved status**

## Performance

- **Duration:** 5 min
- **Started:** 2026-06-15T17:04:00Z
- **Completed:** 2026-06-15T17:09:16Z
- **Tasks:** 1
- **Files modified:** 1

## Accomplishments

- Created the phase-local `rec-v1` protocol contract covering command negotiation, recording control, status, finalized-session metadata, chunk retrieval, completion, abort, and clear commands.
- Defined the `SDRF` binary retrieval frame with magic/version, frame type, session id, chunk index, byte offset, payload length, total file size, header CRC, payload checksum, EOF, abort, retry, duplicate, and resume behavior.
- Split firmware recording state, transfer state, and plugin/local verification state so downstream work cannot collapse `SD finalized`, `local transfer complete`, `checksum passed`, and `analyzer passed` into one misleading saved state.
- Documented direct TCP and USB bridge expectations, reconnect semantics across failure modes, path safety rules, and analyzer-backed local integrity requirements.

## Task Commits

Each task was committed atomically:

1. **Task 1: Recording protocol and status contract** - `3fce799` (docs)

**Plan metadata:** recorded in the final GSD metadata commit for this plan.

## Files Created/Modified

- `.planning/phases/06-plugin-controlled-esp32-sd-recording-with-reconnect-safe-loc/06-PROTOCOL.md` - Defines the shared `rec-v1` recording, status, reconnect, and retrieval protocol contract.

## Decisions Made

- Used a named `rec-v1` capability negotiation flow so old firmware is clearly unsupported for plugin-controlled SD recording instead of falling back to legacy optimistic behavior.
- Required `SDRF` framed binary transfer and explicit transfer isolation modes so finalized-file bytes cannot be interpreted as live Open Ephys sample data.
- Required retrieval by opaque session id or `latest_finalized` token only, which prevents arbitrary SD filename reads.
- Made analyzer continuity pass the first state that may be presented as fully saved and verified.

## Deviations from Plan

None - plan executed exactly as written.

## Issues Encountered

None.

## Known Stubs

None - stub scan found no placeholder/TODO/FIXME patterns or empty hardcoded UI data in the created protocol document.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

Plan 06-02 can implement firmware and USB bridge behavior against the concrete `rec-v1` contract. Plan 06-03 can wire plugin UI, asynchronous retrieval, and analyzer-backed local verification without redefining the protocol.

## Self-Check: PASSED

- Found `.planning/phases/06-plugin-controlled-esp32-sd-recording-with-reconnect-safe-loc/06-PROTOCOL.md`.
- Found `.planning/phases/06-plugin-controlled-esp32-sd-recording-with-reconnect-safe-loc/06-01-SUMMARY.md`.
- Found task commit `3fce799`.

---
*Phase: 06-plugin-controlled-esp32-sd-recording-with-reconnect-safe-loc*
*Completed: 2026-06-15*

---
phase: 01-measurement-contract-baseline
plan: 01
subsystem: acquisition-reliability
tags: [esp32, sd-card, sample-integrity, stress-test, open-ephys]

requires: []
provides:
  - SD ground-truth file contract
  - Firmware counter contract
  - Acquisition stress-test mode matrix
  - Baseline diagnosis for current SD and stream risk layers
affects: [buffered-sd-logger, stress-harness, open-ephys-stall-isolation, hardware-uat]

tech-stack:
  added: []
  patterns:
    - SD file and firmware-counter contract before implementation
    - Device-side SD log is authoritative; host streams are comparison sinks

key-files:
  created:
    - .planning/phases/01-measurement-contract-baseline/01-BASELINE-CONTRACT.md
  modified: []

key-decisions:
  - "SD-card recordings are the v1.1 ground-truth acquisition record."
  - "No-loss claims require SD sequence continuity plus firmware counters."
  - "Open Ephys, serial/TCP, and UDP outputs are downstream comparison paths only."
  - "Buffered SD logger implementation is deferred to Phase 2."

patterns-established:
  - "Use sequence/timestamp continuity and generated/saved/drop counters as the proof mechanism."
  - "Separate acquisition, filter, SD logging, and streaming modes before interpreting stress results."

requirements-completed: [SD-01, SD-03, SD-04, SD-05, LOSS-01, STRESS-05, STALL-01, STALL-02]

duration: 28 min
completed: 2026-06-12
---

# Phase 1 Plan 01: Measurement Contract & Baseline Summary

**SD ground-truth contract with firmware counters, mode matrix, and baseline failure-layer diagnosis**

## Performance

- **Duration:** 28 min
- **Started:** 2026-06-12T21:20:00Z
- **Completed:** 2026-06-12T21:48:00Z
- **Tasks:** 3 completed
- **Files modified:** 1 created

## Accomplishments

- Defined the SD file/session contract required to prove acquisition continuity.
- Defined firmware counters needed to distinguish generated, saved, dropped, overrun, queued, and latency states.
- Defined the mode matrix needed to isolate acquisition, filter, SD, stream, and full-path bottlenecks.
- Documented current baseline risks from code inspection: per-sample `fwrite`/`fflush`, latest-sample-only `s_latest`, and host-only stress analysis lacking SD ground truth.

## Task Commits

The plan produced one consolidated documentation outcome commit because all three tasks created sections of the same contract artifact:

1. **Tasks 1-3: SD contract, counter/mode contract, baseline diagnosis** - `7404966` (`docs(01-01): define SD baseline contract`)

**Plan metadata:** this summary commit.

## Files Created/Modified

- `.planning/phases/01-measurement-contract-baseline/01-BASELINE-CONTRACT.md` - SD ground-truth contract, required sample/session fields, firmware counters, mode matrix, current failure-layer diagnosis, and future verification checklist.

## Decisions Made

- SD card is the authoritative source for v1.1 acquisition integrity.
- Host/Open Ephys/serial/TCP/UDP paths are comparison sinks and cannot independently prove no sample loss.
- Firmware must expose counters before "no samples lost" can be accepted.
- Phase 2 should implement a buffered/measured SD writer before Phase 3 expands stress tooling.

## Deviations from Plan

None - plan executed exactly as written.

## Issues Encountered

None.

## Verification

- `rg -n "seq|timestamp_us|channel|sample rate|channel layout|filter|stream" .planning/phases/01-measurement-contract-baseline/01-BASELINE-CONTRACT.md` passed.
- `rg -n "generated|saved|dropped|overrun|latency|queue|SD on|filter|stream|frequency|status" .planning/phases/01-measurement-contract-baseline/01-BASELINE-CONTRACT.md` passed.
- `rg -n "fwrite|fflush|s_latest|ground truth|no samples lost|Phase 2|Phase 3" .planning/phases/01-measurement-contract-baseline/01-BASELINE-CONTRACT.md` passed.
- `git diff --check -- .planning/phases/01-measurement-contract-baseline/01-BASELINE-CONTRACT.md` passed.

## User Setup Required

None - no hardware action required for Phase 1.

## Next Phase Readiness

Phase 2 can implement the buffered SD logger against a concrete contract:

- per-sample records must carry `seq`, `timestamp_us`, and channel payload;
- session/header metadata must make SD files self-describing;
- counters must report generated/saved/drop/overrun/latency state;
- SD writes must be decoupled from the acquisition loop.

---
*Phase: 01-measurement-contract-baseline*
*Completed: 2026-06-12*

## Self-Check: PASSED

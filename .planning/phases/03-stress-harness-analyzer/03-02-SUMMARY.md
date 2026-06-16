---
phase: 03-stress-harness-analyzer
plan: "02"
subsystem: stress-harness
tags: [duplicate-detection, payload-fingerprint, summary-refactor, cli]
dependency_graph:
  requires: [03-01]
  provides: [payload-fingerprint-dup-detection, write-summary-helper, rebuild-summary-cli]
  affects: [esp32/host/stress_test_serial.py]
tech_stack:
  added: []
  patterns:
    - payload-fingerprint duplicate detection (compare 22-byte payload bytes between consecutive frames)
    - extracted write_summary() helper for testable summary generation
    - --rebuild-summary CLI flag for offline summary regeneration from SUMMARY.json
key_files:
  modified:
    - esp32/host/stress_test_serial.py
decisions:
  - "dup_payload initialized to 0 before the binary/CSV branch so the CSV code path is safe without re-defining the variable"
  - "Inline last_good/recommended console print block kept in main() as live feedback; only file-writing extracted to write_summary()"
  - "First failure line uses r.note or 'see counters' fallback so it is never blank"
metrics:
  duration: "3min"
  completed: "2026-06-16"
  tasks_completed: 3
  files_modified: 1
---

# Phase 03 Plan 02: Payload-Fingerprint Duplicate Detection and Summary Refactor

Real payload-byte duplicate detection replacing always-zero seq-index check; write_summary() helper with note column and first-failure sentence; --rebuild-summary CLI flag.

## Tasks Completed

| Task | Name | Commit | Files |
|------|------|--------|-------|
| 1 | Extract and fingerprint payload in capture_binary_to_csv() | cad02fb | esp32/host/stress_test_serial.py |
| 2 | Thread dup_payload into test_one_rate() and pass/fail | 43c8b92 | esp32/host/stress_test_serial.py |
| 3 | Add --rebuild-summary flag and write_summary() helper | 3c1b2b0 | esp32/host/stress_test_serial.py |

## What Was Built

### Task 1 — Payload fingerprint in `capture_binary_to_csv()`

Added `prev_payload: bytes | None = None` and `dup_payload: int = 0` before the capture while loop. After the frame header is validated and `frame_size` is known, extracted `payload = bytes(pending[HEADER_SIZE:frame_size])` before `del pending[:frame_size]`. Compared to `prev_payload` and incremented `dup_payload` on match. Changed return type from `tuple[int, float | None]` to `tuple[int, float | None, int]`.

### Task 2 — Thread into `test_one_rate()`

Unpacked the new third return value: `rows, mean_hz_bin, dup_payload = capture_binary_to_csv(...)`. Discarded the always-zero analyzer seq-dup with `_, gap, mean_hz_a, gap_time = run_analyzer(...)`. Set `dup = dup_payload`. Initialized `dup_payload = 0` before the binary/CSV branch so the CSV code path does not raise `NameError`. Added note annotation when `dup_payload > 0`.

### Task 3 — `write_summary()` and `--rebuild-summary`

Extracted the SUMMARY.md/JSON writing block into `write_summary(results, port, baud)`. Added `note` column to the Markdown table header and every row. Added `First failure: {hz} Hz — {note}` line after the table. Added `--rebuild-summary` CLI argument that reads `SUMMARY.json`, reconstructs `list[RateResult]`, calls `write_summary()`, and exits 0. Replaced the inline writing block in `main()` with a single `write_summary(results, port, args.baud)` call.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] Initialized dup_payload before binary/CSV branch**
- **Found during:** Task 2 implementation
- **Issue:** `dup_payload` was only assigned in the `else` (binary) branch. The note check `if binary_mode is not False and dup_payload:` after the branch would raise `NameError` on the CSV path because the variable would be undefined.
- **Fix:** Added `dup_payload = 0` immediately before the `if binary_mode is False:` branch so both paths are safe.
- **Files modified:** esp32/host/stress_test_serial.py
- **Commit:** 43c8b92

## Known Stubs

None. All changes wire real data through to the output.

## Threat Flags

None. No new network endpoints, auth paths, or file access patterns introduced. The `--rebuild-summary` flag reads from `RESULTS_DIR / "SUMMARY.json"` (a local file this script itself writes).

## Self-Check: PASSED

- esp32/host/stress_test_serial.py: modified (confirmed)
- cad02fb: exists (confirmed)
- 43c8b92: exists (confirmed)
- 3c1b2b0: exists (confirmed)
- All static checks pass: prev_payload, dup_payload, write_summary, rebuild_summary, First failure

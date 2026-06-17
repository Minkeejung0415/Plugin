---
status: partial
phase: 04-open-ephys-stall-isolation
source: [04-VERIFICATION.md]
started: 2026-06-17
updated: 2026-06-17
---

# Human UAT: Phase 04 Magnetometer Channel Verification

## Current Test

Awaiting device + Open Ephys GUI verification.

## Tests

### 1. Handshake Reports 14 Channels

expected: USB bridge or direct Wi-Fi path reports `14 channels` and `OK CHANNELS:14`.
result: pending

### 2. Open Ephys Channel List Includes Magnetometer

expected: Open Ephys Acq Board channel list includes `mx`, `my`, and `mz`.
result: pending

### 3. Saved CSV Includes Magnetometer

expected: recorded/exported or retrieved CSV includes `mx,my,mz` columns or values.
result: pending

### 4. Magnetometer Values Change

expected: moving the IMU or applying a nearby magnet changes `mx`, `my`, or `mz`; flat zeros are classified as `mag=FALLBACK` / `mag_ok=0` or a downstream export issue.
result: pending

## Summary

total: 4
passed: 0
issues: 0
pending: 4
skipped: 0
blocked: 0

## Gaps

None yet. Awaiting human/device test results.


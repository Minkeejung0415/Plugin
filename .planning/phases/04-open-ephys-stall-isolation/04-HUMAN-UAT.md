---
status: partial
phase: 04-open-ephys-stall-isolation
source: [04-VERIFICATION.md]
started: 2026-06-17
updated: 2026-06-18
---

# Human UAT: Phase 04

## Tests

### 1. UDP Capability And Channels

expected: Plugin handshake reports UDP samples plus TCP control, and Open Ephys shows all 14 channels including `mx`, `my`, and `mz`.
result: pending

### 2. Normal Receiver SD Integrity

expected: At 950 and 1000 Hz with filter, SD, and UDP enabled for 60 seconds, finalized SD has zero gaps, duplicates, drops, or write errors.
result: pending

### 3. Receiver-Stall Isolation

expected: Pausing or disconnecting Open Ephys may increase UDP drops, but SD remains perfect and reaches at least 97% of the corresponding stream-off baseline.
result: pending

### 4. Datagram Recovery

expected: Open Ephys resumes on later valid UDP datagrams after loss or receiver interruption without restarting acquisition.
result: pending

### 5. Magnetometer End-To-End

expected: Moving the IMU or applying a nearby magnet changes `mx`, `my`, or `mz`, and the saved/retrieved data includes those channels.
result: pending

## Summary

total: 5
passed: 0
issues: 0
pending: 5
skipped: 0
blocked: 0

## Gaps

Awaiting device, SD card, network, and Open Ephys bench results.

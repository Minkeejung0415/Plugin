---
phase: 07-espnow-master-timestamp-sync
milestone: v1.1
created: 2026-06-15
status: planned
---

# Phase 07 Context: ESP-NOW Master Broadcast & Slave Timestamp Sync

## Goal

Enable ESP-NOW one-way broadcast from the master ESP32 (currently connected via USB) to
slave nodes so all devices report samples with a common time reference. A slave node
corrects its local `esp_timer_get_time()` timestamps by the measured offset to the master
clock before logging to SD or streaming to Open Ephys.

## Background

The codebase already contains a skeleton:
- `esp32/arduino/step_node/step_node.ino`: `#define ENABLE_ESPNOW false`, `NODE_IS_MASTER true`,
  `sendEspNowSync()` broadcasts `{seq, esp_timer_get_time()}` (correct), but `onEspNowRecv()`
  is an empty stub and no offset is ever computed or applied.
- `esp32/firmware/main/espnow_sync.c`: master broadcasts `sync_packet_t{seq, time_us}`,
  slave stores `master_seq`/`master_time_us`, but `clock_offset_us` is never computed or applied.
- Kconfig: `CONFIG_STEP_ENABLE_ESPNOW` defaults to `n`; `CONFIG_STEP_NODE_MASTER` defaults to `y`.

## Decisions

- D-01: The master is the currently-connected ESP32 in USB serial bench mode (`NODE_IS_MASTER true`).
- D-02: ESP-NOW uses broadcast address (0xFF*6) so slave MAC addresses do not need to be
  hardcoded; up to 20 unencrypted slaves are supported.
- D-03: Master broadcasts `{seq, local_time_us}` on every acquisition sample (already implemented).
- D-04: Slave computes `clock_offset_us = master_time_us - local_time_us_at_recv` on first
  receipt and applies it to all subsequent timestamps.
- D-05: In USB+ESPNOW mode (ENABLE_TCP=false, ENABLE_ESPNOW=true), WiFi is initialized in
  STA mode on a fixed channel (ESPNOW_WIFI_CHANNEL=1) without attempting an AP join.
  This avoids the 45-second STA timeout blocking USB-mode startup.
- D-06: All nodes must use the same WiFi channel for ESP-NOW; ESPNOW_WIFI_CHANNEL=1 is the
  default. A slave on a TCP AP must be on the same channel (set via esp_wifi_set_channel).
- D-07: The offset is not filtered (no PLL/EWMA) in this phase; single-packet correction is
  sufficient for the ~100 Hz synchronization requirement.
- D-08: When ENABLE_ESPNOW is enabled, STATUS output includes espnow role, sync status, and
  current clock_offset_us so the operator can verify sync is working.
- D-09: The ESP-IDF firmware (espnow_sync.c) receives the same fix for completeness, even
  though the Arduino sketch is the active path.

## Key Files

| File | Role |
|------|------|
| `esp32/arduino/step_node/step_node.ino` | Primary: enable ESPNOW, fix recv callback, apply offset |
| `esp32/firmware/main/espnow_sync.c` | Secondary: fix clock_offset_us computation |
| `esp32/firmware/main/main.c` | Secondary: apply offset on slave sample.timestamp_us |

## Out of Scope

- Multi-hop or mesh networks
- PLL/EWMA timestamp filtering (single-packet correction only)
- Slave-to-master feedback or two-way sync (NTP-style round-trip)
- Changing slave MAC registration (broadcast covers all slaves)
- New Open Ephys plugin changes

## Success Criteria

1. Master ESP32 boots in USB mode with ESPNOW enabled and prints `ESP-NOW ready (role=master)`.
2. Slave ESP32 receives sync packets and prints `ESPNOW: offset=NNN us sync=1`.
3. Slave SD log `time_us` and OE header `offset` reflect master clock rather than slave local clock.
4. STATUS command on both nodes shows `espnow=enabled`.

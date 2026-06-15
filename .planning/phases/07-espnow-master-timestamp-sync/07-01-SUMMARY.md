---
phase: 07-espnow-master-timestamp-sync
plan: 01
subsystem: esp32-firmware
tags: [espnow, sync, arduino, esp-idf, timestamp]
dependency_graph:
  requires: []
  provides: [espnow-master-broadcast, slave-clock-offset]
  affects: [step_node.ino, espnow_sync.c, main.c]
tech_stack:
  added: []
  patterns: [ESP-NOW broadcast, clock offset correction, #if guard pattern]
key_files:
  created: []
  modified:
    - esp32/arduino/step_node/step_node.ino
    - esp32/firmware/main/espnow_sync.c
    - esp32/firmware/main/main.c
decisions:
  - "D-01: NODE_IS_MASTER=true for the connected ESP32; not changed"
  - "D-02: Broadcast address 0xFF*6 used; no per-slave MAC hardcoding"
  - "D-03: Master broadcasts {seq, local_time_us} on every sample via sendEspNowSync()"
  - "D-04: Slave offset = pkt->time_us - recv_us; applied to all subsequent timestamps"
  - "D-05: USB+ESPNOW skips 45 s STA join; fast STA-mode-only WiFi init in setupEspNow()"
  - "D-06: ESPNOW_WIFI_CHANNEL=1; all nodes must match"
  - "D-07: Single-packet offset, no filtering (acceptable for 100 Hz / 10 ms period)"
  - "D-08: STATUS output includes espnow role, sync_received flag, offset_us, last_seq"
  - "D-09: espnow_sync.c also receives the clock_offset_us fix for ESP-IDF firmware parity"
metrics:
  duration: "~15 minutes"
  completed: "2026-06-15"
  tasks_completed: 10
  tasks_total: 10
  files_modified: 3
---

# Phase 07 Plan 01: ESP-NOW Master Broadcast & Slave Timestamp Sync Summary

**One-liner:** ESP-NOW enabled with fast STA-mode WiFi init, slave clock-offset computed from broadcast timestamps, and applied in both OE header and SD log paths.

## Tasks Completed

| Task | Description | Status |
|------|-------------|--------|
| A-1 | Enable ENABLE_ESPNOW true + add ESPNOW_WIFI_CHANNEL 1 constant | Done |
| A-2 | Add g_clock_offset_us, g_espnow_sync_received, g_espnow_last_seq globals | Done |
| A-3 | Fix useWifi() to exclude ESPNOW (removes 45 s AP join gate) | Done |
| A-4 | Add #include "esp_wifi.h" to ENABLE_ESPNOW includes block | Done |
| A-5 | Rewrite setupEspNow() with fast STA WiFi init, error checks, broadcast peer | Done |
| A-6 | Implement onEspNowRecv() to compute clock offset; fix onEspNowSent() signature | Done |
| A-7 | Apply clock offset in fillOeHeader() for slave nodes | Done |
| A-8 | Apply clock offset in logSd() for slave nodes | Done |
| A-9 | Add ESPNOW role/sync/offset/last_seq to printAcqStatus() STATUS output | Done |
| B-1 | espnow_sync.c: compute clock_offset_us in espnow_recv() callback | Done |
| B-2 | main.c: apply sync.clock_offset_us to sample.timestamp_us before sending | Done |

## Files Changed

### esp32/arduino/step_node/step_node.ino
- `ENABLE_ESPNOW` flipped from `false` to `true`
- `ESPNOW_WIFI_CHANNEL 1` constant added
- `#include "esp_wifi.h"` added inside `#if ENABLE_ESPNOW` block
- Three new guarded globals: `g_clock_offset_us`, `g_espnow_sync_received`, `g_espnow_last_seq`
- `useWifi()` now returns only `ENABLE_TCP` (ESPNOW excluded — handles its own WiFi init)
- `setupEspNow()` rewritten: STA-mode WiFi init when `wifi_up` is false, `esp_wifi_set_channel()` call, `esp_now_init()` error check, broadcast peer with `peer.channel=0`
- `onEspNowRecv()` now computes `g_clock_offset_us = pkt->time_us - recv_us` on slave
- `onEspNowSent()` signature corrected from `wifi_tx_info_t*` to `const uint8_t *mac` (matches `esp_now_send_cb_t`)
- `fillOeHeader()`: offset applied to `t_us` before packing into `hdr->offset`
- `logSd()`: offset applied to `t_us` before assigning `rec.time_us`
- `printAcqStatus()`: ESPNOW status line printed after main STATUS line

### esp32/firmware/main/espnow_sync.c
- `espnow_recv()`: captures `recv_us = esp_timer_get_time()` before storing master fields; sets `s_state.clock_offset_us = (int32_t)(p->time_us - recv_us)` when `!s_master`

### esp32/firmware/main/main.c
- `acquisition_task()`: slave path applies `sync.clock_offset_us` to `sample.timestamp_us` before `espnow_sync_on_local_frame()` and the downstream stream/SD writes

## Key Decisions Applied

All nine decisions from the plan frontmatter (D-01 through D-09) were applied exactly as specified. No architectural changes were needed beyond the plan scope.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] Fixed onEspNowSent() callback signature**
- **Found during:** Task A-6
- **Issue:** Existing stub used `const wifi_tx_info_t *info` which is not the correct type for `esp_now_register_send_cb`. The correct Arduino ESP32 `esp_now_send_cb_t` signature is `void (*)(const uint8_t *mac_addr, esp_now_send_status_t status)`.
- **Fix:** Changed parameter to `const uint8_t *mac` — noted explicitly in the plan as a required fix.
- **Files modified:** `esp32/arduino/step_node/step_node.ino`
- **Commit:** 4f040f9

No other deviations. Plan executed exactly as written.

## Known Stubs

None — all ESP-NOW paths are fully wired. The `sendEspNowSync()` function (already present in the file pre-plan) broadcasts `{seq, time_us}` on every sample; `onEspNowRecv()` now consumes it.

## Verification Checklist (Requires Hardware Flash)

- [ ] Master boots: Serial Monitor shows `ESP-NOW ready (role=master ch=1)`
- [ ] Master STATUS: `ESPNOW role=master ch=1 sync=0 offset_us=0 last_seq=0`
- [ ] No 45 s WiFi delay when USB_OPEN_EPHYS_MODE=true + ENABLE_ESPNOW=true
- [ ] Slave boots, receives first packet: `ESPNOW role=slave sync=1 offset_us=<non-zero>`
- [ ] Slave SD log `time_us` values align with master within ~500 us
- [ ] Single-node build (ENABLE_ESPNOW false) compiles cleanly
- [ ] ESP-IDF firmware: espnow_sync.c compiles; s_state.clock_offset_us populated on slave

## Commit

`4f040f9` — feat(07-01): enable ESP-NOW master broadcast and slave timestamp sync

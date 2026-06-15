# Phase 06: Plugin-Controlled ESP32 SD Recording With Reconnect-Safe Local Retrieval - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md; this log preserves the alternatives considered.

**Date:** 2026-06-15
**Phase:** 06-plugin-controlled-esp32-sd-recording-with-reconnect-safe-loc
**Areas discussed:** phase scope, recording ownership, reconnect behavior

---

## Phase Scope

| Option | Description | Selected |
|--------|-------------|----------|
| New Phase | Create a focused phase for plugin-controlled SD recording, reconnect grace, and file retrieval without overloading the stress-test phase. | yes |
| Fold Into Phase 4 | Treat it as part of Open Ephys stall isolation because it crosses plugin, bridge, and connection-loss behavior. | |
| Plan Phase 3 | Add it to the stress harness/analyzer phase, even though file transfer and reconnect policy are broader than stress testing. | |

**User's choice:** New Phase
**Notes:** The feature crosses plugin UI, command protocol, bridge behavior, firmware recording state, reconnect handling, and local file retrieval. It should not be hidden inside stress testing.

---

## Recording Ownership

| Option | Description | Selected |
|--------|-------------|----------|
| Board-side SD primary | Plugin RECORD controls ESP32 SD logging; local PC receives a completed file after stop/finalize. | yes |
| PC-side CSV primary | Plugin saves live stream to local disk during acquisition. | |
| Hybrid live mirror | ESP32 saves SD while plugin also writes a live local mirror. | |

**User's choice:** Board-side SD primary
**Notes:** Current PC-side saving is not sufficient because the requirement is to save on the ESP32 SD card when the plugin RECORD button is pressed.

---

## Reconnect Behavior

| Option | Description | Selected |
|--------|-------------|----------|
| Graceful autonomous logging | On connection loss, ESP32 keeps logging for a configurable one-to-two-minute grace period, resumes normally if reconnected, otherwise flushes/closes and stops. | yes |
| Stop immediately on disconnect | ESP32 stops recording as soon as host connection drops. | |
| Log indefinitely | ESP32 keeps logging until manual intervention even if the host never reconnects. | |

**User's choice:** Graceful autonomous logging
**Notes:** The reconnect timeout must be defined near the top of the relevant firmware file so it is easy to change.

---

## the agent's Discretion

- Choose the concrete file retrieval mechanism after research, provided acquisition remains SD-first and transfer includes status/integrity checks.
- Choose exact UI wording, as long as the plugin distinguishes command sent, recording confirmed, finalized, retrieved, and failed states.

## Deferred Ideas

- Multi-board synchronized SD retrieval.
- Full SD-card model benchmarking.
- Replay-from-SD into Open Ephys as a separate workflow unless it is the simplest retrieval format.

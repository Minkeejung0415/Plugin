# Roadmap: OpenSim View Angle on Trigger

**Project:** Open Ephys Red Pitaya Plugin  
**Created:** 2026-06-10  
**Phases:** 5  
**Requirement coverage:** 12/12 v1 requirements mapped ✓

## Overview

| # | Phase | Goal | Requirements | Success Criteria |
|---|-------|------|--------------|------------------|
| 1 | View Config & Simbody Spike | Prove camera preset + sidecar pipeline | DISP-01, DISP-03 | 3 |
| 2 | Plugin View Selector | Operator selects and persists preset | VIEW-01, VIEW-02, VIEW-03 | 3 |
| 3 | Trigger Wiring | Trigger fires view command to OpenSim | TRIG-01, TRIG-02, TRIG-03, OPS-01 | 4 |
| 4 | Display Label | Show angle name beside sim clock | DISP-02 | 3 |
| 5 | Integration Verify | End-to-end validation, no regressions | DISP-04, OPS-02 | 3 |

---

## Phase Details

### Phase 1: View Config & Simbody Spike
**Goal:** Establish the plugin→Python view command contract and prove Simbody camera presets work on the target OpenSim 4.5 install.

**Requirements:** DISP-01, DISP-03

**Success Criteria:**
1. `opensim_view_config.json` schema documented with `view_id`, `label`, `trigger_ts`, `seq` fields
2. Python watcher in `opensim_live_realtime.py` applies at least two distinct camera presets when config changes
3. Preset switch completes within 200 ms of file update while live UDP stream continues

**Key files:** `opensim_live_realtime.py`, new `opensim_view_presets.py` (optional module), `docs/opensim-view-config.md`

**Research flags:** Simbody Python camera API enumeration required in plan-phase

---

### Phase 2: Plugin View Selector & Persistence
**Goal:** Add operator-facing view preset selection in the device editor with save/load support.

**Requirements:** VIEW-01, VIEW-02, VIEW-03

**Success Criteria:**
1. ComboBox labeled "View Angle" visible in Red Pitaya device editor alongside OpenSim controls
2. All seven presets selectable and shown by human-readable label
3. Selected preset restores correctly after saving and reloading plugin settings XML

**Key files:** `device editor.cpp`, `device editor.h`

**UI hint:** yes

---

### Phase 3: Trigger → View Command Wiring
**Goal:** Connect acquisition triggers to view config writes so the selected angle applies when triggered.

**Requirements:** TRIG-01, TRIG-02, TRIG-03, OPS-01

**Success Criteria:**
1. Handling `ACQBOARD TRIGGER` writes config with current preset to OpenSim work directory
2. Atomic write pattern prevents partial reads; sequence increments on each event
3. IMU UDP packet timing unchanged (no measurable sample rate impact)
4. "Apply View" utility button writes config without requiring external trigger (debug/preview)

**Key files:** `devicethread.cpp`, `acqboard.ccp`, `devices/redpitaya/AcqBoardRedPitaya.h`

---

### Phase 4: Display Label Beside Sim Clock
**Goal:** Show the active view preset name adjacent to the Simbody simulation timer on the OpenSim Live window.

**Requirements:** DISP-02

**Success Criteria:**
1. When a preset is active, its label (e.g. "Lateral Right") is visible in the Simbody visualizer window beside the sim time display
2. Label updates immediately when preset changes via trigger or manual apply
3. If native Simbody text overlay is unavailable, fallback (window title or documented alternative) is implemented and noted in docs

**Key files:** `opensim_live_realtime.py`

**UI hint:** yes

---

### Phase 5: Integration Verification & Documentation
**Goal:** Validate end-to-end workflow and confirm no regression to live OpenSim streaming.

**Requirements:** DISP-04, OPS-02

**Success Criteria:**
1. Manual test: select preset → start OpenSim Live → Play acquisition → fire trigger → view and label change correctly
2. Live IK skeleton continues updating through view switches (no freeze > 1 s)
3. Operator documentation covers preset list, trigger workflow, config path, and troubleshooting

**Key files:** `docs/opensim-view-config.md`, `docs/opensim-udp-v2.md` (cross-reference)

---

## Phase Ordering Rationale

1. **Spike first** — Simbody camera API uncertainty is the highest technical risk  
2. **UI before trigger** — operator must select preset before trigger can mean anything  
3. **Trigger before label polish** — core value is view-on-trigger; label completes UX  
4. **Verify last** — integration tests need all components wired  

## Dependencies

```
Phase 1 ──► Phase 2 ──► Phase 3 ──► Phase 4 ──► Phase 5
              │           │
              └───────────┴── both depend on Phase 1 config contract
```

---
*Roadmap created: 2026-06-10*

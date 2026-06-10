# Roadmap: Joint Angle Display on Trigger

**Project:** Open Ephys Red Pitaya Plugin  
**Created:** 2026-06-10  
**Revised:** 2026-06-10 (scope correction)  
**Phases:** 5  
**Requirement coverage:** 14/14 v1 requirements mapped ✓

## Overview

| # | Phase | Goal | Requirements | Success Criteria |
|---|-------|------|--------------|------------------|
| 1 | Display Config Contract & Spike | 1/3 | In Progress|  |
| 2 | Plugin Joint Selector | Operator selects and persists joint list | JOIN-01, JOIN-02, JOIN-03, JOIN-04 | 4 |
| 3 | Trigger Wiring | Trigger fires display config to OpenSim | TRIG-01, TRIG-02, TRIG-03, OPS-01 | 4 |
| 4 | Filtered Display | Show only selected angles beside clock | DISP-01, DISP-02, DISP-05 | 3 |
| 5 | Integration Verify | End-to-end validation, no regressions | DISP-04, OPS-02 | 3 |

---

## Phase Details

### Phase 1: Display Config Contract & Spike
**Goal:** Establish the plugin→Python joint-display command contract and prove filtered angle readout can render beside the Simbody sim clock.

**Requirements:** DISP-03

**Success Criteria:**
1. `opensim_joint_display_config.json` schema documented with `joints` (coordinate name list), `trigger_ts`, `seq` fields
2. Python watcher in `opensim_live_realtime.py` applies a 2-joint filter when config changes (e.g. only `knee_angle_r`, `hip_flexion_r`)
3. Filtered values update within 200 ms of file update while live UDP stream continues

**Locked defaults (2026-06-10):** curated catalog (flexion only), max 6 joints, abbreviated 1-decimal HUD (`knee_r: 42.1°`), trigger applies current selection (Phase 3)

**Key files:** `opensim_live_realtime.py`, new `opensim_joint_catalog.py` (optional module), `docs/opensim-joint-display-config.md`

**Research flags:** Simbody text overlay / status API beside `setShowSimTime` — enumerate in plan-phase

---

### Phase 2: Plugin Joint Selector & Persistence
**Goal:** Add operator-facing joint coordinate multi-select in the device editor with save/load support.

**Requirements:** JOIN-01, JOIN-02, JOIN-03, JOIN-04

**Success Criteria:**
1. Multi-select UI (checkbox list or equivalent) labeled for joint angle display visible in Red Pitaya device editor
2. Curated coordinate catalog covers instrumented-limb joints (hips, knees, ankles, pelvis minimum)
3. Selected joints restore correctly after saving and reloading plugin settings XML
4. Optional: joints near active `streamSensorNames` segments are visually grouped or pre-checked

**Key files:** `device editor.cpp`, `device editor.h`

**UI hint:** yes

---

### Phase 3: Trigger → Display Config Wiring
**Goal:** Connect acquisition triggers to display config writes so the selected joint filter applies when triggered.

**Requirements:** TRIG-01, TRIG-02, TRIG-03, OPS-01

**Success Criteria:**
1. Handling `ACQBOARD TRIGGER` writes config with current joint selection to OpenSim work directory (`kOpenSimWorkDir`)
2. Atomic write pattern prevents partial reads; sequence increments on each event
3. IMU UDP packet timing unchanged (no measurable sample rate impact)
4. "Apply Display" utility button writes config without requiring external trigger (debug/preview)

**Key files:** `devicethread.cpp`, `acqboard.ccp`, `devices/redpitaya/AcqBoardRedPitaya.h`

---

### Phase 4: Filtered Display Beside Sim Clock
**Goal:** Render only selected joint angle values adjacent to the Simbody simulation timer on the OpenSim Live window.

**Requirements:** DISP-01, DISP-02, DISP-05

**Success Criteria:**
1. Only configured coordinates appear in the on-screen readout; unselected joints are hidden
2. Values update live from IK state (`coord_set.get(name).getValue(state)`) in degrees beside sim time
3. Empty selection shows no coordinate dump (clean default)
4. If native Simbody text overlay is unavailable, fallback (window title augmentation or documented alternative) is implemented

**Key files:** `opensim_live_realtime.py`

**UI hint:** yes

---

### Phase 5: Integration Verification & Documentation
**Goal:** Validate end-to-end workflow and confirm no regression to live OpenSim streaming.

**Requirements:** DISP-04, OPS-02

**Success Criteria:**
1. Manual test: select joints → start OpenSim Live → Play acquisition → fire trigger → only selected angles appear beside clock
2. Live IK skeleton continues updating through display filter changes (no freeze > 1 s)
3. Operator documentation covers joint catalog, trigger workflow, config path, sensor→segment context, and troubleshooting

**Key files:** `docs/opensim-joint-display-config.md`, `docs/opensim-udp-v2.md` (cross-reference)

---

## Phase Ordering Rationale

1. **Spike first** — Simbody on-screen text beside sim time is the highest display risk  
2. **UI before trigger** — operator must select joints before trigger can apply a filter  
3. **Trigger before display polish** — core value is filter-on-trigger; display rendering completes UX  
4. **Verify last** — integration tests need all components wired  

## Dependencies

```
Phase 1 ──► Phase 2 ──► Phase 3 ──► Phase 4 ──► Phase 5
              │           │
              └───────────┴── both depend on Phase 1 config contract
```

## Superseded Planning

Prior roadmap phases for camera/view presets (Simbody camera API, `opensim_view_config.json`, anatomical view labels) are **void**. Do not implement.

---
*Roadmap created: 2026-06-10*  
*Revised: 2026-06-10 — joint angle display control*

# Project Research Summary

**Project:** Open Ephys Red Pitaya Plugin — Joint Angle Display on Trigger  
**Domain:** Biomechanics / electrophysiology acquisition + real-time OpenSim visualization  
**Researched:** 2026-06-10  
**Revised:** 2026-06-10 (scope correction)  
**Confidence:** HIGH

## Executive Summary

This is a **brownfield increment** on an existing C++/JUCE Open Ephys plugin and Python 3.8 OpenSim Live bridge. The stack is proven; the new feature adds a **joint coordinate multi-select**, **trigger-driven display filter**, and **filtered angle readout beside the Simbody sim clock**.

**Scope correction:** Prior research assumed camera/view presets. User intent is **which joint angles to display**, not Simbody camera angles. Multiple IMU sensors on body segments produce IK coordinates; showing all joints is visually noisy.

Recommended approach: **JSON sidecar config** (`opensim_joint_display_config.json`) written by the plugin on trigger, watched by a lightweight Python thread in `opensim_live_realtime.py`, filtering which `coord_set` values render on-screen — without modifying UDP v2 quaternion packets. Primary risk is Simbody on-screen text API coverage — mitigated by an early display spike and window-title fallback.

## Key Findings

### Recommended Stack

Keep existing C++ plugin + Python 3.8 + OpenSim 4.5. Add only a JSON sidecar channel and joint multi-select UI. No new dependencies.

### Expected Features

**Must have:** joint multi-select, trigger apply, filtered angles beside clock, stream non-regression  
**Defer v2:** per-TTL-line joint sets, named preset profiles, auto-select from active sensors

### Architecture

Plugin UI → XML → trigger handler → JSON sidecar → Python watcher → filtered coordinate HUD beside sim time. Quaternion UDP unchanged.

### Critical Pitfalls

1. Simbody text overlay API gaps — spike early  
2. UDP format regression — use sidecar only  
3. File write races — atomic rename + sequence numbers  
4. Confusing sensors with joints — document coordinate names vs IMU segment names  
5. Display clutter — cap joints and format degrees consistently

## Implications for Roadmap

### Phase 1: Display Config Contract & Spike
**Rationale:** Validate on-screen text beside sim time before UI/trigger work  
**Delivers:** JSON schema, Python reader, 2-joint filter working on display  
**Avoids:** Pitfall 1, 2

### Phase 2: Plugin Joint Selector & Persistence
**Rationale:** Operator must choose joints before trigger  
**Delivers:** Multi-select in device editor, XML save/load, curated catalog  
**Avoids:** Pitfall 4

### Phase 3: Trigger → Display Config Wiring
**Rationale:** Core value — filter applies on trigger  
**Delivers:** Write sidecar on `ACQBOARD TRIGGER`, optional "Apply Now" test  
**Avoids:** Pitfall 3

### Phase 4: Filtered Display Beside Sim Clock
**Rationale:** Completes UX requirement  
**Delivers:** Live coordinate values for selected joints only, beside sim time  
**Avoids:** Pitfall 5

### Phase 5: Integration Verification & Docs
**Rationale:** Brownfield — regression risk on live pipeline  
**Delivers:** Manual test checklist, update docs  

## Confidence Assessment

| Area | Confidence | Notes |
|------|------------|-------|
| Stack | HIGH | Existing repo |
| Features | HIGH | User correction clarifies intent |
| Architecture | HIGH | Sidecar pattern straightforward |
| Pitfalls | MEDIUM | Simbody overlay API TBD |

## Sources

- Repository: `opensim_live_realtime.py`, `devicethread.cpp`, `device editor.cpp`, `docs/opensim-udp-v2.md`, `opensim_sensor_map.json`
- OpenSim 4.5 `CoordinateSet` / SimbodyVisualizer (display API verification needed at execution)

---
*Research completed: 2026-06-10*  
*Revised: 2026-06-10 — joint angle display*  
*Ready for roadmap: yes*

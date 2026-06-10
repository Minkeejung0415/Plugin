# Project Research Summary

**Project:** Open Ephys Red Pitaya Plugin — OpenSim View Angle on Trigger  
**Domain:** Biomechanics / electrophysiology acquisition + real-time OpenSim visualization  
**Researched:** 2026-06-10  
**Confidence:** HIGH

## Executive Summary

This is a **brownfield increment** on an existing C++/JUCE Open Ephys plugin and Python 3.8 OpenSim Live bridge. The stack is proven; the new feature adds a **view preset selector**, **trigger-driven camera application**, and **on-screen angle label** beside the Simbody sim clock.

Recommended approach: **JSON sidecar config** (`opensim_view_config.json`) written by the plugin on trigger, watched by a lightweight Python thread in `opensim_live_realtime.py`, applying Simbody camera presets without modifying UDP v2 quaternion packets. Primary risk is Simbody Python camera API coverage — mitigated by an early API spike and window-title fallback.

## Key Findings

### Recommended Stack

Keep existing C++ plugin + Python 3.8 + OpenSim 4.5. Add only a JSON sidecar channel and UI ComboBox. No new dependencies.

### Expected Features

**Must have:** preset selector, trigger apply, label beside clock, stream non-regression  
**Defer v2:** per-TTL-line preset maps, animated transitions

### Architecture

Plugin UI → XML → trigger handler → JSON sidecar → Python watcher → Simbody camera + label. Quaternion UDP unchanged.

### Critical Pitfalls

1. Simbody camera API gaps — spike early  
2. UDP format regression — use sidecar only  
3. File write races — atomic rename + sequence numbers  
4. Label overlay limitations — title fallback  

## Implications for Roadmap

### Phase 1: View Config Contract & Simbody Spike
**Rationale:** Validate camera API before UI/trigger work  
**Delivers:** JSON schema, Python reader, camera preset map (at least 2 presets working)  
**Avoids:** Pitfall 1, 2

### Phase 2: Plugin View Selector & Persistence
**Rationale:** Operator must choose preset before trigger  
**Delivers:** ComboBox in device editor, XML save/load  
**Avoids:** Pitfall 6

### Phase 3: Trigger → View Command Wiring
**Rationale:** Core value — view changes on trigger  
**Delivers:** Write sidecar on `ACQBOARD TRIGGER`, optional "Apply Now" test  
**Avoids:** Pitfall 3, 4, 7

### Phase 4: Live Display Label Beside Sim Clock
**Rationale:** Completes UX requirement  
**Delivers:** Label rendering adjacent to sim time in Simbody window  
**Avoids:** Pitfall 5

### Phase 5: Integration Verification & Docs
**Rationale:** Brownfield — regression risk on live pipeline  
**Delivers:** Manual test checklist, update docs  

## Confidence Assessment

| Area | Confidence | Notes |
|------|------------|-------|
| Stack | HIGH | Existing repo |
| Features | HIGH | Clear user request |
| Architecture | HIGH | Sidecar pattern straightforward |
| Pitfalls | MEDIUM | Simbody overlay API TBD |

## Sources

- Repository: `opensim_live_realtime.py`, `devicethread.cpp`, `device editor.cpp`, `docs/opensim-udp-v2.md`
- OpenSim 4.5 SimbodyVisualizer (API verification needed at execution)

---
*Research completed: 2026-06-10*  
*Ready for roadmap: yes*

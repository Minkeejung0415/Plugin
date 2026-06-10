# Project State

**Project:** Joint Angle Display on Trigger  
**Initialized:** 2026-06-10  
**Scope revised:** 2026-06-10  
**Current phase:** 1 (not started)  
**Status:** Planning revised — ready for `/gsd:plan-phase 1`

## Project Reference

See: `.planning/PROJECT.md` (updated 2026-06-10)

**Core value:** Trigger fires → OpenSim shows only pre-selected joint angles beside sim timer  
**Current focus:** Phase 1 — Display Config Contract & Spike

## Progress

| Phase | Name | Status | Plans |
|-------|------|--------|-------|
| 1 | Display Config & Spike | ○ Pending | 0/0 |
| 2 | Plugin Joint Selector | ○ Pending | 0/0 |
| 3 | Trigger Wiring | ○ Pending | 0/0 |
| 4 | Filtered Display | ○ Pending | 0/0 |
| 5 | Integration Verify | ○ Pending | 0/0 |

**Overall:** ○○○○○ 0/5 phases

## Requirements Snapshot

- v1: 14 requirements
- Mapped: 14/14 ✓
- Validated (existing codebase): 7 capabilities documented in PROJECT.md

## Active Decisions

| Decision | Status |
|----------|--------|
| Joint display via JSON sidecar (`opensim_joint_display_config.json`), not UDP v2 change | Locked for v1 |
| Coordinate-name selection (not camera presets) | Locked — user correction |
| Curated joint catalog for v1 UI | Locked for v1 |
| OpenSim Live only (not Gen Motion) | Locked for v1 |

## Superseded (Do Not Implement)

| Prior assumption | Status |
|------------------|--------|
| Camera/view angle presets | Void — user correction |
| `opensim_view_config.json` | Void — replaced by `opensim_joint_display_config.json` |
| Simbody camera API spike | Void — not needed for this milestone |

## Blockers

| Blocker | Owner | Notes |
|---------|-------|-------|
| Simbody on-screen text beside sim time API unverified | Phase 1 spike | May need window-title fallback |
| Open questions on joint catalog defaults & trigger semantics | User | See PROJECT.md Open Questions |

## Session Notes

- Brownfield repo: C++/JUCE plugin + Python 3.8 OpenSim Live
- IK coordinates read via `model.getCoordinateSet()` after `ikSolver.assemble(state)`
- Sensor → segment: `SENSORS` / `opensim_sensor_map.json`; joints are IK output coordinates
- Trigger path: `ACQBOARD TRIGGER <line> <ms>` in `devicethread.cpp`
- Prior planning incorrectly assumed camera presets; corrected 2026-06-10

---
*State updated: 2026-06-10 — scope correction*

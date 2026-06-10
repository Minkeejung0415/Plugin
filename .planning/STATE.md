# Project State

**Project:** OpenSim View Angle on Trigger  
**Initialized:** 2026-06-10  
**Current phase:** 1 (not started)  
**Status:** Planning complete — ready for `/gsd:plan-phase 1`

## Project Reference

See: `.planning/PROJECT.md` (updated 2026-06-10)

**Core value:** Trigger fires → OpenSim shows pre-selected camera view with angle label beside sim timer  
**Current focus:** Phase 1 — View Config & Simbody Spike

## Progress

| Phase | Name | Status | Plans |
|-------|------|--------|-------|
| 1 | View Config & Simbody Spike | ○ Pending | 0/0 |
| 2 | Plugin View Selector | ○ Pending | 0/0 |
| 3 | Trigger Wiring | ○ Pending | 0/0 |
| 4 | Display Label | ○ Pending | 0/0 |
| 5 | Integration Verify | ○ Pending | 0/0 |

**Overall:** ○○○○○ 0/5 phases

## Requirements Snapshot

- v1: 12 requirements
- Mapped: 12/12 ✓
- Validated (existing codebase): 5 capabilities documented in PROJECT.md

## Active Decisions

| Decision | Status |
|----------|--------|
| Camera presets via JSON sidecar (not UDP v2 change) | Locked for v1 |
| Seven presets + Default | Locked for v1 |
| OpenSim Live only (not Gen Motion) | Locked for v1 |

## Blockers

| Blocker | Owner | Notes |
|---------|-------|-------|
| Simbody label overlay API unverified | Phase 1 spike | May need window-title fallback |
| Interactive questioning gates skipped in subagent | User | Confirm preset list and trigger source if different from assumptions |

## Session Notes

- Brownfield repo: C++/JUCE plugin + Python 3.8 OpenSim Live
- Existing OpenSim Live window uses `setShowSimTime(True)` — label goes beside this
- Trigger path: `ACQBOARD TRIGGER <line> <ms>` in `devicethread.cpp`

---
*State initialized: 2026-06-10*

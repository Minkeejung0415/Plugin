---
gsd_state_version: 1.0
milestone: v1.0
milestone_name: milestone
current_phase: 01
status: ready_to_plan
last_updated: 2026-06-10T16:50:25.125Z
progress:
  total_phases: 5
  completed_phases: 0
  total_plans: 3
  completed_plans: 3
  percent: 0
stopped_at: Phase 01 complete (3/3) — ready to discuss Phase 2
---

# Project State

**Project:** Joint Angle Display on Trigger  
**Initialized:** 2026-06-10  
**Scope revised:** 2026-06-10  
**Defaults locked:** 2026-06-10  
**Current phase:** 2
**Status:** Ready to plan

## Project Reference

See: `.planning/PROJECT.md` (updated 2026-06-10)

**Core value:** Trigger fires → OpenSim shows only pre-selected joint angles beside sim timer  
**Current focus:** Phase 2 — plugin joint selector & persistence

## Progress

| Phase | Name | Status | Plans |
|-------|------|--------|-------|
| 1 | Display Config & Spike | ◐ Planned | 3/3 |
| 2 | Plugin Joint Selector | ○ Pending | 0/0 |
| 3 | Trigger Wiring | ○ Pending | 0/0 |
| 4 | Filtered Display | ○ Pending | 0/0 |
| 5 | Integration Verify | ○ Pending | 0/0 |

**Overall:** ◐○○○○ 0/5 phases executed, 1/5 planned

## Phase 1 Plans

| Plan | Wave | Objective |
|------|------|-----------|
| 01-01 | 1 | Schema doc + `opensim_joint_catalog.py` |
| 01-02 | 2 | Config watcher thread in `opensim_live_realtime.py` |
| 01-03 | 3 | Simbody HUD spike + manual 200 ms verification |

**Phase dir:** `.planning/phases/01-display-config-contract-spike/`

## Requirements Snapshot

- v1: 14 requirements
- Mapped: 14/14 ✓
- Phase 1 covers: DISP-03

## Locked Defaults (2026-06-10)

| # | Decision | Value |
|---|----------|-------|
| 1 | Joint catalog | Curated instrumented-limb list only |
| 2 | Trigger behavior | Apply current checkbox selection on trigger |
| 3 | Max on screen | 6 joints |
| 4 | DOF per joint | Primary flexion only |
| 5 | Display format | Abbreviated labels, 1 decimal (`knee_r: 42.1°`) |

## Active Decisions

| Decision | Status |
|----------|--------|
| Joint display via JSON sidecar (`opensim_joint_display_config.json`), not UDP v2 change | Locked for v1 |
| Coordinate-name selection (not camera presets) | Locked — user correction |
| Curated joint catalog for v1 UI | Locked 2026-06-10 |
| OpenSim Live only (not Gen Motion) | Locked for v1 |
| Max 6 joints, flexion-only, abbreviated HUD | Locked 2026-06-10 |
| Trigger applies current selection (not per-TTL presets) | Locked 2026-06-10 |

## Superseded (Do Not Implement)

| Prior assumption | Status |
|------------------|--------|
| Camera/view angle presets | Void — user correction |
| `opensim_view_config.json` | Void — replaced by `opensim_joint_display_config.json` |
| Simbody camera API spike | Void — not needed for this milestone |

## Blockers

| Blocker | Owner | Notes |
|---------|-------|-------|
| Simbody on-screen text beside sim time API unverified | Phase 1 plan 01-03 | Spike + window-title fallback in plan |
| ~~Open questions on joint catalog defaults~~ | — | **Resolved** — defaults locked 2026-06-10 |

## Session Notes

- Brownfield repo: C++/JUCE plugin + Python 3.8 OpenSim Live
- IK coordinates read via `model.getCoordinateSet()` after `ikSolver.assemble(state)`
- Sensor → segment: `SENSORS` / `opensim_sensor_map.json`; joints are IK output coordinates
- Trigger path: `ACQBOARD TRIGGER <line> <ms>` in `devicethread.cpp`
- Phase 1 planning complete: CONTEXT, RESEARCH, 3 PLAN.md files

---
*State updated: 2026-06-10 — Phase 1 planned, defaults locked*

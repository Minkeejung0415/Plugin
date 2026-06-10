# Open Ephys Red Pitaya Plugin — OpenSim View Angle on Trigger

## What This Is

An Open Ephys acquisition-board plugin for Red Pitaya hardware that streams IMU orientation data to OpenSim Live for real-time musculoskeletal visualization. This milestone adds the ability to **select a camera/view angle** in the plugin and **apply that view when a trigger fires**, with the **active angle name shown beside the Simbody visualizer clock** on the OpenSim Live display.

The plugin already launches OpenSim Live, forwards quaternion UDP packets on port 5000, and supports TTL/broadcast triggers via `ACQBOARD TRIGGER`. The new work connects view selection → trigger event → OpenSim camera + on-screen label.

## Core Value

When an experiment trigger fires during live acquisition, the operator immediately sees the skeleton from the **pre-selected anatomical view**, with the **view name visible next to the sim timer** — no manual camera adjustment during time-critical sessions.

## Requirements

### Validated

- ✓ Red Pitaya IMU acquisition and quaternion streaming to OpenSim (UDP v2 on port 5000) — existing
- ✓ OpenSim Live launch from device editor (`OpenSim Live` button) — existing
- ✓ Simbody visualizer with simulation time display (`setShowSimTime(True)`) — existing in `opensim_live_realtime.py`
- ✓ Digital output / broadcast trigger path (`ACQBOARD TRIGGER <line> <ms>`) — existing in `devicethread.cpp`
- ✓ Device editor persistence via XML settings — existing pattern in `device editor.cpp`

### Active

- [ ] Operator can select a view angle preset in the plugin UI before/during acquisition
- [ ] Selected view angle is applied to the OpenSim Simbody visualizer when a trigger fires
- [ ] Active view angle label is displayed beside the sim timer/clock on the OpenSim Live window
- [ ] View selection persists across plugin save/load (device settings XML)
- [ ] Angle change works with the existing OpenSim Live UDP stream (no regression to IK/visualization)

### Out of Scope

- Custom free-rotate camera control from the plugin — Simbody window mouse controls remain manual fallback only
- Joint-angle readouts as the "angle" display — user request refers to **camera/view angle**, not coordinate DOF values
- OpenSim Gen Motion offline pipeline changes — scope is **OpenSim Live display** only
- Multi-monitor or embedded web viewer — Simbody native window only
- Triggering view changes from external software beyond existing `ACQBOARD TRIGGER` broadcast — v1 uses current trigger path only

## Context

**Brownfield codebase** (`C:\Users\justi\Plugin`):

| Layer | Technology | Key files |
|-------|------------|-----------|
| Plugin (C++/JUCE) | Open Ephys GUI plugin | `device editor.cpp`, `devicethread.cpp`, `acqboard.ccp`, `devices/redpitaya/AcqBoardRedPitaya.h` |
| OpenSim bridge | Python 3.8 + OpenSim 4.5 SDK | `opensim_live_realtime.py` |
| Transport | UDP localhost:5000 | `docs/opensim-udp-v2.md` |
| Triggers | Broadcast `ACQBOARD TRIGGER <ttlLine> <durationMs>` | `devicethread.cpp:340-355` |

**OpenSim Live display today:** `opensim_live_realtime.py` creates a SimbodyVisualizer (`viz.setShowSimTime(True)`), window title `"Connect OpenSim - Red Pitaya 8-IMU"`. No programmatic camera presets or angle label overlay exist.

**Trigger flow today:** External or GUI-initiated broadcast → `DeviceThread::handleBroadcastMessage` → `acquisitionBoard->triggerDigitalOutput()`. No coupling to OpenSim view state.

**Questioning decisions** (from feature request + codebase review; interactive gates deferred to parent session):

| Topic | Decision |
|-------|----------|
| "Angle" meaning | Camera/view presets (Anterior, Posterior, Lateral L/R, Superior, Isometric) — not joint coordinates |
| Trigger source | Existing `ACQBOARD TRIGGER` broadcast and plugin TTL output path |
| Display location | Simbody visualizer window, adjacent to sim time (clock) — may require Simbody API overlay or window-title/status augmentation if native overlay unavailable |
| Config transport | Extend UDP side-channel or write `opensim_view_config.json` atomically on trigger; prefer lightweight JSON sidecar read by Python watcher to avoid breaking v2 quaternion packet format |
| Preset count v1 | Six standard anatomical views + Default (current camera) |

## Constraints

- **Tech stack**: Must stay on C++/JUCE plugin + Python 3.8 OpenSim 4.5 — no new runtime dependencies
- **Compatibility**: Must not break existing UDP v2 quaternion packets consumed by `opensim_live_realtime.py`
- **Platform**: Windows paths hardcoded for OpenSim install (`C:\OpenSim 4.5\...`) — follow existing conventions
- **OpenSim API**: SimbodyVisualizer camera APIs vary by OpenSim 4.5 Python bindings — implementation must verify available methods at plan time
- **Performance**: View switch on trigger must not stall the ~20 Hz visualizer loop (`OPENSIM_LIVE_VISUALIZER_RATE`)

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Camera view presets, not joint angles | User said "angle seen on the opensim" beside timer — matches visualizer camera | — Pending |
| JSON sidecar for view commands (`opensim_view_config.json`) | Avoids breaking UDP v2 packet layout; Python can poll/watch cheaply | — Pending |
| Hook trigger in plugin broadcast handler + optional UI test button | Reuses proven `ACQBOARD TRIGGER` path | — Pending |
| OpenSim Live only (not Gen Motion) | Display requirement targets live Simbody window with clock | — Pending |
| Six presets + Default for v1 | Covers standard biomechanics review angles without custom UI complexity | — Pending |

## Evolution

This document evolves at phase transitions and milestone boundaries.

**After each phase transition** (via `/gsd-transition`):
1. Requirements invalidated? → Move to Out of Scope with reason
2. Requirements validated? → Move to Validated with phase reference
3. New requirements emerged? → Add to Active
4. Decisions to log? → Add to Key Decisions
5. "What This Is" still accurate? → Update if drifted

**After each milestone** (via `/gsd:complete-milestone`):
1. Full review of all sections
2. Core Value check — still the right priority?
3. Audit Out of Scope — reasons still valid?
4. Update Context with current state

---
*Last updated: 2026-06-10 after initialization*

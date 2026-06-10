# Open Ephys Red Pitaya Plugin — Joint Angle Display on Trigger

## What This Is

An Open Ephys acquisition-board plugin for Red Pitaya hardware that streams IMU orientation data to OpenSim Live for real-time musculoskeletal visualization. Multiple IMU sensors are attached to body segments; IK solves joint coordinates continuously. **Today, too many joint angles appear on the OpenSim Live display and create visual noise.** This milestone lets the operator **select which joint angles to show**, **apply that filter when a trigger fires** (or on demand), and **render the filtered values beside the Simbody simulation timer/clock**.

The plugin already launches OpenSim Live, forwards quaternion UDP packets on port 5000, and supports TTL/broadcast triggers via `ACQBOARD TRIGGER`. The new work connects joint selection in the plugin → trigger/config event → filtered on-screen joint-angle readout in OpenSim Live.

## Core Value

When an experiment trigger fires during live acquisition, the operator sees **only the pre-selected joint angles** (e.g. right knee flexion, left hip flexion) **beside the sim timer** — not the full noisy coordinate dump from every model joint.

## Requirements

### Validated

- ✓ Red Pitaya IMU acquisition and quaternion streaming to OpenSim (UDP v2 on port 5000) — existing
- ✓ OpenSim Live launch from device editor (`OpenSim Live` button) — existing
- ✓ Simbody visualizer with simulation time display (`setShowSimTime(True)`) — existing in `opensim_live_realtime.py`
- ✓ IK produces joint coordinate values from IMU orientations — existing
- ✓ Sensor → body-segment mapping — existing
- ✓ Digital output / broadcast trigger path — existing
- ✓ Device editor persistence via XML settings — existing pattern
- ✓ Operator joint multi-select in device editor (Phase 2)
- ✓ Joint selection persists in plugin XML (Phase 2)
- ✓ Trigger and Apply Display write `opensim_joint_display_config.json` (Phase 3)
- ✓ OpenSim Live filtered HUD beside sim clock (Phases 1, 4)
- ✓ Operator documentation (Phase 5)

### Active

- [ ] Manual hardware UAT: live trigger → HUD filter, IK continuity (DISP-04 sign-off)

### Out of Scope

- **Camera/view angle presets** — prior assumption; explicitly not this milestone
- Custom free-rotate camera control from the plugin — Simbody mouse controls remain manual
- OpenSim Gen Motion offline pipeline changes — scope is **OpenSim Live display** only
- Multi-monitor or embedded web viewer — Simbody native window only
- UDP v2 packet format changes — use JSON sidecar for display config
- Triggering display changes from external software beyond existing `ACQBOARD TRIGGER` broadcast — v1 uses current trigger path only

## Context

**Brownfield codebase** (`C:\Users\justi\Plugin`):

| Layer | Technology | Key files |
|-------|------------|-----------|
| Plugin (C++/JUCE) | Open Ephys GUI plugin | `device editor.cpp`, `devicethread.cpp`, `acqboard.ccp`, `devices/redpitaya/AcqBoardRedPitaya.h` |
| OpenSim bridge | Python 3.8 + OpenSim 4.5 SDK | `opensim_live_realtime.py` |
| Transport | UDP localhost:5000 | `docs/opensim-udp-v2.md` |
| Sensor mapping | JSON sidecar | `opensim_sensor_map.json` |
| Triggers | Broadcast `ACQBOARD TRIGGER <ttlLine> <durationMs>` | `devicethread.cpp:340-355` |

**Sensor → segment mapping today:** Red Pitaya sensor index maps distal → proximal (`tibia_r_imu`, `femur_r_imu`, `pelvis_imu`, `torso_imu`, …). Override via `opensim_sensor_map.json` `sensor_slots`. IK then solves model coordinates (`hip_flexion_r`, `knee_angle_r`, `pelvis_tilt`, etc.) from orientation references.

**OpenSim Live display today:** `opensim_live_realtime.py` runs IK at ~20 Hz, calls `viz.setShowSimTime(True)`, and logs sample coordinates to console (`[COORD …]`). No filtered on-screen joint-angle HUD beside the clock exists yet; all coordinates are effectively available and the display is noisy without filtering.

**Trigger flow today:** External or GUI-initiated broadcast → `DeviceThread::handleBroadcastMessage` → `acquisitionBoard->triggerDigitalOutput()`. No coupling to OpenSim display state.

**Scope correction (2026-06-10):** User clarified "angle beside the clock" means **joint angle values**, not Simbody camera presets. Prior camera/view planning is superseded.

| Topic | Decision |
|-------|----------|
| "Angle" meaning | **Joint coordinate values** (degrees) from IK — not camera/view presets |
| Selection unit | OpenSim coordinate names (e.g. `knee_angle_r`, `hip_flexion_l`) |
| Trigger source | Existing `ACQBOARD TRIGGER` broadcast and plugin TTL output path |
| Display location | Simbody visualizer window, adjacent to sim time (clock) |
| Config transport | `opensim_joint_display_config.json` sidecar (atomic write); do not modify UDP v2 |
| Default joint catalog v1 | **Locked:** curated instrumented-limb list only (primary flexion DOFs); full model enum deferred |
| Trigger semantics v1 | **Locked:** apply current checkbox selection on trigger (not per-TTL preset maps) |
| Max joints on screen | **Locked:** 6 joints maximum |
| DOF per joint | **Locked:** primary flexion only (e.g. `hip_flexion_r`, not adduction/rotation) |
| Display format | **Locked:** abbreviated labels, 1 decimal, e.g. `knee_r: 42.1°` |

## Constraints

- **Tech stack**: Must stay on C++/JUCE plugin + Python 3.8 OpenSim 4.5 — no new runtime dependencies
- **Compatibility**: Must not break existing UDP v2 quaternion packets consumed by `opensim_live_realtime.py`
- **Platform**: Windows paths hardcoded for OpenSim install (`C:\OpenSim 4.5\...`) — follow existing conventions
- **OpenSim API**: Simbody text overlay beside sim time may need API spike; window-title or status-line fallback acceptable
- **Performance**: Display filter update on trigger must not stall the ~20 Hz visualizer loop (`OPENSIM_LIVE_VISUALIZER_RATE`)

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Joint angle display, not camera presets | User correction overrides prior camera assumption | — Pending |
| JSON sidecar for display config (`opensim_joint_display_config.json`) | Avoids breaking UDP v2 packet layout; Python can poll/watch cheaply | — Pending |
| Coordinate-name selection in plugin | IK output is joint coordinates; sensors map to segments, not display labels directly | — Pending |
| Hook trigger in plugin broadcast handler + optional UI test button | Reuses proven `ACQBOARD TRIGGER` path | — Pending |
| OpenSim Live only (not Gen Motion) | Display requirement targets live Simbody window with clock | — Pending |
| Curated joint list for v1 UI | Full Rajagopal coordinate set is large; start with instrumented-limb coordinates | Locked 2026-06-10 |
| Trigger applies current selection | User default; per-TTL preset maps deferred to v2 | Locked 2026-06-10 |
| Max 6 joints, flexion-only, abbreviated HUD | Readability beside sim clock | Locked 2026-06-10 |

## Locked Defaults (2026-06-10)

User approved **proceed with defaults**:

| # | Topic | Decision |
|---|-------|----------|
| 1 | Joint catalog | Curated instrumented-limb list only (not all model coordinates) |
| 2 | Trigger behavior | Apply current checkbox selection on trigger |
| 3 | Max on screen | 6 joints |
| 4 | DOF per joint | Primary flexion only |
| 5 | Display format | Abbreviated labels with 1 decimal, e.g. `knee_r: 42.1°` |

These defaults govern catalog design, UI limits, config schema, and HUD formatting across all phases.

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
*Last updated: 2026-06-10 — scope corrected to joint angle display control*

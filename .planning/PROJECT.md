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
- ✓ IK produces joint coordinate values from IMU orientations (`model.getCoordinateSet()`, `coord_set.get(name).getValue(state)`) — existing in `opensim_live_realtime.py`
- ✓ Sensor → body-segment mapping (`SENSORS`, `SENSOR_CHAIN_UP`, `opensim_sensor_map.json`) — existing
- ✓ Digital output / broadcast trigger path (`ACQBOARD TRIGGER <line> <ms>`) — existing in `devicethread.cpp`
- ✓ Device editor persistence via XML settings — existing pattern in `device editor.cpp`

### Active

- [ ] Operator can select which joint coordinates to display in the plugin UI before/during acquisition
- [ ] Selected joint list persists across plugin save/load (device settings XML)
- [ ] On trigger (or manual apply), plugin writes the selected joint list to OpenSim work directory config
- [ ] OpenSim Live displays **only** selected joint angle values, formatted beside the sim timer/clock
- [ ] Display filter changes without regressing live IK/visualization or UDP v2 streaming

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
| Default joint catalog v1 | Curated list from Rajagopal model coordinates near instrumented segments; full model enum deferred |

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
| Curated joint list for v1 UI | Full Rajagopal coordinate set is large; start with instrumented-limb coordinates | — Pending |

## Open Questions (User)

1. **Default joint set** — All model coordinates vs only joints near active IMUs vs user-defined list?
2. **Trigger semantics** — Does trigger *apply the current checkbox selection* or *switch to a pre-mapped trigger-specific set*?
3. **Max joints on screen** — Cap for readability beside the clock (e.g. 4–6)?
4. **Per-joint DOF** — Show primary flexion only, or all DOFs (flexion/adduction/rotation) per selected joint?
5. **Units/format** — Degrees with fixed precision? Include joint label abbreviations?

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

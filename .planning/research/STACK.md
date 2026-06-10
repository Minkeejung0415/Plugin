# Stack Research — Joint Angle Display Feature

**Project:** Open Ephys Red Pitaya Plugin — Joint Angle Display on Trigger  
**Researched:** 2026-06-10  
**Revised:** 2026-06-10 (scope correction)  
**Confidence:** HIGH (brownfield — stack already in production use)

## Current Stack (Validated in Repo)

| Component | Version / Path | Role |
|-----------|----------------|------|
| Open Ephys GUI plugin | C++ / JUCE | Device editor, acquisition, UDP send |
| Red Pitaya firmware | C (`RedPitaya_justin.c`) | IMU read, sensor fusion |
| OpenSim SDK | 4.5 (`C:\OpenSim 4.5\`) | IK + Simbody visualizer |
| Python bridge | 3.8 only | `opensim_live_realtime.py` |
| UDP transport | port 5000 | Quaternion v2 packets |
| Sensor mapping | `opensim_sensor_map.json` | RP index → body segment IMU frames |
| Trigger bus | Open Ephys broadcast | `ACQBOARD TRIGGER` |

## Recommended Additions (No New Runtimes)

| Addition | Purpose | Rationale |
|----------|---------|-----------|
| `opensim_joint_display_config.json` sidecar | Plugin → Python display filter commands | Avoids UDP packet format change; atomic write pattern |
| Multi-select joint UI in `device editor.cpp` | Coordinate selection | Matches existing Red Pitaya UI patterns (`sensorSelectCombo`, checkboxes) |
| `model.getCoordinateSet()` reads | Live angle values | Already used in `opensim_live_realtime.py` for debug logging |
| Optional: file watcher thread in Python | Low-latency filter apply | Poll interval 50–100 ms acceptable for trigger events |

## Joint Display Implementation Notes

IK pipeline: UDP quaternions → orientation references → `InverseKinematicsSolver.assemble(state)` → coordinate values via `coord_set.get(name).getValue(state)`.

Display renders **filtered subset** of coordinates beside `viz.setShowSimTime(True)` output. Simbody Python bindings may expose limited text overlay APIs — Phase 1 spike required.

**Sensor vs joint distinction:**
- **Sensors** (`torso_imu`, `femur_r_imu`, …) — IMU attachment frames, drive IK inputs
- **Joints/coordinates** (`hip_flexion_r`, `knee_angle_r`, …) — IK output values shown to operator

**Fallback if overlay unavailable:** Augment window title with selected joint values, or composite status string documented in ops guide.

## What NOT to Use

- **Simbody camera preset APIs** — superseded; not this milestone
- **Web/Electron viewer** — out of scope, adds stack
- **OpenSim GUI (OpenSim64.exe) automation** — Live mode uses embedded Simbody only
- **Extra UDP port** — unnecessary if JSON sidecar works

## Confidence

| Area | Level | Notes |
|------|-------|-------|
| Plugin stack | HIGH | Code in repo |
| Python/OpenSim IK | HIGH | `opensim_live_realtime.py` proven |
| Simbody on-screen text API | MEDIUM | Needs spike in Phase 1 plan |
| Sensor→segment mapping | HIGH | `opensim_sensor_map.json` + `SENSOR_CHAIN_UP` |

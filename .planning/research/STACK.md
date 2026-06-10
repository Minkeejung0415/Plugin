# Stack Research — OpenSim View Angle Feature

**Project:** Open Ephys Red Pitaya Plugin — OpenSim View on Trigger  
**Researched:** 2026-06-10  
**Confidence:** HIGH (brownfield — stack already in production use)

## Current Stack (Validated in Repo)

| Component | Version / Path | Role |
|-----------|----------------|------|
| Open Ephys GUI plugin | C++ / JUCE | Device editor, acquisition, UDP send |
| Red Pitaya firmware | C (`RedPitaya_justin.c`) | IMU read, sensor fusion |
| OpenSim SDK | 4.5 (`C:\OpenSim 4.5\`) | IK + Simbody visualizer |
| Python bridge | 3.8 only | `opensim_live_realtime.py` |
| UDP transport | port 5000 | Quaternion v2 packets |
| Trigger bus | Open Ephys broadcast | `ACQBOARD TRIGGER` |

## Recommended Additions (No New Runtimes)

| Addition | Purpose | Rationale |
|----------|---------|-----------|
| `opensim_view_config.json` sidecar | Plugin → Python view commands | Avoids UDP packet format change; atomic write pattern |
| `ComboBox` in `device editor.cpp` | View preset selection | Matches existing Red Pitaya UI patterns (`sensorSelectCombo`) |
| SimbodyVisualizer camera API | Apply preset on trigger | Native OpenSim — no third-party viewer |
| Optional: `ReadWriteLock` + file watcher thread in Python | Low-latency view apply | Poll interval 50–100 ms acceptable for trigger events |

## Camera Preset Implementation Notes

OpenSim 4.x SimbodyVisualizer exposes camera control via C++ API (`setCameraTransform`, ground/fixed camera modes). Python bindings may expose subset via `model.updVisualizer().updSimbodyVisualizer()`.

**Fallback if overlay unavailable:** Append angle name to window title alongside existing title; use Simbody status line if API supports it.

## What NOT to Use

- **Web/Electron viewer** — out of scope, adds stack
- **OpenSim GUI (OpenSim64.exe) automation** — Live mode uses embedded Simbody only
- **Extra UDP port** — unnecessary if JSON sidecar works; defer unless file locking issues arise

## Confidence

| Area | Level | Notes |
|------|-------|-------|
| Plugin stack | HIGH | Code in repo |
| Python/OpenSim | HIGH | `opensim_live_realtime.py` proven |
| Simbody camera Python API | MEDIUM | Needs spike in Phase 1 plan |

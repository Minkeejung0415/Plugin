<!-- GSD:project-start source:PROJECT.md -->
## Project

**Open Ephys Red Pitaya Plugin — OpenSim View Angle on Trigger**

An Open Ephys acquisition-board plugin for Red Pitaya hardware that streams IMU orientation data to OpenSim Live for real-time musculoskeletal visualization. This milestone adds the ability to **select a camera/view angle** in the plugin and **apply that view when a trigger fires**, with the **active angle name shown beside the Simbody visualizer clock** on the OpenSim Live display.

The plugin already launches OpenSim Live, forwards quaternion UDP packets on port 5000, and supports TTL/broadcast triggers via `ACQBOARD TRIGGER`. The new work connects view selection → trigger event → OpenSim camera + on-screen label.

**Core Value:** When an experiment trigger fires during live acquisition, the operator immediately sees the skeleton from the **pre-selected anatomical view**, with the **view name visible next to the sim timer** — no manual camera adjustment during time-critical sessions.

### Constraints

- **Tech stack**: Must stay on C++/JUCE plugin + Python 3.8 OpenSim 4.5 — no new runtime dependencies
- **Compatibility**: Must not break existing UDP v2 quaternion packets consumed by `opensim_live_realtime.py`
- **Platform**: Windows paths hardcoded for OpenSim install (`C:\OpenSim 4.5\...`) — follow existing conventions
- **OpenSim API**: SimbodyVisualizer camera APIs vary by OpenSim 4.5 Python bindings — implementation must verify available methods at plan time
- **Performance**: View switch on trigger must not stall the ~20 Hz visualizer loop (`OPENSIM_LIVE_VISUALIZER_RATE`)
<!-- GSD:project-end -->

<!-- GSD:stack-start source:research/STACK.md -->
## Technology Stack

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
<!-- GSD:stack-end -->

<!-- GSD:conventions-start source:CONVENTIONS.md -->
## Conventions

Conventions not yet established. Will populate as patterns emerge during development.
<!-- GSD:conventions-end -->

<!-- GSD:architecture-start source:ARCHITECTURE.md -->
## Architecture

Architecture not yet mapped. Follow existing patterns found in the codebase.
<!-- GSD:architecture-end -->

<!-- GSD:skills-start source:skills/ -->
## Project Skills

No project skills found. Add skills to any of: `.claude/skills/`, `.agents/skills/`, `.cursor/skills/`, `.github/skills/`, or `.codex/skills/` with a `SKILL.md` index file.
<!-- GSD:skills-end -->

<!-- GSD:workflow-start source:GSD defaults -->
## GSD Workflow Enforcement

Before using Edit, Write, or other file-changing tools, start work through a GSD command so planning artifacts and execution context stay in sync.

Use these entry points:
- `/gsd-quick` for small fixes, doc updates, and ad-hoc tasks
- `/gsd-debug` for investigation and bug fixing
- `/gsd-execute-phase` for planned phase work

Do not make direct repo edits outside a GSD workflow unless the user explicitly asks to bypass it.
<!-- GSD:workflow-end -->



<!-- GSD:profile-start -->
## Developer Profile

> Profile not yet configured. Run `/gsd-profile-user` to generate your developer profile.
> This section is managed by `generate-claude-profile` -- do not edit manually.
<!-- GSD:profile-end -->

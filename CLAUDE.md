<!-- GSD:project-start source:PROJECT.md -->
## Project

**Open Ephys Red Pitaya Plugin — Joint Angle Display on Trigger**

An Open Ephys acquisition-board plugin for Red Pitaya hardware that streams IMU orientation data to OpenSim Live for real-time musculoskeletal visualization. Multiple IMU sensors attach to body segments; IK solves joint coordinates continuously. This milestone lets the operator **select which joint angles to display**, **apply that filter when a trigger fires**, with **filtered values shown beside the Simbody simulation timer/clock** on the OpenSim Live display.

The plugin already launches OpenSim Live, forwards quaternion UDP packets on port 5000, and supports TTL/broadcast triggers via `ACQBOARD TRIGGER`. The new work connects joint selection → trigger event → filtered on-screen joint-angle readout.

**Core Value:** When an experiment trigger fires during live acquisition, the operator sees **only the pre-selected joint angles** beside the sim timer — not the full noisy coordinate dump from every model joint.

### Constraints

- **Tech stack**: Must stay on C++/JUCE plugin + Python 3.8 OpenSim 4.5 — no new runtime dependencies
- **Compatibility**: Must not break existing UDP v2 quaternion packets consumed by `opensim_live_realtime.py`
- **Platform**: Windows paths hardcoded for OpenSim install (`C:\OpenSim 4.5\...`) — follow existing conventions
- **OpenSim API**: Simbody on-screen text beside sim time may need API spike; fallback acceptable
- **Performance**: Display filter update on trigger must not stall the ~20 Hz visualizer loop (`OPENSIM_LIVE_VISUALIZER_RATE`)
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
| Sensor mapping | `opensim_sensor_map.json` | RP index → body segment |
| Trigger bus | Open Ephys broadcast | `ACQBOARD TRIGGER` |
## Recommended Additions (No New Runtimes)
| Addition | Purpose | Rationale |
|----------|---------|-----------|
| `opensim_joint_display_config.json` sidecar | Plugin → Python display filter | Avoids UDP packet format change; atomic write pattern |
| Joint multi-select in `device editor.cpp` | Coordinate selection | Matches existing Red Pitaya UI patterns |
| `model.getCoordinateSet()` filtered reads | Live angle values | Already used in IK loop |
| Optional: file watcher thread in Python | Low-latency filter apply | Poll interval 50–100 ms acceptable for trigger events |
## Joint Display Implementation Notes
IK: UDP quats → orientations → `InverseKinematicsSolver` → `coord_set.get(name).getValue(state)`. Display filtered subset beside `setShowSimTime(True)`.
## What NOT to Use
- **Camera/view presets** — superseded scope
- **Web/Electron viewer** — out of scope, adds stack
- **OpenSim GUI (OpenSim64.exe) automation** — Live mode uses embedded Simbody only
- **Extra UDP port** — unnecessary if JSON sidecar works
## Confidence
| Area | Level | Notes |
|------|-------|-------|
| Plugin stack | HIGH | Code in repo |
| Python/OpenSim | HIGH | `opensim_live_realtime.py` proven |
| Simbody on-screen text API | MEDIUM | Needs spike in Phase 1 plan |
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

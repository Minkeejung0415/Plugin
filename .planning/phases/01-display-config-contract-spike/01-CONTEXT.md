# Phase 1: Display Config Contract & Spike - Context

**Gathered:** 2026-06-10  
**Status:** Ready for planning  
**Source:** User-approved defaults (proceed with defaults)

<domain>
## Phase Boundary

Phase 1 establishes the plugin→Python joint-display **command contract** and proves filtered angle readout can render beside the Simbody sim clock. Scope is Python-side spike + schema/docs — **no plugin UI or trigger wiring yet** (Phases 2–3).

Deliverables:
- `opensim_joint_display_config.json` schema documented
- Curated joint catalog module with abbreviated labels
- Python config watcher in `opensim_live_realtime.py`
- Simbody HUD spike (native overlay or documented fallback)
- Latency proof: filter applies within 200 ms (DISP-03)

</domain>

<decisions>
## Implementation Decisions

### Joint catalog
- Curated instrumented-limb list only — not all 80+ Rajagopal coordinates
- Primary flexion DOFs per segment: `pelvis_tilt`, `hip_flexion_r/l`, `knee_angle_r/l`, `ankle_angle_r/l`
- Catalog maps OpenSim coordinate name → display abbrev (e.g. `knee_angle_r` → `knee_r`)

### Display limits & format
- Maximum 6 joints rendered on screen at once; config with >6 joints truncates with warning log
- Values in degrees, 1 decimal place
- Line format: `{abbrev}: {value}°` (e.g. `knee_r: 42.1°`)
- Multiple joints: newline-separated block beside sim time (or window-title fallback)

### Config transport
- File: `{WORK_DIR}/opensim_joint_display_config.json`
- Fields: `joints` (string[]), `trigger_ts` (float epoch seconds), `seq` (monotonic int)
- Atomic write pattern (temp + rename) documented for Phase 3; Phase 1 spike may write manually for testing
- Do **not** modify UDP v2 quaternion packet format

### Trigger semantics (downstream phases)
- On trigger, plugin writes **current checkbox selection** — not per-TTL preset maps (v2)

### Simbody display spike
- Enumerate `SimbodyVisualizer` Python bindings on OpenSim 4.5 (`dir(viz)`)
- Preferred: on-screen text adjacent to `setShowSimTime(True)` clock
- Acceptable fallback: augment `setWindowTitle()` with compact joint readout; document in schema doc

### Claude's Discretion
- Watcher poll interval (50–100 ms target per STACK.md)
- Whether catalog lives in `opensim_joint_catalog.py` or inline module
- Exact Simbody API if multiple overlay methods exist

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Live bridge
- `opensim_live_realtime.py` — IK loop, `viz.setShowSimTime(True)`, `coord_set.get(name).getValue(state)`, `WORK_DIR`
- `docs/opensim-udp-v2.md` — UDP v2 format (must not change)

### Plugin work directory
- `acqboard.ccp` — `kOpenSimWorkDir`, `launchOpenSimLive()` copies script to work dir

### Sensor vs joint distinction
- `opensim_sensor_map.json` — IMU segment names (not display coordinates)
- `.planning/research/ARCHITECTURE.md` — data flow and proposed catalog table

### Requirements
- `.planning/REQUIREMENTS.md` — DISP-03 (200 ms filter apply latency)

</canonical_refs>

<specifics>
## Specific Ideas

Phase 1 spike test: manually write config with 2 joints (`knee_angle_r`, `hip_flexion_r`) while live UDP stream runs; verify only those values appear in HUD and update within 200 ms of file change.

Existing debug logging at `opensim_live_realtime.py` lines ~740–753 reads `hip_flexion_r`, `knee_angle_r`, pelvis coords — reuse `coord_set` access pattern.

</specifics>

<deferred>
## Deferred Ideas

- Plugin UI multi-select (Phase 2)
- Trigger → config write (Phase 3)
- Per-TTL-line preset profiles (v2 TRIG-10/11)
- Full model coordinate picker (v2)

</deferred>

---

*Phase: 01-display-config-contract-spike*  
*Context gathered: 2026-06-10 via user-approved defaults*

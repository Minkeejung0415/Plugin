# Phase 1 Research: Display Config Contract & Spike

**Researched:** 2026-06-10  
**Phase:** 1 — Display Config Contract & Spike  
**Status:** RESEARCH COMPLETE

## Summary

Phase 1 de-risks the highest-uncertainty item: **rendering filtered IK joint values beside the Simbody sim clock** without breaking the proven UDP→IK pipeline. The sidecar JSON pattern is straightforward; Simbody text overlay API coverage is the spike target.

## Codebase Findings

### opensim_live_realtime.py (live bridge)

| Item | Location / detail |
|------|-------------------|
| Work directory | `WORK_DIR = r"C:\Users\KIN Student\Open-Sim--Bio-Mech"` (line 50) — same path as `kOpenSimWorkDir` in `acqboard.ccp` |
| Visualizer | `viz = model.updVisualizer().updSimbodyVisualizer()` (617) |
| Sim time | `viz.setShowSimTime(True)` (622) |
| Window title | `viz.setWindowTitle(...)` already used with try/except (618–621) |
| Coordinate reads | `coord_set = model.getCoordinateSet()` (660); `coord_set.get("hip_flexion_r").getValue(state)` in degrees via `np.degrees` (740–741) |
| Render loop | ~20 Hz throttled via `OPENSIM_LIVE_VISUALIZER_RATE` / `target_frame_s` (658–704) |
| JSON sidecar precedent | `_load_sensor_map()` reads `opensim_sensor_map.json` from `WORK_DIR` with cache (176–190) |

### Plugin trigger path (Phase 3 — reference only)

- `devicethread.cpp:330` — `handleBroadcastMessage` parses `ACQBOARD TRIGGER <line> <ms>`
- Calls `acquisitionBoard->triggerDigitalOutput(ttlLine, eventDurationMs)` — no OpenSim config write today
- Phase 1 does not modify C++; manual JSON writes suffice for spike

### Rajagopal primary flexion coordinates (curated catalog)

Instrumented-limb primary flexion DOFs aligned with locked defaults:

| Coordinate | Abbrev | Segment driver |
|------------|--------|----------------|
| `pelvis_tilt` | `pelvis_tilt` | `pelvis_imu` |
| `hip_flexion_r` | `hip_r` | `femur_r_imu` |
| `knee_angle_r` | `knee_r` | `tibia_r_imu` |
| `ankle_angle_r` | `ankle_r` | `calcn_r_imu` |
| `hip_flexion_l` | `hip_l` | `femur_l_imu` |
| `knee_angle_l` | `knee_l` | `tibia_l_imu` |
| `ankle_angle_l` | `ankle_l` | `calcn_l_imu` |

Exclude adduction/rotation/beta coupled coords for v1.

## Simbody Display API (Spike Required)

**Known working (OpenSim 4.5 Python):**
- `setShowSimTime(True)` — sim clock overlay
- `setWindowTitle(str)` — used in repo

**Unknown / spike at execution:**
- Custom on-screen text beside clock — enumerate `dir(viz)` and OpenSim docs
- Candidate methods to probe: any `set*Text`, `add*Label`, `setStatusLine` variants

**Fallback (acceptable per PROJECT.md):**
- Append compact HUD to window title: `"Connect OpenSim | knee_r: 42.1° hip_r: 15.3°"`
- Rate-limit title updates to visualizer loop (≤20 Hz) to avoid OS churn

## Config Schema Recommendation

```json
{
  "joints": ["knee_angle_r", "hip_flexion_r"],
  "trigger_ts": 1718035200.123,
  "seq": 1
}
```

| Field | Type | Rules |
|-------|------|-------|
| `joints` | string[] | OpenSim coordinate names from curated catalog; max 6 enforced at read time |
| `trigger_ts` | number | Unix epoch seconds (float OK); informational for logging |
| `seq` | integer | Monotonic; reader ignores stale `seq` |

Write pattern for Phase 3: `opensim_joint_display_config.json.tmp` → atomic rename.

## Watcher Thread Design

- Daemon thread polling `WORK_DIR/opensim_joint_display_config.json` mtime every 50–100 ms
- On change: parse JSON, validate `seq` > last seen, clamp joints to 6, store in thread-safe shared state
- Main IK loop reads shared filter list each frame; no file I/O in hot path
- Invalid coordinate names: skip with `[WARN]` log, do not crash loop

## Latency Budget (DISP-03)

Target: config change → HUD reflects new filter within **200 ms**.

- Poll interval 50 ms → worst-case detection ~50 ms
- Main loop at 20 Hz → worst-case render ~50 ms
- Total ~100 ms typical; 200 ms budget achievable without file watcher events

## Validation Architecture

| Dimension | Approach |
|-----------|----------|
| Schema | Unit test or script validates example JSON against documented fields |
| Watcher | Script writes config with incremented `seq`; assert log line within 200 ms |
| HUD | Manual or scripted: 2-joint filter shows only selected abbrev labels |
| Regression | UDP v2 packet handling unchanged — grep/diff `opensim_live_realtime.py` UDP path |

## Risks

| Risk | Mitigation |
|------|------------|
| No native Simbody text API | Window title fallback documented in schema doc |
| `coord_set.get` throws on bad name | Validate against catalog; skip unknown |
| File partial read | Phase 3 atomic write; Phase 1 spike uses complete manual writes |

## Dependencies

- OpenSim 4.5 Python 3.8 at `C:\OpenSim 4.5\`
- Existing `opensim_live_realtime.py` live loop
- No new pip/npm dependencies

---

*Phase: 01-display-config-contract-spike*  
*Research complete — ready for planning*

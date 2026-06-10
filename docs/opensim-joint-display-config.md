# OpenSim joint display config (sidecar)

Plugin and operator tools write a JSON sidecar that tells `opensim_live_realtime.py` which IK coordinates to show on the Simbody HUD beside the sim clock.

**Transport:** This file is separate from [UDP v2 quaternion packets](opensim-udp-v2.md). Display filtering does not change UDP packet layout or parsing.

## File location

```
{WORK_DIR}/opensim_joint_display_config.json
```

`WORK_DIR` is the OpenSim work directory shared by:

- `opensim_live_realtime.py` — `WORK_DIR` constant (line 50)
- `acqboard.ccp` — `kOpenSimWorkDir` (plugin copies the live script here on launch)

Default path on this project: `C:\Users\KIN Student\Open-Sim--Bio-Mech\opensim_joint_display_config.json`

## Schema

| Field | Type | Rules |
|-------|------|-------|
| `joints` | string[] | OpenSim coordinate names from the curated catalog in `opensim_joint_catalog.py`; max 6; display order preserved |
| `trigger_ts` | number | Unix epoch seconds (float OK) when the config was applied; informational for logging |
| `seq` | integer | Monotonic sequence number; readers ignore stale `seq` values |

### Example

```json
{
  "joints": ["knee_angle_r", "hip_flexion_r"],
  "trigger_ts": 1718035200.123,
  "seq": 1
}
```

## Locked defaults (v1)

| Setting | Value |
|---------|-------|
| Catalog | Curated instrumented-limb list only (not all Rajagopal coordinates) |
| DOF | Primary flexion per segment |
| Max joints | 6 (extras truncated at read time with warning) |
| Apply semantics | On trigger, plugin writes current checkbox selection (Phase 3) |
| Display format | Abbreviated label, 1 decimal degree: `knee_r: 42.1°` |
| Multiple joints | Newline-separated block beside sim time, or window-title fallback |

## Sensor vs joint names

IMU segment names in `opensim_sensor_map.json` (e.g. `femur_r_imu`) identify which body frames receive UDP orientations. Joint display uses **IK coordinate names** (e.g. `hip_flexion_r`, `knee_angle_r`) from `model.getCoordinateSet()` after inverse kinematics.

## Atomic write pattern (Phase 3)

The C++ plugin should write atomically to avoid partial reads:

1. Write `opensim_joint_display_config.json.tmp` with complete JSON
2. Rename/replace `opensim_joint_display_config.json`

Phase 1 spike testing may write the final file directly.

## Reader behavior (`opensim_live_realtime.py`)

- A daemon thread polls file mtime every 50 ms
- On change: parse JSON, validate `joints` against the catalog, apply only if `seq` increased
- Main IK loop reads the shared filter list each frame (no per-frame file I/O)

## Simbody HUD

At startup, `run_live()` probes `SimbodyVisualizer` Python bindings and logs `[JOINT-DISPLAY-SPIKE]` with text-related method names from `dir(viz)`.

### Methods tested (OpenSim 4.5 spike)

| Method | Result |
|--------|--------|
| `setShowSimTime` | Used for sim clock (existing) |
| `setWindowTitle` | **Works** — primary fallback for joint readout |
| `setStatusLine` | Probed at startup; use if call succeeds without error |
| `setStatusText` | Probed at startup; use if call succeeds without error |
| `setCaption` | Probed at startup; use if call succeeds without error |

### Chosen strategy

- **Preferred:** `overlay` when a status/text method accepts a static probe string at startup
- **Fallback:** `window_title` — appends compact joint text: `Connect OpenSim - RedPitaya 8-IMU | knee_r: 42.1° | hip_r: 15.3°`
- Title updates are throttled to when the HUD string changes (≤20 Hz visualizer rate)

### Fallback behavior

If no overlay API works on the target install, only `setWindowTitle` is used. Empty filter restores the base window title with no joint suffix.

## Phase 1 Verification Checklist

Manual verification with OpenSim 4.5 Python 3.8 and a live or test UDP stream:

1. Copy `opensim_live_realtime.py` and `opensim_joint_catalog.py` to `WORK_DIR` (or run from repo if paths match).
2. Start OpenSim Live; confirm IK skeleton updates from UDP.
3. Write to `WORK_DIR/opensim_joint_display_config.json`:
   ```json
   {"joints": ["knee_angle_r", "hip_flexion_r"], "trigger_ts": 0, "seq": 1}
   ```
4. Within **200 ms**, confirm `[JOINT-DISPLAY] seq=1 ...` log and HUD shows only `knee_r` and `hip_r` lines (window title or overlay).
5. Update `seq: 2` with `"joints": ["ankle_angle_r"]` — HUD switches to single `ankle_r` line within **200 ms**.
6. Confirm IK skeleton continues updating (no freeze > 1 s).
7. Confirm `[COORD]` debug logging still works; UDP v2 path unchanged.

Optional automated pre-check: set `OPENSIM_JOINT_DISPLAY_TEST=1` before launch to write seq 1 and 2 configs; watcher should log within 200 ms after `run_live()` starts the watcher thread.

**Operator sign-off required** for steps 3–6 on hardware with Simbody window open.

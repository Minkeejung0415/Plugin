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

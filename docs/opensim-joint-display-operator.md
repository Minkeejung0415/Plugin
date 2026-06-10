# OpenSim Joint Angle Display — Operator Guide

Display **selected IK joint angles** beside the Simbody simulation clock. **Right knee (`knee_r`) is always shown** when no joints are selected; toggles add or remove angles (up to 6 total).

## Quick workflow

1. In the Open Ephys **Acq Board** editor (Red Pitaya), check up to **6 joints** under **Joint HUD** (optional — knee shows by default).
2. Click **OpenSim Live**, then press **Play** in Open Ephys to stream IMU data.
3. Angles update automatically when toggles change, acquisition starts, or a trigger fires (`ACQBOARD TRIGGER <line> <ms>`).
4. OpenSim Live shows the active set (e.g. `knee_r: 42.1°`) beside the sim timer.

## Joint catalog (v1)

| Checkbox label | OpenSim coordinate | Typical IMU segment |
|----------------|-------------------|---------------------|
| pelvis | `pelvis_tilt` | `pelvis_imu` |
| hip_r | `hip_flexion_r` | `femur_r_imu` |
| knee_r | `knee_angle_r` | `tibia_r_imu` |
| ankle_r | `ankle_angle_r` | `calcn_r_imu` |
| hip_l | `hip_flexion_l` | `femur_l_imu` |
| knee_l | `knee_angle_l` | `tibia_l_imu` |
| ankle_l | `ankle_angle_l` | `calcn_l_imu` |

Orange checkbox labels indicate joints on segments that are active in the current sensor stream.

## Config file

Path: `{WORK_DIR}/opensim_joint_display_config.json`

The plugin writes this file automatically (no Apply button). If the file is missing or empty, OpenSim Live falls back to **`knee_angle_r` only**.

Schema and HUD behavior: [opensim-joint-display-config.md](opensim-joint-display-config.md)

UDP quaternion streaming is unchanged — see [opensim-udp-v2.md](opensim-udp-v2.md).

## Trigger integration

Broadcast format (existing):

```
ACQBOARD TRIGGER <ttlLine> <durationMs>
```

On each valid trigger, the plugin writes the **current checkbox selection** (minimum knee when none selected) with an incremented `seq` field (atomic temp + rename).

## Troubleshooting

| Symptom | Check |
|---------|--------|
| No HUD text | OpenSim Live running? Config `seq` increasing? Console shows `[JOINT-DISPLAY]` |
| All coordinates still in console | `[COORD]` debug is separate; HUD uses filtered list only |
| Write failed status | Work dir exists and is writable; path matches `kOpenSimWorkDir` in plugin |
| Stale angles after trigger | Watcher applies only when `seq` increases |

## Manual UAT (hardware)

Requires OpenSim 4.5 Python 3.8, Red Pitaya/ESP32 stream, and Simbody window:

1. Open **OpenSim Live** with no joints checked → HUD shows `knee_r` within 200 ms.
2. Select 2 joints → HUD updates automatically within 200 ms.
3. Change selection → fire trigger → HUD updates to new set.
4. Confirm skeleton keeps moving (no freeze > 1 s).
5. Clear all checkboxes → HUD still shows `knee_r` (default minimum).

See Phase 1 checklist in [opensim-joint-display-config.md](opensim-joint-display-config.md#phase-1-verification-checklist).

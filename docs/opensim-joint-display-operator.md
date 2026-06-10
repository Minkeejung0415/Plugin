# OpenSim Joint Angle Display — Operator Guide

Display **selected IK joint angles** beside the Simbody simulation clock when a trigger fires or when you click **Apply Display** in the Red Pitaya device editor.

## Quick workflow

1. In the Open Ephys **Acq Board** editor (Red Pitaya), check up to **6 joints** under **Joint HUD**.
2. Click **OpenSim Live**, then press **Play** in Open Ephys to stream IMU data.
3. Fire a trigger (`ACQBOARD TRIGGER <line> <ms>`) or click **Apply Display**.
4. OpenSim Live shows only the selected angles (e.g. `knee_r: 42.1°`) beside the sim timer.

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

Default on this project: `C:\Users\KIN Student\Open-Sim--Bio-Mech\opensim_joint_display_config.json`

Schema and HUD behavior: [opensim-joint-display-config.md](opensim-joint-display-config.md)

UDP quaternion streaming is unchanged — see [opensim-udp-v2.md](opensim-udp-v2.md).

## Trigger integration

Broadcast format (existing):

```
ACQBOARD TRIGGER <ttlLine> <durationMs>
```

On each valid trigger, the plugin writes the **current checkbox selection** to the config file with an incremented `seq` field (atomic temp + rename).

## Troubleshooting

| Symptom | Check |
|---------|--------|
| No HUD text | OpenSim Live running? Config `seq` increasing? Console shows `[JOINT-DISPLAY]` |
| All coordinates still in console | `[COORD]` debug is separate; HUD uses filtered list only |
| Write failed status | Work dir exists and is writable; path matches `kOpenSimWorkDir` in plugin |
| Stale angles after trigger | Watcher applies only when `seq` increases |

## Manual UAT (hardware)

Requires OpenSim 4.5 Python 3.8, Red Pitaya/ESP32 stream, and Simbody window:

1. Select 2 joints → **Apply Display** → HUD updates within 200 ms.
2. Change selection → fire trigger → HUD updates to new set.
3. Confirm skeleton keeps moving (no freeze > 1 s).
4. Clear all checkboxes → **Apply Display** → HUD clears (base window title only).

See Phase 1 checklist in [opensim-joint-display-config.md](opensim-joint-display-config.md#phase-1-verification-checklist).

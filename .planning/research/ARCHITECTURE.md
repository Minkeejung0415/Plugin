# Architecture Research — Joint Angle Display on Trigger

**Project:** Open Ephys Red Pitaya Plugin  
**Researched:** 2026-06-10  
**Revised:** 2026-06-10 (scope correction)  
**Confidence:** HIGH

## Component Diagram

```
┌─────────────────────┐     save/load      ┌──────────────────┐
│  DeviceEditor (UI)  │◄──────────────────►│ settings XML     │
│  - joint multi-sel  │                    └──────────────────┘
└─────────┬───────────┘
          │ selected coordinate names
          ▼
┌─────────────────────┐   ACQBOARD TRIGGER   ┌──────────────────┐
│  DeviceThread       │◄─────────────────────│ External/GUI TTL │
│  handleBroadcast    │                      └──────────────────┘
└─────────┬───────────┘
          │ write opensim_joint_display_config.json
          ▼
┌─────────────────────┐   UDP :5000 quats   ┌──────────────────────────┐
│  AcqBoardRedPitaya │─────────────────────►│ opensim_live_realtime.py │
│  (existing stream)  │                     │  - IK loop               │
└─────────────────────┘                     │  - config watcher thread │
                                            │  - coord_set reads       │
                                            │  - filtered HUD render   │
                                            └──────────────────────────┘
```

## Data Flow — Sensor → IK → Display

1. Red Pitaya sends UDP v2 quaternions (`N` sensors, mapped via `opensim_sensor_map.json` or `SENSOR_CHAIN_UP`)
2. Python maps packets to `SENSORS` body frames (`torso_imu`, `femur_r_imu`, …)
3. `InverseKinematicsSolver.assemble(state)` solves model coordinates (`hip_flexion_r`, `knee_angle_r`, …)
4. Operator selects which **coordinates** to display in plugin UI (not sensor names directly)
5. On trigger, plugin writes:
   ```json
   {
     "joints": ["knee_angle_r", "hip_flexion_r"],
     "trigger_ts": 1234567890.1,
     "seq": 42
   }
   ```
   to `WORK_DIR/opensim_joint_display_config.json` (atomic rename)
6. Python watcher detects mtime/seq change; main loop reads only listed coordinates from `coord_set` and renders beside sim time

## Joint Catalog (Proposed v1)

Curated from Rajagopal OpenSense model — primary DOFs near instrumented segments:

| Coordinate | Joint / segment | Typical IMU driver |
|------------|-----------------|-------------------|
| `pelvis_tilt` | Pelvis | `pelvis_imu` |
| `pelvis_list` | Pelvis | `pelvis_imu` |
| `pelvis_rotation` | Pelvis | `pelvis_imu` |
| `hip_flexion_r` | Right hip | `femur_r_imu` |
| `knee_angle_r` | Right knee | `tibia_r_imu` |
| `ankle_angle_r` | Right ankle | `calcn_r_imu` |
| `hip_flexion_l` | Left hip | `femur_l_imu` |
| `knee_angle_l` | Left knee | `tibia_l_imu` |
| `ankle_angle_l` | Left ankle | `calcn_l_imu` |

Additional coordinates (adduction, rotation) may be added per user open-question on DOF display.

## Build Order

1. Config schema + Python reader + display spike (can test standalone)
2. Plugin UI + XML persistence
3. Trigger → write config
4. Filtered HUD render beside sim clock
5. Integration test with live UDP

## Integration Points (Existing Files)

| File | Change type |
|------|-------------|
| `device editor.cpp` / `.h` | Joint multi-select UI, save/load XML |
| `devicethread.cpp` | On TRIGGER, call display config writer |
| `AcqBoardRedPitaya.h/.cpp` or helper | `writeOpenSimJointDisplayConfig()` |
| `opensim_live_realtime.py` | Watcher thread, filtered coordinate HUD |
| `docs/opensim-joint-display-config.md` | New — schema and ops |
| `docs/opensim-udp-v2.md` | Cross-reference sensor vs joint distinction |

## Superseded

Prior architecture for `opensim_view_config.json` and Simbody camera presets is void.

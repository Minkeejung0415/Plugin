# OpenSim UDP packet format v2 (quaternion)

Red Pitaya plugin sends orientation to OpenSim Live / Gen Motion on **UDP port 5000** (localhost).

## Layout (little-endian `float32`)

| Field | Count | Description |
|-------|-------|-------------|
| `t` | 1 | Stream time (seconds, same as Open Ephys buffer time) |
| `version` | 1 | Always `2.0` |
| `N` | 1 | Number of sensors from board `SENSORS:` line |
| quaternions | `N × 4` | Per sensor: `qw, qx, qy, qz` (unit quaternion) |

Total floats: `3 + 4×N`.

Quaternions are taken from the **processor stream** after raw channels for each sensor:

- MPU6050: raw 6 ch, then quat at offsets `+6 … +9`
- 9-axis (ICM20948, MPU9250, BNO055): raw 9 ch, then quat at `+9 … +12`

Q15 values are scaled by `1/32767` and normalized. With **FILTER OFF**, quat channels are zero and the 3D view follows that.

## Sensor → OpenSim body mapping

By default, Red Pitaya sensor index maps **distal → proximal** (up the leg/trunk):

| Sensors | RP index 0 | 1 | 2 | 3 |
|---------|------------|---|---|---|
| 1 | tibia | — | — | — |
| 2 | tibia | thigh (femur) | — | — |
| 3 | tibia | thigh | hip (pelvis) | — |
| 4+ | … continues up (`torso`, foot, left leg) | | | |

Uninstrumented segments stay at the **neutral standing pose** during live IK so the whole body does not drift.

Override with `opensim_sensor_map.json` in the OpenSim work directory (`sensor_slots` list).

## Angle feedback v3.1 (plugin display)

`opensim_live_realtime.py` sends the selected joint angle back to the Open Ephys plugin on **UDP port 5001** after each IK solve.

| Field | Description |
|-------|-------------|
| `t` | Stream time (seconds) |
| `version` | `3.1` |
| `joint_index` | Index into the plugin **Display Joint** dropdown |
| `angle_deg` | Selected joint angle (degrees) |

Total: 4 little-endian `float32` values.

## Display joint

Choose which joint to monitor in the plugin **Display Joint** dropdown. The live script draws that joint's angle on the **OpenSim Simbody viewer** (screen text + window title). Selection is written to `opensim_display_joint.json` in the OpenSim work directory.

## Legacy v1 (acc/gyro)

Packets with `(num_floats - 1) % 6 == 0` after a leading timestamp are still accepted by `opensim_live_realtime.py` for older senders.

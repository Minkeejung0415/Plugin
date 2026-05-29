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

## Legacy v1 (acc/gyro)

Packets with `(num_floats - 1) % 6 == 0` after a leading timestamp are still accepted by `opensim_live_realtime.py` for older senders.

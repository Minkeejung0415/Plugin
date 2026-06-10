# VQF → IK Pipeline Map

Trace of one IMU sample from Red Pitaya hardware to the OpenSim joint-angle
HUD, with the bottlenecks and correctness issues found at each stage.

## Stage map

```
[Red Pitaya, C]
  run_stream() loop @ g_stream_hw_hz (1–2000 Hz, GPIO counter paced)
    └─ acquire_sensor_samples_decimated()          RedPitaya_justin.c
         ├─ per-sensor worker threads (n-1 workers + caller thread)
         ├─ per-sensor decimation: fusion only runs every
         │    g_decim_interval[i] ticks (sample/hold otherwise)
         ├─ read_sensor_raw_channels() → raw acc/gyr/mag counts
         └─ fusion_update_sensor()                 sensor_fusion.c
              ├─ raw counts → physical units (per-sensor LSB scales)
              ├─ vqf_update() / vqf_update_mag()   vqf.c (VQF 6D/9D)
              └─ quaternion → Q15 int16, appended to frame channels
    └─ frame → UDP chunks → Open Ephys plugin (devicethread/acqboard)

[Open Ephys plugin, C++]
  quaternion channels forwarded as UDP v2 packet on port 5000:
    <t, version=2.0, N, N×(qw,qx,qy,qz)>  little-endian float32

[Python bridge: opensim_live_realtime.py, Python 3.8 + OpenSim 4.5]
  _udp_ahrs_thread()
    ├─ v2 quat packets: frame-rotate (_Q_OPENSIM_FRAME) → frame queue
    └─ legacy acc/gyr packets: imufusion AHRS per slot → quats → queue
  run_live() render loop @ OPENSIM_LIVE_VISUALIZER_RATE (~20 Hz)
    ├─ merge live quats with neutral pose for missing sensors
    ├─ TimeSeriesTableRotation → OrientationsReference
    ├─ InverseKinematicsSolver.assemble(state)   (rebuilt per frame —
    │    documented workaround for OpenSim 4.5 Python binding)
    ├─ coord_set.get(name).getValue(state) → joint angles
    ├─ HUD: Simbody DecorationGenerator / window title
    └─ angle feedback UDP :5001 (v3.1 packet)
```

## Findings

### F1 — VQF integrates with the wrong time step under decimation (BUG)
`reinit_fusion_for_hz()` initializes every sensor's VQF with
`Ts = 1 / g_stream_hw_hz`, but `sensor_worker()` only calls
`fusion_update_sensor()` once every `g_decim_interval[i]` ticks
(`init_sensor_decimation()`, derived from `cfg_target_hz`). A sensor
decimated 10:1 therefore integrates gyro rotation with a 10× too-small dt —
orientation lags real motion by the decimation factor.

**Fix**: per-sensor effective rate (`hw_hz / decim_interval`) passed into the
fusion config (`FusionSensorConfig.imu_sample_rate_hz`) and used when
initializing each sensor's VQF instance.

### F2 — `CFG <i> SRATE` changes decimation but not VQF Ts (BUG)
Both `CFG SRATE` handlers (streaming + idle command loops) update
`cfg_target_hz` and decimation but never re-register the sensor with the
fusion module, so VQF keeps the old Ts.

**Fix**: re-register the affected sensor with its new effective rate.

### F3 — Mag-source/fusion-update block duplicated 3× in the hot path
`acquire_sensor_samples()` and both branches of `sensor_worker()` carry
identical ~30-line blocks (gyro extraction, BNO055/MPU9250/ICM20948 mag
selection, bias subtraction, timed `fusion_update_sensor`). Divergence risk
on every future change.

**Fix**: single `fusion_update_from_channels()` helper returning elapsed ns.

### F4 — Redundant status-flag write in `fusion_update_sensor()`
`FUSION_STATUS_MAGNETOMETER_USED` is OR-ed in before
`refresh_last_quaternion()`, which overwrites `last_status_flags`, then OR-ed
in again afterwards. The first write is dead.

### F5 — Python legacy path runs AHRS on filler slots
`_udp_ahrs_thread()` updates `imufusion.Ahrs` + `Offset` for all 8 slots per
packet even though slots outside `slot_map` hold neutral filler whose
quaternions are never read. With 1–2 real sensors that is ~4–8× wasted AHRS
work at packet rate.

### F6 — Per-frame/per-packet allocations in the Python hot loops
- `_neutral_quats_opensim_frame()` recomputes 8 quaternion multiplies per
  rendered frame; the product is constant.
- `struct.unpack` format strings (`<3f`, `<{n*4}f`, `<{n*6}f`, `<4f`) are
  re-parsed every packet; `struct.Struct` objects can be cached.
- `_is_tibia_only_mode()` evaluated 3× per frame.
- `_apply_tibia_only_locks()` makes a SWIG `setLocked` call per coordinate
  per frame even when the mode hasn't changed; locks persist in the state,
  so they only need re-applying on a mode transition.

### Left alone (deliberate)
- Per-frame `InverseKinematicsSolver` rebuild: commented workaround — the
  buffered reference doesn't update coordinates on this OpenSim 4.5 build.
- vqf.c algorithm internals: faithful port of upstream VQF; double-precision
  Butterworth internals are required for stability. (`VQF_SINGLE_PRECISION`
  remains available as a build-time option if the Zynq core needs headroom;
  measured avg/max per-call time is already reported by
  `maybe_report_vqf_stats`.)
- UDP v2 packet format: frozen by compatibility constraint.

## Status

| Finding | Severity | Action |
|---------|----------|--------|
| F1 decimation vs VQF Ts | HIGH (correctness) | fixed |
| F2 CFG SRATE stale Ts | MEDIUM (correctness) | fixed |
| F3 triplicated fusion block | MEDIUM (maintainability) | fixed |
| F4 dead flag write | LOW | fixed |
| F5 AHRS on filler slots | MEDIUM (CPU) | fixed |
| F6 hot-loop allocations | LOW–MEDIUM (CPU) | fixed |

F1 is covered by `tests/vqf_decimation_ts_test.c` (standalone, same pattern as
`tests/redpitaya_stream_framing_test.c`): a 10:1-decimated sensor rotating at
90 °/s for 1 s must fuse to ~90° of yaw — under the old module-rate Ts it
fused to 9°.

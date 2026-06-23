# VQF on ESP32

Orientation filter sources copied from the STEP Plugin repo (`vqf.c`, `vqf.h`), matching Red Pitaya `sensor_fusion.c` / `RedPitaya_justin.c`.

- **Algorithm:** [VQF](https://github.com/dlaidig/vqf) (Daniel Laidig, MIT) — C port in Plugin tree.
- **ESP32 build:** `VQF_SINGLE_PRECISION` (float) for RAM/CPU; Plugin desktop/RP build uses double by default.
- **Scaling:** Same ICM20948 defaults as `sensor_fusion.c` (±2 g → 16384 LSB/g, ±250 dps → 131 LSB/(°/s), mag 0.15 µT/LSB when enabled).
- **Mode:** 6-DOF (gyro + accel) until AK09916 magnetometer read is added on-device; then 9-DOF via `vqf_update_mag`.

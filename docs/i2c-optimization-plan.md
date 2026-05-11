# I2C Sensor Read Optimization Plan

Derived from code review of `RedPitaya_justin.c` and `sensor_fusion.c`.
Five tasks. Do not execute until this plan is approved.

---

## Task 1a — Fix `mag_units_per_lsb` for MPU9250

**File:** `sensor_fusion.c`
**Why:** The ×16 pre-scale applied in `RedPitaya_justin.c:356` is intentional (boosts
small Earth-field values in int16 representation). The conversion factor in sensor_fusion
does not account for it, making reported mag magnitude 16× too large and corrupting
VQF heading.

**Change:** In the MPU9250 sensor config block, divide the raw AK8963 sensitivity by 16:

```c
// Before:
mag_units_per_lsb = 0.15f;

// After:
mag_units_per_lsb = 0.15f / 16.0f;  // 0.009375 uT/LSB — AK8963 16-bit mode + x16 pre-scale
```

Verify the ICM20948 and BNO055 paths are unaffected — they do not apply the ×16
pre-scale and must retain their existing conversion factors.

---

## Task 1b — Guard ST2 HOFL; reuse last good mag on overflow

**File:** `RedPitaya_justin.c` (MPU9250 I2C branch, lines 354–361)
**Struct:** `SensorInstance` (line 75–90)

**Why:** `mag_raw[6]` is the AK8963 ST2 register. Bit 3 (`HOFL`) signals magnetic
sensor overflow — data is unreliable when set. Currently the code marks `mag_is_fresh =
true` unconditionally and passes overflowed data to VQF. Zero-filling channels is
undesirable because VQF would see a spurious field collapse; reusing the last valid
reading is the least-disruptive fallback.

**Step 1 — Add mag cache and per-sensor skip counter to `SensorInstance`:**

```c
typedef struct {
    char name[16];
    void *axi_map;
    uint8_t i2c_addr;
    int num_channels;
    uint8_t data_reg_start;
    bool active;
    bool split_read;
    bool is_spi;
    int16_t gyro_bias[3];
    bool mag_is_fresh;
    int16_t mag_cache[3];       // last known-good mag (pre-scaled); 0,0,0 until first valid read
    int mag_skip_counter;       // for per-sensor mag decimation (Task 3c)
    int cfg_acc_id;
    int cfg_gyr_id;
    int cfg_target_hz;
} SensorInstance;
```

`mag_cache` is zero-initialized at startup (C struct init), so VQF will see zeros only
until the first valid mag read — acceptable since VQF runs in 6D mode until mag is
ready.

**Step 2 — Replace the mag parsing block (lines 354–361):**

```c
// Before:
for (int j = 0; j < 3; j++) {
    int16_t val = (int16_t)(mag_raw[j * 2] | (mag_raw[j * 2 + 1] << 8));
    int32_t amplified = (int32_t)val * 16;
    if (amplified > 32767) amplified = 32767;
    if (amplified < -32768) amplified = -32768;
    channel_out[j + 6] = (int16_t)amplified;
}
s->mag_is_fresh = true;

// After:
if (mag_raw[6] & 0x08) {
    // HOFL set — magnetic sensor overflow, data invalid
    // Reuse last known-good values; do not mark fresh
    channel_out[6] = s->mag_cache[0];
    channel_out[7] = s->mag_cache[1];
    channel_out[8] = s->mag_cache[2];
    // mag_is_fresh remains false (set at top of read_sensor_raw_channels)
} else {
    for (int j = 0; j < 3; j++) {
        int16_t val = (int16_t)(mag_raw[j * 2] | (mag_raw[j * 2 + 1] << 8));
        int32_t amplified = (int32_t)val * 16;
        if (amplified > 32767) amplified = 32767;
        if (amplified < -32768) amplified = -32768;
        channel_out[j + 6] = (int16_t)amplified;
    }
    s->mag_cache[0] = channel_out[6];
    s->mag_cache[1] = channel_out[7];
    s->mag_cache[2] = channel_out[8];
    s->mag_is_fresh = true;
}
```

---

## Task 2 — Consolidate split-read into 14-byte burst

**File:** `RedPitaya_justin.c` (split_read branch, lines 397–403)

**Why:** The MPU-6050/9250 register layout from 0x3B is:

| Offset | Bytes | Content          |
|--------|-------|------------------|
| 0      | 6     | Accel X/Y/Z      |
| 6      | 2     | Temp (skip)      |
| 8      | 6     | Gyro X/Y/Z       |

The two 6-byte transactions read accel (0x3B) and gyro (0x43) separately. Registers
0x41–0x42 (temperature) sit between them, which is why a 12-byte burst from 0x3B would
interleave temp bytes into gyro channels. The correct burst is 14 bytes from 0x3B,
discarding bytes 6–7 before parsing gyro.

A single burst also guarantees that accel and gyro come from the same internal ODR
snapshot — two transactions do not.

**Do not retire `split_read`.** The flag encodes a layout difference (temp bytes present)
that a generic `else` branch does not handle. Remove it only if a layout-aware parser
with an explicit `accel_offset`/`gyro_offset` per sensor type replaces the current
dispatch logic.

**Change (lines 397–403):**

```c
// Before:
} else if (s->split_read) {
    uint8_t raw[12];
    axi_iic_read_n_bytes(s->axi_map, s->i2c_addr, 0x3B, raw, 6);
    axi_iic_read_n_bytes(s->axi_map, s->i2c_addr, 0x43, raw + 6, 6);
    for (int j = 0; j < s->num_channels; j++) {
        channel_out[j] = (int16_t)((raw[j * 2] << 8) | raw[j * 2 + 1]);
    }

// After:
} else if (s->split_read) {
    uint8_t raw[14];
    axi_iic_read_n_bytes(s->axi_map, s->i2c_addr, 0x3B, raw, 14);
    // bytes 0–5: accel X/Y/Z (big-endian int16 pairs)
    for (int j = 0; j < 3; j++) {
        channel_out[j] = (int16_t)((raw[j * 2] << 8) | raw[j * 2 + 1]);
    }
    // bytes 6–7: temp — skip
    // bytes 8–13: gyro X/Y/Z
    for (int j = 0; j < 3; j++) {
        channel_out[j + 3] = (int16_t)((raw[8 + j * 2] << 8) | raw[8 + j * 2 + 1]);
    }
```

Verify `s->num_channels` is 6 for MPU6050 instances (accel + gyro, no mag) before
applying.

---

## Task 3a — Determine actual AXI IIC bus clock

**File:** no code change yet — diagnostic first

**Why:** At 100 kHz I2C, the MPU9250 I2C path takes ~2.29 ms (2 transactions, 19 bytes
total) and the split-read path takes ~1.66 ms — both exceed the 1 ms sample budget.
At 400 kHz both are within budget. The actual clock is set in the AXI IIC IP
configuration and is not visible in C source.

**Action:** Run the system at 1 kHz with MPU9250 in I2C mode and observe
`warn_if_sample_loop_slow_us` output (threshold: 900 µs, line 738). If warnings appear
on every sample, the bus is at 100 kHz.

To confirm and/or change the clock, check:
1. The device tree overlay or `xparameters.h` for the `AXI_IIC` IP instance —
   look for `SCL_RATIO` or equivalent clock divider parameter.
2. If the IP is configured at build time (bitstream), changing it requires a new
   bitstream. If it is runtime-configurable, the prescaler register can be written
   before the first transaction.

No code changes should be made until the actual clock is confirmed.

---

## Task 3b — Verify bypass mode and AK8963 response

**File:** `RedPitaya_justin.c` (MPU9250 I2C init, lines 1183–1193)

**Why:** Bypass mode writes already exist (lines 1187–1188: disable I2C master, set
BYPASS_EN). The gap is that there is no confirmation that the AK8963 is actually
reachable after those writes. A failed bypass enable would cause the mag read at line
348 to NACK or time out silently, producing garbage data with no diagnostic output.

**Add a WHO_AM_I probe immediately after the bypass enable:**

```c
// After existing bypass enable (line 1188 + usleep):
axi_iic_write_byte(map, addr, 0x6A, 0x00);
axi_iic_write_byte(map, addr, 0x37, 0x02);
usleep(5000);

// Verify AK8963 is reachable before configuring it
uint8_t ak_wia = 0;
axi_iic_read_n_bytes(map, 0x0C, 0x00, &ak_wia, 1);  // WIA register, expected 0x48
if (ak_wia != 0x48) {
    printf("  WARNING: AK8963 WIA = 0x%02X (expected 0x48) — bypass mode may have failed\n", ak_wia);
} else {
    printf("  -> AK8963 verified at 0x0C (WIA = 0x48)\n");
}

axi_iic_write_byte(map, 0x0C, 0x0A, 0x16);  // existing line 1192
```

This is a startup diagnostic, not a hard error — leave the init sequence running so
the sensor can be used even if the probe fails, but the warning makes the failure
visible.

---

## Task 3c — Per-sensor mag decimation with cache

**File:** `RedPitaya_justin.c` (MPU9250 I2C branch, lines 344–362)
**Struct:** `SensorInstance` (uses `mag_skip_counter` added in Task 1b)

**Why:** The AK8963 ODR is 100 Hz maximum (configured as 0x16 = continuous mode 2 at
100 Hz). Reading it every sample at 1 kHz wastes ~230 µs of I2C budget 90% of the
time for data that has not changed. Each sensor needs its own counter because multiple
sensors may be active at different rates.

**Logic:** Read mag only when the counter reaches zero; on other samples, copy cached
values and leave `mag_is_fresh = false` so VQF uses the previous mag without treating
it as a new measurement.

The decimation period is `hw_hz / 100` — at 1 kHz this is 10 (read every 10th sample);
at 100 Hz this is 1 (read every sample). Compute it from `current_stream_hw_hz()` at
the call site.

**Change to MPU9250 I2C branch:**

```c
} else {
    // MPU9250 I2C: accel + gyro burst, decimated mag read
    uint8_t raw[12];
    axi_iic_read_n_bytes(s->axi_map, s->i2c_addr, 0x3B, raw, 12);
    for (int j = 0; j < 6; j++) {
        channel_out[j] = (int16_t)((raw[j * 2] << 8) | raw[j * 2 + 1]);
    }

    int mag_period = current_stream_hw_hz() / 100;
    if (mag_period < 1) mag_period = 1;

    if (++s->mag_skip_counter >= mag_period) {
        s->mag_skip_counter = 0;

        uint8_t mag_raw[7];
        axi_iic_read_n_bytes(s->axi_map, 0x0C, 0x03, mag_raw, 7);

        if (mag_raw[6] & 0x08) {
            // HOFL overflow — reuse cache, do not mark fresh
            channel_out[6] = s->mag_cache[0];
            channel_out[7] = s->mag_cache[1];
            channel_out[8] = s->mag_cache[2];
        } else {
            for (int j = 0; j < 3; j++) {
                int16_t val = (int16_t)(mag_raw[j * 2] | (mag_raw[j * 2 + 1] << 8));
                int32_t amplified = (int32_t)val * 16;
                if (amplified > 32767) amplified = 32767;
                if (amplified < -32768) amplified = -32768;
                channel_out[j + 6] = (int16_t)amplified;
            }
            s->mag_cache[0] = channel_out[6];
            s->mag_cache[1] = channel_out[7];
            s->mag_cache[2] = channel_out[8];
            s->mag_is_fresh = true;
        }
    } else {
        // Non-mag sample — copy cache, leave mag_is_fresh = false
        channel_out[6] = s->mag_cache[0];
        channel_out[7] = s->mag_cache[1];
        channel_out[8] = s->mag_cache[2];
    }
}
```

Note: when Task 3c is implemented, Task 1b's standalone HOFL guard becomes redundant for
the MPU9250 path because the overflow check is inlined here. Task 1b may still be useful
as a standalone guard if other mag-reading paths are added later.

---

## Execution order

| # | Task | Effort | Dependency |
|---|------|--------|------------|
| 1 | 1a — fix `mag_units_per_lsb` | ~5 min | none |
| 2 | 1b — HOFL guard + mag_cache field | ~20 min | add struct fields first |
| 3 | 2 — 14-byte burst split-read | ~10 min | none |
| 4 | 3b — AK8963 WIA probe in init | ~10 min | none |
| 5 | 3a — measure bus clock | diagnostic | run after 3b to see clean output |
| 6 | 3c — per-sensor mag decimation | ~30 min | after 1b (needs mag_cache + mag_skip_counter) |

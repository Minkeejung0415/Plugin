
# If red pitaya does not boot

1. check if you can access the [rp-f0f85a.local](http://rp-f0f85a.local/) page
2. check if you can ssh into the red pitaya
3. check if blue and green lights are on and red light is blinking
4. If the led lights are doing fine but you can't ssh into the red pitaya, check if the red pitaya is correctly wired to the router which is connected to the PC
5. if those are fine but you can't access the red pitaya, go to terminal and use "arp -a" command and check if you can see f0f85a or f0cd35 as one of the addresses
6. If you can find them but can't access both of them check if the router is connected to the power on the wall
7. If that is also good, just unplug everything and let is rest for a bit

---

## Final summary (Red Pitaya + Open Ephys plugin)

This document describes the **current** behavior after the acquisition, streaming, UI, and stability work. Rebuild and deploy **both** the Open Ephys plugin and the Red Pitaya server binary when you change either side.

### Red Pitaya server (`RedPitaya_justin.c`)

| Topic | Behavior |
|--------|----------|
| **Recordings** | Streamed sessions still write **`.bin`** and **`.csv`** under `/root/Measurements/` as before. |
| **TCP RX after stream** | After `run_stream()` returns, **`drain_client_rx()`** clears leftover bytes in the client socket receive buffer so the next command loop `read()` does not see tail data from binary packets. |
| **Frame sequence** | First binary frame of each session uses sequence **`0`** in the 22-byte header (`ns` starts at `-1`, increments once per frame before acquire/send). |
| **Hardware tick** | Global stream interval uses **`g_stream_hw_hz`** (default `DESIRED_SAMPLE_RATE_HZ`). **`FREQ:n\n`** (1–2000) updates it (idle + during stream). |
| **Per-sensor rate** | **`CFG i SRATE h\n`** sets desired effective Hz for sensor index `i` (capped to HW rate). Writes the chip's internal ODR register immediately AND updates the decimation counter in `run_stream`. |
| **Sensor snapshot** | Immediately after **`STARTED …`**, server sends **`SENSORS:0,Name;1,Name2\n`** (active sensors at stream start). |
| **Accel/gyro presets** | **`CFG i ACC p\n`** / **`CFG i GYR p\n`** immediately write the sensor chip's full-scale range register over I2C/SPI. Takes effect on the very next sample. |
| **ADC channels** | Red Pitaya IN1/IN2 are enabled (`ANALOG_WAVEFORM_CHANNELS = 2`). If the oscilloscope IP is absent from the FPGA bitstream, SIGBUS is caught and both channels silently output zero. |

### Open Ephys plugin (Red Pitaya board)

| Topic | Behavior |
|--------|----------|
| **Start handshake** | `startAcquisition()` requires a **`STARTED`** or **`STARTED …`** line from the board before `startThread()`. On failure, the command socket is closed and cleared. |
| **Stop / restart** | **`stopAcquisition()`** does **not** send `STOP\n` on the same socket while `run()` reads binary data (that corrupted framing). Order: signal thread → **close socket** → `stopThread()` → delete socket. |
| **Fresh TCP each run** | Each **`startAcquisition()`** opens a **new** TCP connection before sending `START\n`. |
| **Variable frame size** | **`run()`** reads **`bytes_per_frame`** from byte **offset 4** of each 22-byte header and reads exactly that many payload bytes, with byte sliding resync if the magic bytes (`dtype` at offset 8–9) are wrong. Maps up to the configured channel count; pads with zeros if the frame is short. |
| **MSVC** | Nested lambdas in `run()` use explicit capture / naming (`socketReadFully`, `parseHeaderBytesPerFrame` with `[=]`) so Visual Studio builds cleanly. |
| **Sensor list** | After **`STARTED`**, reads **`SENSORS:`** line into **`streamSensorNames`**; cleared on **`stopAcquisition()`**. |
| **CFG send** | **`sendSensorCfgAcc/Gyr`** send `CFG …\n` and also store the preset locally in `sensorAccPreset[]` / `sensorGyrPreset[]` so `run()` can apply the correct scale factor immediately. |
| **Physical unit scaling** | `run()` rebuilds a `channelScale[64]` array each buffer pass and multiplies every raw int16 by the appropriate factor before handing samples to Open Ephys. ACC channels → g, GYR channels → °/s. Same physical motion displays at the same amplitude regardless of active range preset. |

### Device editor (Red Pitaya wide layout)

- Editor width **560** for Red Pitaya only.
- **Left column (~x 6):** sample rate, filter, compact analog in/out, **RECORD** at bottom (y 118).
- **Middle (~x 200):** three **ComboBox** rows — Accel preset, Gyro preset, Sensor Hz (choices derived from HW rate from the sample-rate field).
- **Right (~x 430):** **Sensor** combo filled from **`SENSORS:`** after acquisition starts (freeze **A**).
- **`toFront()`** includes the new combos during animation.

Other board types keep the original **340** width and **x=155** column for filter/analog.

**Note:** Option A full-stack-only layout was reverted earlier due to **RECORD** clipping at fixed panel height.

---

## Changes in the last week of April (detail)

### `RedPitaya_justin.c`

- Save recordings to `.bin` and `.csv`.
- After each streaming session, **drain the TCP receive buffer** on the client socket so leftover binary tail bytes are not read as the next command by the outer `read()` loop.
- **Stream sequence (`ns`):** First binary frame of each session uses sequence **0** in the header; counter increments once per frame before send (CSV row index matches the header).
- **`SO_REUSEPORT`:** On the listen socket when the OS supports it (in addition to `SO_REUSEADDR`).

### Open Ephys plugin (bullet list)

- Fixed channel layout: raw sensor, VQF/filter channels, analog waveform inputs (e.g. per sensor: 9 raw + 4 filter + 2 analog where applicable).
- Filter/VQF toggling; two analog waveform input channels; UI for record, filter, sample rate, analog in gain, analog out voltage.
- **Acquisition start validation:** `STARTED` required before reader thread; socket reset on failure.
- **Stop / restart:** Close-first teardown; new TCP per start; no in-band `STOP` on the binary stream.
- **Header-driven payload** in `run()` for fusion/frame-size changes.
- **Device editor:** Original geometry + `toFront()` during acquisition (see table above).

![[Screenshot 2026-04-27 142836 1.png]]

---

## Changes — 2026-05-04

### `RedPitaya_justin.c`

#### ADC re-enabled
- `ANALOG_WAVEFORM_CHANNELS` changed from `0` to `2` — Red Pitaya IN1 and IN2 are now appended as the last two int16 values in every streamed frame.
- `init_analog_waveform_inputs()` is called at startup. SIGBUS is caught if the oscilloscope IP is absent from the FPGA bitstream; in that case `analog_inputs_ready = false` and both channels output zero silently.

#### Sensor ACC/GYR range preset register writes (`apply_sensor_cfg_acc` / `apply_sensor_cfg_gyr`)

Previously `CFG i ACC p` and `CFG i GYR p` only stored the preset ID — no register was ever written to the sensor chip. Now the full-scale range register is written immediately over I2C or SPI when the command arrives, in both the pre-START idle loop and during a live stream.

Preset mapping (same for ACC and GYR direction):

| Preset | ACC range | GYR range |
|--------|-----------|-----------|
| 0 | ±2 g | ±250 °/s |
| 1 | ±4 g | ±500 °/s |
| 2 | ±8 g | ±1000 °/s |
| 3 | ±16 g | ±2000 °/s |

Per-sensor register details:
- **MPU6050 / MPU9250 I2C:** ACCEL_CONFIG `0x1C`, GYRO_CONFIG `0x1B` — bits `[4:3]` select FS_SEL.
- **MPU9250 / ICM20948 SPI:** same registers via `axi_spi_write`.
- **ICM20948 I2C & SPI:** switches to Bank 2, writes ACCEL_CONFIG `0x14` and GYRO_CONFIG_1 `0x01` (bits `[2:1]`), then restores Bank 0.
- **BNO055:** switches to CONFIG mode, writes ACC_Config `0x08` and GYR_Config_0 `0x0A`, then restores NDOF mode (~60 ms blocking during switch).

`ICM20948_BANK_2 = 0x02` define added alongside existing BANK_0 / BANK_3.

#### Sensor internal ODR register writes (`apply_sensor_odr`)

Previously `CFG i SRATE h` only controlled software decimation — the sensor chip kept running at its power-on default internal rate regardless. Now the chip's own ODR register is written so hardware production rate matches the requested rate.

| Sensor | Register | Formula |
|--------|----------|---------|
| MPU6050 / MPU9250 | CONFIG `0x1A` = `0x01` (enable DLPF → 1 kHz base), SMPLRT_DIV `0x19` | `div = 1000/hz − 1`, clamped 0–255 |
| ICM20948 | Bank 2: GYRO_SMPLRT_DIV `0x00`, ACCEL_SMPLRT_DIV_1/2 `0x10`/`0x11` | gyro: `1100/hz−1` (8-bit); accel: `1125/hz−1` (12-bit) |
| BNO055 | No write — NDOF fusion engine owns the ODR at fixed ~100 Hz | — |

Called at three points:
1. **`identify_and_add_sensor()`** — sets chip to `DESIRED_SAMPLE_RATE_HZ` (100 Hz) at boot instead of leaving it at max default.
2. **`process_stream_commands()`** — fires immediately mid-stream alongside decimation update.
3. **Pre-START command loop** — fires when `CFG i SRATE h` arrives before `START`.

---

### `acqboard.ccp` / `Acqboardredpitaya.h`

#### Per-sensor preset storage
- `sensorAccPreset[6]` and `sensorGyrPreset[6]` added to `AcqBoardRedPitaya`. Default 0 (±2g / ±250°/s).
- `sendSensorCfgAcc` and `sendSensorCfgGyr` now write to these arrays before sending the TCP command so `run()` always has the current preset without a round-trip.

#### Physical unit scaling in `run()`
- Sensitivity lookup tables added at file scope:
  - `kAccSensitivity[4]` = `{ 16384, 8192, 4096, 2048 }` LSB/g
  - `kGyrSensitivity[4]` = `{ 131.072, 65.536, 32.768, 16.384 }` LSB/°/s
- At the top of each buffer pass, `run()` rebuilds a `channelScale[MAX_CHANNELS]` array from `streamSensorNames` and the current presets, then multiplies every raw int16 by its factor before handing the sample to Open Ephys.
- Channel layout handled per sensor type:
  - **MPU family:** `[0-2]` = acc (accScale), `[3-5]` = gyr (gyrScale), `[6-8]` = mag (1.0), `[N..N+3]` = quaternion (1.0)
  - **BNO055:** `[0-2]` = acc, `[3-5]` = mag (1.0), `[6-8]` = gyr (reordered), `[N..N+3]` = quaternion (1.0)
- Result: the same physical motion displays at the same amplitude regardless of which range preset is active. The graph only changes when motion exceeds the narrower range and clips.

---

## VQF / Filter feature

The **`FILTER`** button controls whether VQF/filter values are written into the reserved VQF channels.

**When filter is on:** VQF channels contain calculated values.

**When filter is off:** VQF channels are still present and are filled with zeros.

The Red Pitaya server applies filter commands between frames when possible. **Caveat:** filter commands share the same TCP connection as the binary stream from the plugin's perspective; the plugin and server were hardened for start/stop and framing, but toggling filter very rapidly during streaming can still stress framing—prefer toggling when idle if you see glitches.

---

## Analog output voltage control

The UI has an editable **`Analog Out (V)`** field.

**Valid range:** `0.0` to `1.8` V

**Command sent to the board:**

```
AOUT:<value>
```

The Red Pitaya server receives and logs this command. Analog output is a **command**, not a measured waveform channel; to show the real voltage on-screen you would need to sense it on an input channel.

---

# What more needs to be done

- Answer hardware questions before implementing 10kHz FPGA sync signal (see open questions below).
- Test ADC analog input and output with hardware.
- Fix the Makefile issue where new files using `vqf.c` or `sensor_fusion.c` must be wired in manually instead of being picked up automatically.
- ACC channels display in g (0–2 range) which is very small relative to the default Open Ephys Range=250 display setting — consider scaling to milli-g or advising users to reduce the display range for acc channels.

## Open questions — 10kHz FPGA sync (not yet implemented)

1. What is the signal physically — GPIO pin pulsing at 10kHz, a counter register at a specific AXI address, or something else?
2. What AXI address or `/dev/` path is it accessible from?
3. What should happen on each sync pulse — trigger an ADC read, timestamp a sample, something else?
4. Should IMU sensors and ADC both be read on the sync tick, or just the ADC?
5. Should the sync signal replace the current GPIO counter timing loop or run alongside it?
6. Is 10kHz the new streaming rate, or is it an external event independent of the stream rate?

---

## Tests (no Open Ephys build required)

From the repo root:

```bash
cc -std=c99 -Wall -Wextra -o /tmp/rp_framing_test tests/redpitaya_stream_framing_test.c && /tmp/rp_framing_test
```

This checks variable-payload framing and header resync logic used by the plugin reader.


1. check if you can access the [rp-f0f85a.local](http://rp-f0f85a.local/) page
2. check if you can ssh into the red pitaya
3. check if blue and green lights are on and red light is blinking
4. If the led lights are doing fine but you can't ssh into the red pitaya, check if the red pitaya is correctly wired to the router which is connected to the PC
5. if those are fine but you can't access the red pitaya, go to terminal and use "arp -a" command and check if you can see f0f85a or f0cd35 as one of the addresses
6. If you can find them but can't access both of them check if the router is connected to the power on the wall
7. If that is also good, just unplug everything and let is rest for a bit

---

## Final summary (Red Pitaya + Open Ephys plugin)

This document describes the **current** behavior after the acquisition, streaming, UI, and stability work. Rebuild and deploy **both** the Open Ephys plugin and the Red Pitaya server binary when you change either side.

### Red Pitaya server (`RedPitaya_justin.c`)

| Topic | Behavior |
|--------|----------|
| **Recordings** | Streamed sessions still write **`.bin`** and **`.csv`** under `/root/Measurements/` as before. |
| **TCP RX after stream** | After `run_stream()` returns, **`drain_client_rx()`** clears leftover bytes in the client socket receive buffer so the next command loop `read()` does not see tail data from binary packets. |
| **Frame sequence** | First binary frame of each session uses sequence **`0`** in the 22-byte header (`ns` starts at `-1`, increments once per frame before acquire/send). |
| **Hardware tick** | Global stream interval uses **`g_stream_hw_hz`** (default `DESIRED_SAMPLE_RATE_HZ`). **`FREQ:n\n`** (1–2000) updates it (idle + during stream). |
| **Per-sensor rate** | **`CFG i SRATE h\n`** sets desired effective Hz for sensor index `i` (capped to HW rate). **`run_stream`** **holds** prior sample when decimating (integer `hw/target` ratio). |
| **Sensor snapshot** | Immediately after **`STARTED …`**, server sends **`SENSORS:0,Name;1,Name2\n`** (active sensors at stream start). |
| **Accel/gyro presets** | **`CFG i ACC p\n`** / **`CFG i GYR p\n`** stored per sensor (`cfg_acc_id`, `cfg_gyr_id`); logged (register mapping can be added later). |

### Open Ephys plugin (Red Pitaya board)

| Topic | Behavior |
|--------|----------|
| **Start handshake** | `startAcquisition()` requires a **`STARTED`** or **`STARTED …`** line from the board before `startThread()`. On failure, the command socket is closed and cleared. |
| **Stop / restart** | **`stopAcquisition()`** does **not** send `STOP\n` on the same socket while `run()` reads binary data (that corrupted framing). Order: signal thread → **close socket** → `stopThread()` → delete socket. |
| **Fresh TCP each run** | Each **`startAcquisition()`** opens a **new** TCP connection before sending `START\n`. |
| **Variable frame size** | **`run()`** reads **`bytes_per_frame`** from byte **offset 4** of each 22-byte header and reads exactly that many payload bytes, with byte sliding resync if the magic bytes (`dtype` at offset 8–9) are wrong. Maps up to the configured channel count; pads with zeros if the frame is short. |
| **MSVC** | Nested lambdas in `run()` use explicit capture / naming (`socketReadFully`, `parseHeaderBytesPerFrame` with `[=]`) so Visual Studio builds cleanly. |

| **Sensor list** | After **`STARTED`**, reads **`SENSORS:`** line into **`streamSensorNames`**; cleared on **`stopAcquisition()`**. |
| **CFG send** | **`sendSensorCfgAcc/Gyr/Srate`** send **`CFG …\n`** when the command socket is open. |

### Device editor (Red Pitaya wide layout)

- Editor width **560** for Red Pitaya only.
- **Left column (~x 6):** sample rate, filter, compact analog in/out, **RECORD** at bottom (y 118).
- **Middle (~x 200):** three **ComboBox** rows — Accel preset, Gyro preset, Sensor Hz (choices derived from HW rate from the sample-rate field).
- **Right (~x 430):** **Sensor** combo filled from **`SENSORS:`** after acquisition starts (freeze **A**).
- **`toFront()`** includes the new combos during animation.

Other board types keep the original **340** width and **x=155** column for filter/analog.

**Note:** Option A full-stack-only layout was reverted earlier due to **RECORD** clipping at fixed panel height.

---

## Changes in the last week of April (detail)

### `RedPitaya_justin.c`

- Save recordings to `.bin` and `.csv`.
- After each streaming session, **drain the TCP receive buffer** on the client socket so leftover binary tail bytes are not read as the next command by the outer `read()` loop.
- **Stream sequence (`ns`):** First binary frame of each session uses sequence **0** in the header; counter increments once per frame before send (CSV row index matches the header).
- **`SO_REUSEPORT`:** On the listen socket when the OS supports it (in addition to `SO_REUSEADDR`).

### Open Ephys plugin (bullet list)

- Fixed channel layout: raw sensor, VQF/filter channels, analog waveform inputs (e.g. per sensor: 9 raw + 4 filter + 2 analog where applicable).
- Filter/VQF toggling; two analog waveform input channels; UI for record, filter, sample rate, analog in gain, analog out voltage.
- **Acquisition start validation:** `STARTED` required before reader thread; socket reset on failure.
- **Stop / restart:** Close-first teardown; new TCP per start; no in-band `STOP` on the binary stream.
- **Header-driven payload** in `run()` for fusion/frame-size changes.
- **Device editor:** Original geometry + `toFront()` during acquisition (see table above).

![[Screenshot 2026-04-27 142836 1.png]]

---

## VQF / Filter feature

The **`FILTER`** button controls whether VQF/filter values are written into the reserved VQF channels.

**When filter is on:** VQF channels contain calculated values.

**When filter is off:** VQF channels are still present and are filled with zeros.

The Red Pitaya server applies filter commands between frames when possible. **Caveat:** filter commands share the same TCP connection as the binary stream from the plugin’s perspective; the plugin and server were hardened for start/stop and framing, but toggling filter very rapidly during streaming can still stress framing—prefer toggling when idle if you see glitches.

---

## Analog output voltage control

The UI has an editable **`Analog Out (V)`** field.

**Valid range:** `0.0` to `1.8` V

**Command sent to the board:**

```
AOUT:<value>
```

The Red Pitaya server receives and logs this command. Analog output is a **command**, not a measured waveform channel; to show the real voltage on-screen you would need to sense it on an input channel.

---

# What more needs to be done

- Test analog input and output with ADC hardware.
- Fix the Makefile issue where new files using `vqf.c` or `sensor_fusion.c` must be wired in manually instead of being picked up automatically.

---

## Tests (no Open Ephys build required)

From the repo root:

```bash
cc -std=c99 -Wall -Wextra -o /tmp/rp_framing_test tests/redpitaya_stream_framing_test.c && /tmp/rp_framing_test
```

This checks variable-payload framing and header resync logic used by the plugin reader.

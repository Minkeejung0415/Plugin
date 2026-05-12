
# If red pitaya does not boot

1. check if you can access the [rp-f0f85a.local](http://rp-f0f85a.local/) page
2. check if you can ssh into the red pitaya
3. check if blue and green lights are on and red light is blinking
4. If the led lights are doing fine but you can't ssh into the red pitaya, check if the red pitaya is correctly wired to the router which is connected to the PC
5. if those are fine but you can't access the red pitaya, go to terminal and use "arp -a" command and check if you can see f0f85a or f0cd35 as one of the addresses
6. If you can find them but can't access both of them check if the router is connected to the power on the wall
7. If that is also good, just unplug everything and let is rest for a bit

---

## Current system summary (Red Pitaya + Open Ephys plugin)

Rebuild and deploy **both** the Open Ephys plugin and the Red Pitaya server binary when you change either side.

### Red Pitaya server (`RedPitaya_justin.c`)

| Topic | Behavior |
|--------|----------|
| **Recordings** | Streamed sessions write **`.bin`** and **`.csv`** under `/root/Measurements/`. |
| **TCP RX after stream** | After `run_stream()` returns, `drain_client_rx()` clears leftover bytes in the client socket receive buffer so the next command loop `read()` does not see tail data. |
| **Frame sequence** | First binary frame of each session uses sequence **`0`** in the 22-byte header (`ns` starts at `-1`, increments once per frame before acquire/send). |
| **Hardware tick** | Global stream interval uses `g_stream_hw_hz` (default `DESIRED_SAMPLE_RATE_HZ`). `FREQ:n\n` (1–2000) updates it (idle + during stream). |
| **Per-sensor rate** | `CFG i SRATE h\n` sets desired effective Hz for sensor `i` (capped to HW rate). Writes the chip's internal ODR register immediately AND updates the decimation counter in `run_stream`. |
| **Sensor snapshot** | Immediately after `STARTED …`, server sends `SENSORS:0,Name;1,Name2\n` (active sensors at stream start). |
| **Accel/gyro presets** | `CFG i ACC p\n` / `CFG i GYR p\n` immediately write the sensor chip's full-scale range register over I2C/SPI. Takes effect on the very next sample. |
| **ADC channels** | Red Pitaya IN1/IN2 enabled (`ANALOG_WAVEFORM_CHANNELS = 2`). If the oscilloscope IP is absent from the FPGA bitstream, SIGBUS is caught and both channels silently output zero. |
| **Data transport** | Sample packets sent over **UDP port 55001** (`sendto()` per packet). TCP port 5000 is for control commands only. |

### Open Ephys plugin (Red Pitaya board)

| Topic | Behavior |
|--------|----------|
| **Start handshake** | `startAcquisition()` requires a `STARTED` or `STARTED …` line from the board before `startThread()`. On failure, the command socket is closed and cleared. |
| **Stop / restart** | `stopAcquisition()` closes the TCP command socket; the server detects the close and stops sending UDP data. Order: signal thread → close socket → `stopThread()` → delete socket. |
| **Fresh TCP each run** | Each `startAcquisition()` opens a new TCP connection before sending `START\n`. |
| **Data transport** | Sample data arrives over **UDP port 55001** (`DatagramSocket` bound in `run()`). One packet per sample (`CHUNK_SAMPLES = 1`). Each packet is `HEADER_SIZE (22) + numChannels × 2` bytes. TCP is used only for control commands. |
| **Sensor list** | After `STARTED`, reads `SENSORS:` line into `streamSensorNames`; cleared on `stopAcquisition()`. |
| **CFG send** | `sendSensorCfgAcc/Gyr/Srate` send `CFG …\n` when the command socket is open. Also store the preset locally in `sensorAccPreset[]` / `sensorGyrPreset[]` so `run()` can apply the correct scale factor immediately. |
| **Physical unit scaling** | `run()` rebuilds a `channelScale[64]` array each buffer pass and multiplies every raw int16 by the appropriate factor. ACC channels → g, GYR channels → °/s. Same physical motion displays at the same amplitude regardless of active range preset. |

### Device editor (Red Pitaya wide layout)

- Editor width **560** for Red Pitaya only.
- **Left column (~x 6):** sample rate, filter, compact analog in/out, **RECORD** at bottom (y 118).
- **Middle (~x 200):** three ComboBox rows — Accel preset, Gyro preset, Sensor Hz.
- **Right (~x 430):** Sensor combo filled from `SENSORS:` after acquisition starts.
- `toFront()` includes the new combos during animation.

Other board types keep the original **340** width and **x=155** column for filter/analog.

---

## VQF / Filter feature

The **`FILTER`** button controls whether VQF quaternion values are written into the reserved VQF channels.

**When filter is on:** VQF channels (4 per sensor: qw, qx, qy, qz) contain calculated orientation.

**When filter is off:** VQF channels are present but filled with zeros.

The server applies filter state changes between frames. Prefer toggling when idle if you see glitches from rapid toggling during acquisition.

---

## Analog output voltage control

The UI has an editable **`Analog Out (V)`** field. Valid range: `0.0` to `1.8` V. Command sent: `AOUT:<value>`. Analog output is a command, not a measured waveform channel.

---

## What more needs to be done

- Test ADC analog input and output with hardware.
- Fix the Makefile issue where new files using `vqf.c` or `sensor_fusion.c` must be wired in manually.
- ACC channels display in g (0–2 range) which is small relative to the default Open Ephys Range=250 — consider advising users to reduce the display range.

## Open questions — 10kHz FPGA sync (not yet implemented)

1. What is the signal physically — GPIO pin pulsing at 10kHz, a counter register at a specific AXI address, or something else?
2. What AXI address or `/dev/` path is it accessible from?
3. What should happen on each sync pulse — trigger an ADC read, timestamp a sample, something else?
4. Should IMU sensors and ADC both be read on the sync tick, or just the ADC?
5. Should the sync signal replace the current GPIO counter timing loop or run alongside it?
6. Is 10kHz the new streaming rate, or is it an external event independent of the stream rate?

---

### 2026-05-12

#### Parallel sensor reads with persistent worker thread pool

Three commits in sequence built out full sensor parallelism in `RedPitaya_justin.c`.

**Background — why this was needed:**
The `warn_if_sample_loop_slow_us` threshold (900 µs) was firing regularly with two I2C sensors. Both sensors were read back-to-back on a single thread, totalling ~1100 µs — over the 1 ms budget. The fix runs both sensors simultaneously so the wall-clock cost is the slower of the two, not the sum.

**Commit 1 — `pthread.h` include (17:24)**
Added `#include <pthread.h>` as a standalone preparatory commit.

**Commit 2 — Parallelize `acquire_sensor_samples_decimated` (17:32)**
- Extracted per-sensor work (I2C/SPI read, decimation, VQF update) into `sensor_worker()`.
- `acquire_sensor_samples_decimated()` spawns a `pthread` per sensor, runs the last sensor inline on the calling thread, then `pthread_join`s all workers.
- Per-thread VQF stats (`vqf_total_ns`, `vqf_max_ns`, `vqf_call_count`) are merged after join.
- No mutex needed — each sensor has an independent index into `g_decim_counter`, `g_sensor_hold`, and the `frame_buffer` slice.
- With 2 sensors: wall-clock drops from ~1400 µs to ~700 µs since both bus reads happen simultaneously.

**Commit 3 — Persistent worker thread pool (17:45)**
The previous commit created and destroyed a `pthread` every sample iteration (up to 1000×/sec), costing 50–150 µs of OS overhead that cancelled most of the parallelism gain.

- `sensor_threads_init(ctx, n)` creates `n-1` worker threads once at stream start. Each thread sits in a loop: `sem_wait(&go)` → `sensor_worker()` → `sem_post(&done)`.
- Each sample iteration: post `go` to all workers, run the last sensor inline, then `sem_wait(&done)` from each worker.
- Per-iteration thread overhead drops from ~100 µs (pthread_create/join) to ~2–5 µs (two semaphore ops), preserving the full ~550 µs parallel saving.
- `sensor_threads_shutdown()` sets `running = 0`, posts `go` to unblock each thread, and `pthread_join`s them. Called at every `run_stream` exit point (malloc failure, command stop, normal exit).

**Net result:** Two-sensor loop time reduced from ~1400 µs → ~700 µs. Warnings at 900 µs threshold eliminated at normal operating rates.

---

## Tests (no Open Ephys build required)

```bash
cc -std=c99 -Wall -Wextra -o /tmp/rp_framing_test tests/redpitaya_stream_framing_test.c && /tmp/rp_framing_test
```

Checks variable-payload framing and header resync logic used by the plugin reader.

---

## Change log

### Week of 2026-04-27

**`RedPitaya_justin.c`**
- Save recordings to `.bin` and `.csv` under `/root/Measurements/`.
- Drain TCP receive buffer after `run_stream()` returns (`drain_client_rx()`).
- Stream sequence counter `ns` starts at `-1`, increments before each frame, so the first frame has sequence 0.
- `SO_REUSEPORT` on the listen socket when the OS supports it.

**Open Ephys plugin**
- Fixed channel layout: raw sensor + VQF channels + analog waveform inputs.
- Filter/VQF toggling; two analog waveform input channels; UI for record, filter, sample rate, analog in gain, analog out voltage.
- Acquisition start validation: `STARTED` required before reader thread; socket reset on failure.
- Stop / restart: close-first teardown; new TCP per start.
- Device editor wide layout for Red Pitaya (560 px).

---

### 2026-05-04

**`RedPitaya_justin.c`**

- `ANALOG_WAVEFORM_CHANNELS` changed from `0` to `2` — Red Pitaya IN1 and IN2 appended to every frame. `init_analog_waveform_inputs()` called at startup; SIGBUS caught if oscilloscope IP absent.
- `apply_sensor_cfg_acc` / `apply_sensor_cfg_gyr`: `CFG i ACC/GYR p` now writes the full-scale range register to the sensor chip immediately, in both the idle loop and during a live stream. Preset → register mapping:

| Preset | ACC | GYR |
|--------|-----|-----|
| 0 | ±2 g | ±250 °/s |
| 1 | ±4 g | ±500 °/s |
| 2 | ±8 g | ±1000 °/s |
| 3 | ±16 g | ±2000 °/s |

- `apply_sensor_odr`: `CFG i SRATE h` now writes the chip's ODR register (MPU: SMPLRT_DIV; ICM20948 Bank 2: GYRO/ACCEL_SMPLRT_DIV; BNO055: no-op). Called at sensor init, pre-START, and mid-stream.
- `ICM20948_BANK_2 = 0x02` define added.

**`acqboard.ccp` / `Acqboardredpitaya.h`**
- `sensorAccPreset[6]` and `sensorGyrPreset[6]` added to `AcqBoardRedPitaya`.
- `run()` rebuilds `channelScale[MAX_CHANNELS]` each buffer pass from `streamSensorNames` and current presets (`kAccSensitivity[4]`, `kGyrSensitivity[4]`). Raw int16 × scale → physical units before `addToBuffer()`.

---

### 2026-05-11

#### Data transport switched from TCP to UDP

Sample data now streams over **UDP port 55001**. TCP port 5000 is control-only.

**`RedPitaya_justin.c`:**
- `UDP_PORT 55001` and `CHUNK_SAMPLES 1` constants added.
- `main()` creates a `SOCK_DGRAM` socket bound to `INADDR_ANY:55001`.
- On `START`, server calls `getpeername()` on the TCP client socket to get the client IP and stores it as `client_udp_addr` for `sendto()` destinations.
- `run_stream()` signature: `int udp_fd, struct sockaddr_in *client_udp_addr` added.
- Each assembled packet is sent via `sendto()`. The old `send(client_fd, packet, …)` TCP data path is removed.
- Partial chunk flushed on exit (`chunk_idx > 0`).
- `chunk_buffer` malloc/free added alongside `packet`.

**`acqboard.ccp`:**
- Removed: `socketReadFully`, `parseHeaderBytesPerFrame`, `readOneFrame` TCP lambdas; the inner TCP sample-collection loop.
- Added: `DatagramSocket udpSocket` bound to port 55001; `waitUntilReady(true, 100)` + `read()` loop replacing the old TCP reads.
- `packetsPerChunk = 1` (matches `CHUNK_SAMPLES`).

`CHUNK_SAMPLES = 1` is critical: using 100 caused 100 ms of buffering before each UDP datagram, producing a staccato burst pattern (>1 ms inter-burst gaps visible in Open Ephys). Per-sample delivery (= 1) restores the original smooth cadence.

#### ICM20948 magnetometer fixes

**Fix 1 — Mag cache on SPI path (`read_sensor_raw_channels`, line ~400):**

The AK09916 magnetometer inside the ICM20948 runs at ~100 Hz. At 1 kHz IMU rate, ~90% of frames have status bit 0 = 0 (no new data). Previously, `channel_out[6/7/8]` was written to literal zero on those frames. Now, fresh values are written to `s->mag_cache[]` and non-fresh frames replay the cache — matching the existing MPU9250 I2C behavior.

**Fix 2 — ICM20948 I2C fusion mag path (reverted):**

An earlier change removed `&& s->is_spi` from the three fusion mag-assignment blocks so ICM20948 on I2C would also feed magnetometer data into VQF. This was reverted: the current FPGA image does not collect magnetometer data when ICM20948 is connected over I2C, so passing those values to VQF would corrupt the heading estimate. The `&& s->is_spi` guard is restored in all three blocks. ICM20948 on I2C runs VQF in 6D mode (accelerometer + gyroscope only).

#### Performance note — I2C latency at 1 kHz

`warn_if_sample_loop_slow_us` (threshold: 900 µs) fires frequently with two I2C sensors:

| Step | Cost |
|------|------|
| One 12-byte I2C read at 400 kHz | ~550 µs |
| Two I2C sensors (no mag frames) | ~1100 µs |
| VQF 6D update per sensor | ~150 µs |
| VQF 9D update per sensor | ~400 µs |

Root causes: AXI IIC soft-IP blocks the CPU for the full transaction duration; VQF motion-bias estimator runs a 3×3 matrix inverse (`vqf_matrix3_inv`) every sample. Possible mitigations (not yet implemented): move to SPI sensors, disable `motionBiasEstEnabled` in VQF params, or read sensors on separate threads if they are on independent I2C buses.

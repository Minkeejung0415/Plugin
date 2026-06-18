# Sample-Rate Stress Test (ESP32 STEP Node)

Find the highest sustainable `FREQ:<Hz>` before acquisition degrades. Firmware accepts any Hz >= 1; USB 115200, filter CPU time, SD write latency, and loop time set practical limits.

## Ground Truth

Host CSV/binary captures are useful transport checks, but SD continuity is the ground truth once `RECORD ON` is enabled.

For a no-loss claim, the SD file must show:

- `Lost samples (seq jumps > 1): 0`
- `Duplicate sequence values (consecutive): 0`
- timestamp gaps are zero or explicitly explained
- firmware counters report zero SD errors and zero loop overruns

## Recommended Sweep

1. Sweep `FILTER OFF` and `FILTER ON` to isolate VQF CPU cost.
2. Sweep `SD OFF` and `SD ON` to isolate SD write latency.
3. Start at `100 Hz`, `200 Hz`, `500 Hz`, then `1000 Hz`, or binary search once failure appears.
4. Record at least 10 seconds per step.
5. Increase until duplicates, gaps, SD errors, or loop overruns appear, then back off about 20%.

## Automated Sweep

```powershell
python host\stress_test_serial.py --port COM5 --capture-s 10 --hz 100 200 300 400 --filter both --sd both
python host\stress_test_serial.py --port COM5 --capture-s 20 --hz 100 150 200 --filter on --sd on
python host\stress_test_serial.py --port COM5 --capture-s 60 --hz 500 700 900 --filter on --sd on --stream off
```

Pass per rate:

- no host-visible sequence gaps or duplicates
- enough host-visible rows
- `mean_hz` within 95%-115% of requested Hz
- when SD is enabled, firmware reports saved samples, zero SD errors, and zero loop overruns

Use `--stream off` with `--sd on` to isolate device acquisition + filter + SD from
USB serial transport. In stream-off mode the harness does not expect host rows; it records
for the capture window and computes `mean_hz` from `SD_FINAL saved=`. This is a diagnostic
mode only; the final Open Ephys path still needs a stream-on or TCP/Open Ephys test.

Recommended cap = 80% of the highest passing rate.

Outputs:

- `host/stress_results/rate_<hz>__filter_<on|off>__sd_<on|off>.csv`
- `host/stress_results/SUMMARY.md`
- `host/stress_results/SUMMARY.json`

## No-SD SPI Characterization

2026-06-16 hardware result, with ICM20948 on the SD-card SPI bus and CS wired to XIAO D7 / GPIO44:

- boot confirmed `WHO_AM_I 0xEA` and AK09916 through the ICM SPI auxiliary bus
- SD card was intentionally absent, so every run used `--sd off`
- filter off passed through 1400 Hz before ODR tuning; 1500 Hz produced duplicate payloads
- filter on passed through 1000 Hz; 1050 Hz failed because the loop could not sustain the requested rate
- after explicit ICM ODR setup (`gyro_div=0`, `accel_div=0`, `odr_align=1`, `dlpf=off`), filter off still passed through 1400 Hz and failed at 1500 Hz, but 1500 Hz duplicate count dropped to 3 frames over 20 seconds
- after explicit ICM ODR setup, filter on still passed through 1000 Hz and failed at 1050 Hz due rate shortfall

This is a sensor/firmware/USB characterization only. It does not prove no-loss acquisition,
because the SD ground-truth path was not active.

No-SD raw boundary sweep:

```powershell
python esp32\host\stress_test_serial.py --port COM4 --hz 1300 1400 1500 1600 1800 2000 --capture-s 20 --filter off --sd off --wait-s 5
```

No-SD VQF boundary sweep:

```powershell
python esp32\host\stress_test_serial.py --port COM4 --hz 800 900 950 1000 1050 --capture-s 20 --filter on --sd off --wait-s 5
```

## SD Ground-Truth Verification

After a run with `--sd on`, stop recording, copy `step_session.bin` off the SD card, then analyze it:

```powershell
python host\analyze_sample_rate.py path\to\step_session.bin --format sd-bin
python host\stress_test_serial.py --sd-file path\to\step_session.bin
```

The SD analyzer reads the STEP binary header and checks `seq`, `timestamp_us`, and record alignment.

## SD-On SPI Diagnosis

2026-06-16 regression signature after moving ICM20948 to SPI:

- `500 Hz`, filter on, SD on failed with `sd_err=239`, `max_sd_us=107610`, and `overrun=29`
- `930-970 Hz`, filter on, SD on failed with nonzero `sd_err`, high `max_sd_us`, and loop overruns
- `dup_seq=0` stayed clean, so this was not a duplicate sensor-read failure

The firmware now splits SD failures into explicit fields:

- `queue_drops=`: acquisition loop could not enqueue because the SD queue was full
- `write_errors=`: SD file write returned fewer bytes than requested
- `header_errors=`, `open_errors=`, `begin_errors=`: session setup failures
- `mutex_timeouts=`: SD writer could not take its SD/file mutex
- `max_queue_depth=`: deepest observed SD queue backlog
- `flush_count=`, `max_flush_us=`: flush cadence and worst flush latency
- `spi_mutex_timeouts=`: ICM SPI lock timeout count. Historical SD-on runs also used
  this as the ICM/SD shared SPI lock timeout count before the ICM moved to HSPI.

Interpret SD-on failures from these split counters before increasing queue depth. A 1024-record queue
should absorb ordinary 100 ms stalls at 500 Hz; if errors remain, suspect small per-sample writes,
card/filesystem behavior, or SPI contention on older shared-bus wiring. The Arduino firmware batches
queued SD records into larger writes. Current wiring keeps ICM on HSPI and SD on the board SD bus.

Low-rate SD-on proof command:

```powershell
python esp32\host\stress_test_serial.py --port COM4 --hz 100 200 300 400 500 --capture-s 60 --filter on --sd on
```

## SD-On SPI Tuning (2026-06-17)

Hardware results with batch=32, ICM/mag timeout=5 ms (firmware before this tuning):

| Hz | mean_hz | loop_overruns | Result |
|----|---------|---------------|--------|
| 100 | ~90 | yes | FAIL — contended loop 11 ms > 10 ms period |
| 500 | ~450 | yes | FAIL — ~90% mean Hz |
| 700 | ~630 | yes | FAIL — ~90% mean Hz |
| 770–780 | ~775 | yes | CEILING — filter+SD upper bound |

Root-cause model:

- Historical shared-bus state: ICM and SD shared the same physical SPI bus (GPIO7/8/9 on XIAO Sense).
- SD batch of 32 records fires every `32/Hz` ms. Average SD write latency ≈ 10–12 ms.
- During each write the ICM read waits up to 5 ms (timeout), and at ≤100 Hz the mag read
  also waits up to 5 ms (mag polls every 10 ms = every sample at 100 Hz).
- Contended loop = ICM_wait + mag_wait + VQF ≈ 5+5+1 = 11 ms.
- At 100 Hz (10 ms period): 11 ms > 10 ms → overrun every SD write.
- Ceiling model: avg 10 ms write × 24 writes/s at 780 Hz → 240 ms/s contended →
  760 normal + 240/11 ms contended ≈ 782 Hz effective.

Tuning applied (plan 03-05):

- `SD_WRITE_BATCH_RECORDS` increased 32 → 128. Writes fire 4× less often.
- ICM/mag `spiBusTake` timeout reduced 5 ms → 2 ms. Contended loop = 2+2+1 = 5 ms.
- `sdWriteTask` stack increased 4096 → 8192 bytes to hold the larger batch buffer.

Hardware update (2026-06-18):

- ICM20948 moved off the SD-card SPI bus onto HSPI.
- ICM wiring: CS D6/GPIO43, MISO D5/GPIO6, MOSI D4/GPIO5, SCK D3/GPIO4.
- SD remains on the board SD/Sense SPI wiring via `SD.begin(PIN_SD_CS, SPI, 25000000)`.
- Firmware uses `SPIClass ICM_SPI(HSPI)` for ICM transfers and no longer takes the ICM SPI mutex
  around SD file operations.

Projected outcomes:

- 100 Hz: contended loop 5 ms < 10 ms period → no overruns.
- 500 Hz: batch every 256 ms instead of 64 ms → ~97% mean Hz projected.
- Ceiling: avg 10 ms write × 7.8 writes/s → ~920 Hz projected.

After-state sweep command:

```powershell
python esp32\host\stress_test_serial.py --port COM4 --hz 100 200 300 400 500 600 700 800 --capture-s 60 --filter on --sd on
```

After-state hardware results — intermediate (batch=128, timeout=2 ms):

| Hz | mean_hz | loop_overruns | max_sd_us | Result |
|----|---------|---------------|-----------|--------|
| 500 | 492.0 (98.4%) | 0 | n/a | PASS |
| 550 | 544.7 (99.0%) | 3 | 16,895 µs | FAIL (SPI overruns) |
| 700 | 687.6 (98.2%) | 4 | 16,953 µs | FAIL (SPI overruns) |

Overruns at 550+ Hz because ICM timeout (2 ms) > sample period — any SD write triggers an
overrun. Fix: make ICM reads non-blocking.

After-state hardware results — final (batch=256, ICM SPI non-blocking):

| Hz | mean_hz | loop_overruns | max_sd_us | Result |
|----|---------|---------------|-----------|--------|
| 850 | 825.2 (97.1%) | 0 | n/a | PASS |
| 900 | 855.6 (95.1%) | 0 | n/a | PASS |
| 950 | 864.1 (91.0%) | 2 | 34,390 µs | FAIL (VQF CPU) |
| 1000 | 866.6 (86.7%) | 4 | 29,755 µs | FAIL (VQF CPU) |
| 1050 | 870.1 (82.9%) | 4 | 17,994 µs | FAIL (VQF CPU + gaps) |

All runs: `sd_err=0`, `dup_seq=0`, `gap_seq=0`.

**Ceiling: 900 Hz filter+SD.** Overruns and mean-Hz failures at 950+ Hz are VQF
compute-bound, not SPI contention. The loop takes ~1.1 ms per sample (VQF ~1 ms +
SD queue push + serial), which exceeds the 1.053 ms period at 950 Hz. Non-blocking
ICM reads eliminated all SPI-contention overruns; the residual overruns at 950+ Hz
are natural loop-time variance near the VQF budget.

**Optimization journey:**

| Firmware state | filter+SD ceiling |
|---|---|
| Original (batch=32, 5 ms timeout) | ~780 Hz |
| batch=128, 2 ms timeout | 500 Hz zero-overrun |
| batch=256, non-blocking ICM reads | **900 Hz zero-overrun** |
| VQF CPU hard limit (no SD) | ~1000 Hz |

**Recommended operating cap: 720 Hz** (80% of 900 Hz). Leaves 180 Hz headroom below
the VQF ceiling to absorb WiFi/FreeRTOS jitter and SD card stalls.

Note: ceiling depends on SD card write latency (`max_sd_write_us` in STATUS output).
A slow card (high flash-erase latency) may cause stale ICM samples during writes —
non-blocking reads accept stale data rather than blocking the acquisition loop.
Use `--stream off --sd on` to isolate device+SD from USB serial transport at high rates.

## Firmware Commands

| Command | Use |
|---------|-----|
| `FREQ:<Hz>` | Set acquisition rate |
| `FILTER ON/OFF` | Enable or disable VQF filter cost |
| `RECORD ON/OFF` | Start or stop SD recording |
| `STATUS` | Print counters including SD latency and loop overruns |
| `START` | Start streaming output |

## Manual Serial Capture

```text
FILTER ON
FREQ:500
RECORD ON
START
```

Save Serial output to `bench_500hz.csv` when using CSV mode. First column must be `seq`.

Analyze:

```powershell
python host\analyze_sample_rate.py bench_500hz.csv
python host\analyze_sample_rate.py exported_oe.csv --seq-col 0 --time-col 1
python host\analyze_sample_rate.py step_session.bin --format sd-bin
```

## Interpretation

- `dup_seq > 0`: consecutive frames carried the same sensor payload. After SPI this usually means the firmware requested a frame before the ICM/filter path had a fresh unique reading.
- Duplicates only: lower Hz, tune ICM ODR/DLPF, or disable filter; on USB, close other serial consumers.
- Gaps only: rate too high, host scheduling, or firmware loop cannot schedule the period.
- Low `mean_hz` with no duplicates: firmware/filter/transport cannot sustain the requested cadence.
- SD errors or high `max_sd_write_us`: SD card or filesystem latency is the limiting layer.
- Loop overruns with SD off but filter on: filter/sensor CPU cost is the limiting layer.
- Host gaps while SD is clean: downstream USB/TCP/Open Ephys transport is the likely issue.

Filter type on ESP32 >= v1.6.0: VQF, with ch7-10 as Q15 quaternion when `FILTER ON`.

## SD-First UDP Isolation (2026-06-18)

The ICM20948 now uses HSPI while SD stays on the Sense board SPI bus. Live transport is
also moved out of the acquisition loop:

- Core 1: acquire, filter, enqueue SD first, then non-blocking stream offer.
- Core 0 priority 4: batched SD writer.
- Core 0 priority 1: UDP or queued USB serial writer.
- Stream queue: 16 records, oldest live record dropped when full.
- SD queue: 1024 records and remains the integrity path.

Stream-off hardware baseline with filter and SD enabled:

| Requested Hz | SD-derived Hz | Result |
|---|---:|---|
| 950 | 944.5 | PASS |
| 1000 | 975.8 | PASS |
| 1050 | 991.7 | FAIL, below 95% |
| 1100 | 992.1 | FAIL, saturated near 1 kHz |

The earlier 900 Hz stream-on ceiling included synchronous serial transport work and is
historical. The current architecture must be judged by finalized SD, not UDP delivery.

### Direct Wi-Fi UDP test

Set USB_OPEN_EPHYS_MODE false, flash the firmware, connect Open Ephys to TCP port 5000,
and allow inbound UDP port 55001 through the host firewall. Keep Open Ephys connected so
the firmware learns the UDP destination from the TCP controller IP.

With Open Ephys receiving UDP, use COM only for commands and status:

    python esp32\host\stress_test_serial.py --port COM4 --hz 950 1000 ^
      --capture-s 60 --filter on --sd on --stream on --stream-transport udp

The output reports sd_pass separately from stream_quality. COUNTERS_ONLY means samples
were intentionally not captured from COM because the live path is UDP.

### Receiver-stall test

1. Start the same filter+SD+UDP run.
2. Pause, close, or firewall the Open Ephys UDP receiver for part of the 60-second capture.
3. Leave acquisition and SD recording running.
4. Stop/finalize, retrieve the SD file, and run the SD analyzer.
5. Compare stream counters with SD counters.

Expected behavior:

- stream_queue_drops or stream_send_errors may increase.
- SD queue_drops, write_errors, and mutex_timeouts remain zero.
- Finalized SD has zero duplicate or missing sequences.
- generated and saved counts agree after queue drain/finalization.
- SD-derived rate remains at least 97% of the matching stream-off baseline.
- Open Ephys resumes on later valid datagrams; UDP loss is not replayed.

A no-loss claim applies only to the finalized SD file. UDP/Open Ephys is a lossy live view.
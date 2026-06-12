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
```

Pass per rate:

- no host-visible sequence gaps or duplicates
- enough host-visible rows
- `mean_hz` within 95%-115% of requested Hz
- when SD is enabled, firmware reports saved samples, zero SD errors, and zero loop overruns

Recommended cap = 80% of the highest passing rate.

Outputs:

- `host/stress_results/rate_<hz>__filter_<on|off>__sd_<on|off>.csv`
- `host/stress_results/SUMMARY.md`
- `host/stress_results/SUMMARY.json`

## SD Ground-Truth Verification

After a run with `--sd on`, stop recording, copy `step_session.bin` off the SD card, then analyze it:

```powershell
python host\analyze_sample_rate.py path\to\step_session.bin --format sd-bin
python host\stress_test_serial.py --sd-file path\to\step_session.bin
```

The SD analyzer reads the STEP binary header and checks `seq`, `timestamp_us`, and record alignment.

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

- Duplicates only: lower Hz or disable filter; on USB, close other serial consumers.
- Gaps only: rate too high, host scheduling, or firmware loop cannot schedule the period.
- SD errors or high `max_sd_write_us`: SD card or filesystem latency is the limiting layer.
- Loop overruns with SD off but filter on: filter/sensor CPU cost is the limiting layer.
- Host gaps while SD is clean: downstream USB/TCP/Open Ephys transport is the likely issue.

Filter type on ESP32 >= v1.6.0: VQF, with ch7-10 as Q15 quaternion when `FILTER ON`.

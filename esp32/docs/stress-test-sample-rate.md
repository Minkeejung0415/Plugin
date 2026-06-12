# Sample-rate stress test (ESP32 STEP node)

Find the highest **sustainable** `FREQ:<Hz>` before the stream degrades. Firmware accepts any Hz ≥ 1; USB 115200 and loop time set practical limits.

## What to look for

| Symptom | Meaning | How to detect |
|---------|---------|----------------|
| **Duplicated samples** | Loop or link cannot keep up; same sample emitted twice | Consecutive rows with **same `seq`**; or identical IMU columns row-to-row |
| **Lost samples** | Missed acquisition ticks | **`seq` jumps** by more than 1; or timestamp gaps **>> 1/rate** |

Yes — **checking the CSV (or bench log) for duplicated `seq` or identical consecutive rows is the right manual test.** Automate with `host/analyze_sample_rate.py`.

## Recommended sweep

1. `FILTER OFF` (optional) — isolates transport; turn **FILTER ON** to stress VQF CPU too.
2. Start at **100 Hz** → **200** → **500** → **1000** Hz (or binary search once you see failure).
3. Record **≥ 10 s** per step at each rate.
4. Increase until you see duplicates or gaps, then back off ~20%.

### Automated sweep (`host/stress_test_serial.py`)

Pass per rate: no `seq` gaps/dups, enough rows, and **`mean_hz` within 95%–115%** of requested Hz (e.g. `FREQ:1500` @ 1320 Hz → **FAIL**). Recommended cap = **80%** of the highest passing rate.

### USB theoretical note

Serial bench @ **115200 baud**: raw 11×int16 + 22-byte header ≈ 44 bytes/sample → ~260 Hz theoretical if the loop keeps up. Real max is lower (USB task, `FILTER ON` VQF cost, other USB traffic). Wi-Fi TCP often allows higher sustained rates than USB for the same sketch.

## Tools

| Tool | Use |
|------|-----|
| **`FREQ:<Hz>\n`** | Set rate (Plugin UI or Serial Monitor) |
| **`ENABLE_SERIAL_BENCH`** + CSV | `# seq,ax,...` lines in Serial Monitor; save to file |
| **`host/serial_tcp_bridge.py COMx`** | USB → Open Ephys; log TCP side separately |
| **`host/esp32_tcp_client.py`** | Wi-Fi TCP sanity check |
| **Open Ephys record + export** | Ground truth timeline; export CSV and run analyzer |
| **`host/analyze_sample_rate.py`** | Duplicate / gap report |

### Serial CSV capture

```text
FILTER ON
FREQ:500
START
```

Save Serial output (skip `#` boot lines) to `bench_500hz.csv`. First column must be `seq` (firmware CSV mode: `seq,ax,ay,az,gx,gy,gz,dio,...`).

### Analyze

```powershell
python host\analyze_sample_rate.py bench_500hz.csv
python host\analyze_sample_rate.py exported_oe.csv --seq-col 0 --time-col 1
```

## Interpretation

- **Duplicates only** → try lower Hz or disable filter; on USB, close other serial consumers.
- **Gaps only** → Wi-Fi sleep, host scheduling, or rate set higher than loop can schedule (`period_us` in firmware).
- **Both** → back off Hz; check ICM I2C errors in boot log.

Filter type on ESP32 ≥ v1.6.0: **VQF** (same library as Red Pitaya `sensor_fusion.c` in Plugin repo), ch7–10 Q15 quaternion when `FILTER ON`.

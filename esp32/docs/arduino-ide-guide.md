# Arduino IDE — STEP ESP32-S3 Node

Primary bring-up path for v1: **one Seeed XIAO ESP32S3** — IMU + DIO + Open Ephys TCP. **ESP-NOW is optional** (off by default). Camera deferred to v2 ([camera-feasibility.md](camera-feasibility.md)).

**Full wiring diagram:** [wiring-diagram.md](wiring-diagram.md) (Mermaid + ASCII, v1.3.0 pin table)

## USB-only quick start (Open Ephys — no Wi-Fi)

Board on **USB to PC only**. Firmware **v1.3.4** (`FIRMWARE_VERSION` in sketch). Full detail: [§ USB Open Ephys bridge](#usb-open-ephys-bridge-workflow), [local-open-ephys-setup.md §8](local-open-ephys-setup.md#8-usb-bridge-no-wi-fi).

1. **Flash** — in `arduino/step_node/step_node.ino` set:
   ```cpp
   #define USB_OPEN_EPHYS_MODE true
   ```
   (Sets `ENABLE_TCP false`, `ENABLE_SERIAL_BENCH true`, `SERIAL_OUTPUT_BINARY true`, **100 Hz**, **14 ch**.)
2. **Arduino IDE** — Board **XIAO_ESP32S3**, **USB CDC On Boot → Enabled**, upload.
3. **COM port (Windows)** — **Device Manager → Ports (COM & LPT)** → e.g. `USB Serial Device (COM5)`. Unplug/replug if missing.
4. **Boot check** @ **115200** (optional): `Wi-Fi skipped`, `Serial bench active @115200`, `Format: Open Ephys binary on Serial`. Frames start **5 s** after reset.
5. **Close Serial Monitor** (COM is exclusive).
6. **Bridge** (from repo root; **close Serial Monitor** first):
   ```powershell
   pip install pyserial
   .\host\run_usb_plugin_bridge.ps1 COM5
   ```
   Or: `python host\serial_tcp_bridge.py COM5 --plugin` (Acq Board). For legacy **8- or 11-channel** firmware: set `ESP32_NUM_CHANNELS=8` or `ESP32_NUM_CHANNELS=11` before starting the bridge.
7. **Open Ephys** — **`127.0.0.1:5000`** (not the ESP32 Wi-Fi IP). **Sample rate 100 Hz**; current STEP firmware exposes 14 channels; set `ESP32_NUM_CHANNELS` only for legacy 8- or 11-channel firmware.
   - **Ephys Socket** (built-in): bridge **without** `--plugin`.
   - **Plugin Acq Board** (Minkeejung0415/Plugin, custom GUI): **`run_usb_plugin_bridge.ps1`**, Node IP **`127.0.0.1`** — Wi-Fi not required.

**Quick verify before Open Ephys:** `python host\serial_bench_reader.py COM5 --binary --limit 10`

## Single board quick test

Use **one** XIAO ESP32S3 only — no second node, no ESP-NOW setup.

1. Open `arduino/step_node/step_node.ino`; confirm **`ENABLE_ESPNOW false`** (default).
2. Set **`WIFI_SSID`** / **`WIFI_PASS`**; upload to **XIAO_ESP32S3** (USB CDC On Boot enabled).
3. Serial Monitor @ 115200 → note **`WiFi OK IP=...`** and **`ESP-NOW disabled — single-node mode`**.
4. **TCP test:** `set ESP32_NODE_HOST=<that IP>` then `python host/esp32_tcp_client.py` (port 5000).
5. **Or serial bench:** set **`ENABLE_SERIAL_BENCH true`** and **`ENABLE_TCP false`**, re-upload; watch CSV lines on Serial — no Wi-Fi or host required.

When you add a second board later, set **`ENABLE_ESPNOW true`** on both (master + slave) — see §8.

## Test over USB (no Wi-Fi)

Use when the board is **USB-connected to the PC only** — no router, no TCP.

1. In `arduino/step_node/step_node.ino` set:
   ```cpp
   #define ENABLE_TCP false
   #define ENABLE_SERIAL_BENCH true
   #define ENABLE_ESPNOW false
   ```
2. **Tools → USB CDC On Boot → Enabled**; upload; pick the board’s **COM port**.
3. Optional **`SERIAL_OUTPUT_BINARY true`** — same 22-byte Open Ephys header + 14x int16 as TCP, sent over Serial (for host parsers).
4. Default (**`SERIAL_OUTPUT_BINARY false`**) — one CSV line per sample @ 100 Hz:
   ```
   seq,ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz,dio
   ```
5. **Windows host script** (COM port from Device Manager):
   ```powershell
   pip install pyserial
   python host/serial_bench_reader.py COM5
   ```
   Or: `set SERIAL_PORT=COM5` then `python host/serial_bench_reader.py`

Serial Monitor @ 115200 also works for a quick eyeball test. Boot log should show **`Wi-Fi skipped — USB serial bench mode`**.

## 4-wire ICM20948 + USB to PC

Typical lab setup: **XIAO ESP32-S3 Sense** on USB to the PC, **ICM-20948 on four wires only** (no DIO, SD, camera, or second board).

### Wiring

See **[wiring-diagram.md](wiring-diagram.md)** for Mermaid/ASCII diagrams and the full v1.3.0 pin table.

| ICM-20948 | Connect to XIAO Sense |
|-----------|------------------------|
| VCC / VDD | **3V3** (3.3 V — not 5 V; may be on PCB bottom) |
| GND | **GND** |
| SDA *(or **EDA**)* | **D4** (GPIO **5**, I2C data) |
| SCL *(or **ECL**)* | **D5** (GPIO **6**, I2C clock) |
| AD0 *(or **ADO**)* | **GND** → addr **0x68**; or **3V3** → **0x69** |
| **NCS** | **3V3** — **required** on EDA/ECL modules (I2C mode) |

### ICM-20948 pin mapping (I2C mode — dual silk)

Many modules label **both** `VCC/GND/SDA/SCL` **and** `EDA/ECL/ADO/NCS/…` on the same PCB. **SDA = EDA** and **SCL = ECL** (same copper). Wire **one pad per signal** — do not connect D4 to both SDA and EDA.

| Module label | Connect to | Notes |
|--------------|------------|-------|
| **VCC / VDD** | XIAO **3V3** | 3.3 V only |
| **GND** | XIAO **GND** | Common ground |
| **SDA** *or* **EDA** | XIAO **D4** (GPIO5) | I2C data — pick **one** pad |
| **SCL** *or* **ECL** | XIAO **D5** (GPIO6) | I2C clock — pick **one** pad |
| **AD0** *or* **ADO** | **GND** (0x68) or **3V3** (0x69) | Your setup: **ADO→GND**, `#define ICM20948_ADDR 0x68` |
| **NCS** | **3V3** | Required for I2C on boards that expose it |
| **INT** | NC | Not needed for streaming |
| **FSYNC** | NC or GND | Optional |

Full diagram: [wiring-diagram.md](wiring-diagram.md#icm-20948-module-labels-eda--ecl-silk)

The sketch uses `PIN_I2C_SDA 5` and `PIN_I2C_SCL 6`, which match the [Seeed pin map](https://wiki.seeedstudio.com/xiao_esp32s3_pin_multiplexing/) for D4/D5. In Arduino IDE, select board **XIAO_ESP32S3** (not a generic ESP32-S3 dev module).

**I2C address:** `ICM20948_ADDR 0x68` when **ADO/AD0→GND** (user-confirmed); use **`0x69`** if ADO tied to 3V3.

### Recommended defines (USB-only)

Change from Wi-Fi/TCP config — PC is on USB, not on the AP:

```cpp
#define ENABLE_TCP false
#define ENABLE_SERIAL_BENCH true
#define ENABLE_ESPNOW false
#define ENABLE_SD false
#define SERIAL_OUTPUT_BINARY false   // CSV lines; true = Open Ephys binary on Serial
```

Leave `PIN_I2C_*` at 5/6 unless your wiring differs.

### Arduino IDE steps

1. **Tools → Board → XIAO_ESP32S3**
2. **Tools → USB CDC On Boot → Enabled**
3. **Tools → Port →** your COM port (e.g. COM5 in Device Manager)
4. Upload `arduino/step_node/step_node.ino`

### Serial Monitor — what success looks like

After reset @ **115200 baud**, you should see lines like:

```
STEP node (Arduino) starting
ICM20948: OK
Wi-Fi skipped — USB serial bench mode
ESP-NOW disabled — single-node mode
Serial bench active @115200
Format: CSV seq,ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz,dio
```

- **`ICM20948: OK at I2C 0x69 WHO_AM_I=0xEA`** — real chip; CSV should show changing ax/ay/az (not fixed sinewave)
- **`ICM20948: synthetic fallback`** — no I2C ACK: check VCC/GND, SDA/SCL on **D4/D5**, address 0x68 vs 0x69, and 3.3 V only.

After **5 s pause**, first data line:

```
# STEP boot complete icm=OK addr=0x69
0,<real ax>,<real ay>,...
```

**DIO (ch6):** `PIN_DIO` defaults to **D0 (GPIO 1)** with internal pull-up. Packed int16: **bit 0 = level** (1 idle/high, 0 pressed to GND); **bits 1–15 = debounced edge count**. If nothing is wired, ch6 sits at 1 (or 0 if floating) — harmless for IMU-only tests.

## Phase 2: Test DIO over USB

No Wi-Fi or TCP required — default sketch uses USB serial bench mode (`ENABLE_TCP false`, `ENABLE_SERIAL_BENCH true`).

### Wiring

| Signal | XIAO pad | Notes |
|--------|----------|-------|
| DIO | **D0** (GPIO 1) | `PIN_DIO` in sketch; change `#define` if wired elsewhere |
| Button | D0 → **GND** | Momentary switch; internal pull-up — press pulls low |

Optional: add a 100 nF cap D0–GND for noisy benches (not required for desk test).

### Boot check

After reset @ 115200, look for:

```
DIO: GPIO1 (pad D0) pull-up — initial level=1 (1=idle, 0=GND)
```

Idle (button open): level **1**. Pressed: level **0**.

### 3-step USB test

1. **Upload** `arduino/step_node/step_node.ino` with USB CDC enabled; open Serial Monitor @ 115200 or run `python host/serial_bench_reader.py COM5` (close Monitor if port busy).
2. **Wire** a momentary button from **D0 to GND**. Wait for CSV after the 5 s boot pause (`# STEP boot complete icm=OK ... dio_ch6=level|edges`).
3. **Press/release** the button — within **~20 ms** (one CSV row @ 100 Hz) **ch6** (8th CSV column) should show:
   - **Level:** odd values (1, 3, 5…) = high; even values (0, 2, 4…) = low — decode with `ch6 & 1`
   - **Edges:** each debounced transition increments the upper bits — `(ch6 >> 1)` counts presses+releases

Example CSV row (button pressed once):

```
12,1234,5678,...,0,0
```

Here `dio=0` → level 0 (pressed). After release, `dio=1` (or higher if edges accumulated).

**Pass:** ch6 level toggles 1↔0 with button; edge count increases on each debounced transition. **Fail:** ch6 stuck — check D0 wiring and that `PIN_DIO` matches your pad.

### Windows — read samples on the PC

```powershell
pip install pyserial
python host/serial_bench_reader.py COM5
```

Replace `COM5` with your port. Close Serial Monitor first if the port is busy.

### Serial Monitor shows only numbers (no boot text)

**Cause:** USB CDC often connects *after* the board has already printed boot lines, or CSV scrolls text away immediately.

**Fix (built into firmware v1.2+):**
1. **Tools → USB CDC On Boot → Enabled**; baud **115200**
2. Press **Reset** on the board *after* opening Serial Monitor (or re-upload)
3. Wait **5 seconds** — CSV is paused so diagnostics stay visible
4. You should see the `BOOT DIAGNOSTICS` banner, I2C scan, and `ICM20948: OK` or `synthetic fallback`

First CSV line is prefixed: `# STEP boot complete icm=OK|FALLBACK`

If you still see sinewave + `az=16384` only → **FALLBACK** (I2C not talking to real chip). Check wiring checklist in [4-wire section](#4-wire-icm20948--usb-to-pc). Optional: install **Adafruit ICM20948** library and swap driver if raw registers fail on your breakout.

## Open Wi-Fi and phone hotspot

**Repo default STA credentials** (edit before upload):

```cpp
#define WIFI_SSID "YOUR_HOTSPOT"
#define WIFI_PASS "yourpassword"   // "" for open networks (e.g. ubcvisitor)
#define ENABLE_TCP true
#define ENABLE_SERIAL_BENCH false
```

Empty `WIFI_PASS` → `WiFi.begin(WIFI_SSID)` only. On success Serial shows `WiFi OK IP=…` and `TCP listen :5000`.

### Hotspot troubleshooting (connect timeout)

If Serial shows dots for ~30 s then **`STA failed (status=…)`**, the board **automatically starts Soft AP** `STEP_ESP32` / password `step1234` and prints **`WiFi OK AP IP=192.168.4.1`**. TCP port **5000** still works.

| Check | Action |
|-------|--------|
| **2.4 GHz band** | ESP32-S3 cannot join **5 GHz–only** hotspots. iPhone: *Settings → Personal Hotspot → Maximize Compatibility* ON. Android: enable 2.4 GHz / compatibility mode. |
| **SSID / password** | Must match hotspot exactly (`WL_NO_SSID_AVAIL` = wrong name or hidden 5 GHz). |
| **Same network** | PC and ESP32 must share one SSID (hotspot or Soft AP). |
| **Status codes** | `WL_CONNECT_FAILED` → wrong password; `WL_NO_SSID_AVAIL` → SSID not seen (often 5 GHz). |
| **After timeout** | PC joins **`STEP_ESP32`** / **`step1234`**, host **`192.168.4.1`**, port **5000**. |

**Verify from PC (after WiFi OK or Soft AP):**

```powershell
ping 192.168.4.1
set ESP32_NODE_HOST=192.168.4.1
python host\esp32_tcp_client.py
```

**Windows:** disable VPN; allow Python through firewall; some corporate Wi-Fi blocks device-to-device traffic (use Soft AP fallback instead).

**Campus `ubcvisitor`:** open network but often **client isolation** — PC cannot reach ESP32 even when both “connected.” Use phone hotspot or Soft AP.

**General:** open/hotspot traffic is not encrypted end-to-end beyond WPA on the hotspot itself.

### Same Wi-Fi / correct password — still fails (checklist)

Use this when you believe SSID/password are right and the PC is on the **same** network as the ESP32.

| Step | What to check | Pass criterion |
|------|----------------|----------------|
| 1 | Serial @ 115200 after reset | **Not** `Wi-Fi skipped` (means `USB_OPEN_EPHYS_MODE true` — re-flash with `false`) |
| 2 | Build flags | `USB_OPEN_EPHYS_MODE false`, `ENABLE_TCP true` |
| 3 | STA vs Soft AP | **`WiFi STA CONNECTED`** + IP banner, **not** only `STEP_ESP32` fallback |
| 4 | Scan line | Target SSID **seen** in scan; if **NOT in scan** → typo, hidden-only, or **5 GHz-only** AP |
| 5 | While connecting | `disc_reason=15` or `202` → password/WPA3; `201` → SSID/band |
| 6 | After connect | `TCP listen :5000`; every 10 s: `STATUS` block with same **IP** |
| 7 | PC routing | `ping <IP>` succeeds; disable VPN |
| 8 | TCP | `python host\esp32_tcp_client.py --host <IP>` logs `handshake: 14 channels…` |
| 9 | Open Ephys Plugin | Node IP = **Serial IP** (not `127.0.0.1` unless USB bridge); port **5000** |
| 10 | Isolation | Campus/guest Wi-Fi often blocks PC↔device; iPhone hotspot needs **Maximize Compatibility** |

**Paste from Serial for support:** lines from `Scanning 2.4 GHz` through `WiFi OK` or `STA failed`, including any `disc_reason=…`, plus one `--- STA status ---` block (or `Soft AP` if fallback).

**PC commands** (replace `192.168.1.42` with Serial IP):

```powershell
ping 192.168.1.42
Test-NetConnection -ComputerName 192.168.1.42 -Port 5000
$env:ESP32_NODE_HOST="192.168.1.42"
python host\esp32_tcp_client.py --host 192.168.1.42
```

Optional raw TCP (PowerShell 7+): send handshake lines, then binary follows after `START`.

```powershell
$tcp = [System.Net.Sockets.TcpClient]::new("192.168.1.42", 5000)
$w = [System.IO.StreamWriter]::new($tcp.GetStream()); $w.AutoFlush = $true
$w.WriteLine("REDPITAYA"); $w.WriteLine("START")
```

Type **`STATUS`** in Serial Monitor anytime to re-print IP / TCP client state (firmware v1.3.4+).

## PC cannot join STEP_ESP32 (USB bridge — no Wi-Fi)

Use this when Windows **will not connect** to the board’s Soft AP (`STEP_ESP32`) or when **PC↔ESP32 Wi-Fi is impossible** (VPN, corporate drivers, client isolation). **IMU stays on 4 wires + USB only** — no hotspot, no joining the ESP32 AP.

### Why Soft AP fails on Windows

- Some Wi-Fi drivers **refuse ad-hoc / ESP Soft AP** connections or connect then drop.
- **VPN** (Cisco, GlobalProtect, etc.) captures routes and blocks local AP traffic.
- **Corporate / campus Wi-Fi** policies disable “connect to device AP” profiles.
- Wrong password, cached “forget network”, or PC on **5 GHz–only** adapter while ESP AP is 2.4 GHz.
- **USB phone tethering** gives the PC internet but does **not** put the PC on the ESP32’s `STEP_ESP32` network — still no path to `192.168.4.1`.

**Brief Soft AP fixes to try:** exact SSID `STEP_ESP32`, password `step1234`, forget old network entry, disable VPN, use a 2.4 GHz–capable adapter, reboot board after timeout message. If none work → USB bridge below.

### USB Open Ephys bridge workflow

**Default (v1.3.4):** **`USB_OPEN_EPHYS_MODE true`** — USB binary bench, no on-board TCP. For **Plugin on Wi-Fi**, set **`USB_OPEN_EPHYS_MODE false`** and re-flash; then use the IP from Serial Monitor, not `127.0.0.1`.

1. **Sketch** — set **`USB_OPEN_EPHYS_MODE true`** in `arduino/step_node/step_node.ino` (or manually set):
   ```cpp
   #define USB_OPEN_EPHYS_MODE true
   ```
   Equivalent manual defines:
   ```cpp
   #define ENABLE_TCP false
   #define ENABLE_SERIAL_BENCH true
   #define SERIAL_OUTPUT_BINARY true
   #define ENABLE_ESPNOW false
   #define ENABLE_SD false
   ```
2. **Flash** XIAO (USB CDC On Boot enabled). Serial Monitor @ 115200 should show **`Wi-Fi skipped`**, **`Serial bench active @115200`**, and **`Format: Open Ephys binary on Serial`**. Sample frames start **5 s after boot** (`BOOT_CSV_DELAY_MS`).
3. **Verify USB stream** (close Monitor when done — port is exclusive):
   ```powershell
   python host\serial_bench_reader.py COM5 --binary --limit 10
   ```
   Expect `frame=0 ch=(...)` lines. If you only see boot text or CSV, fix sketch flags before running the bridge.
4. **Close Serial Monitor** (COM port is exclusive).
5. **Bridge on PC** (start **after** step 3 works, or wait **>5 s** after reset):
   ```powershell
   pip install pyserial
   .\host\run_usb_plugin_bridge.ps1 COM5
   ```
   Replace `COM5` with your port. **Plugin Acq Board** uses `--plugin` (included in the script). **Ephys Socket:** `python host\serial_tcp_bridge.py COM5` without `--plugin`. First-frame wait defaults to **15 s**.
6. **Open Ephys:**
   - **Plugin Acq Board:** Node IP **`127.0.0.1`**, port **5000**, **100 Hz** (custom GUI + Plugin patches).
   - **Ephys Socket:** TCP client → **`127.0.0.1:5000`** (no `REDPITAYA`/`START` on GUI side).

The bridge reads Open Ephys **binary frames** from USB and serves them on **localhost:5000**. With **`--plugin`**, it answers **`REDPITAYA`** / **`START`** like ESP32 Wi-Fi firmware. Test without Open Ephys:

```powershell
python host\esp32_tcp_client.py
set ESP32_NODE_HOST=127.0.0.1
```

(`esp32_tcp_client.py` is unchanged — point it at `127.0.0.1` while the bridge runs.)

**CSV fallback** (if binary not enabled): `python host\serial_tcp_bridge.py COM5 --csv`

## 1. Install Arduino IDE

1. Install [Arduino IDE 2.x](https://www.arduino.cc/en/software).
2. **File → Preferences → Additional boards manager URLs**, add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. **Tools → Board → Boards Manager** → install **esp32** by Espressif (≥ 3.0 recommended).
4. **Tools → Board → esp32** → select **XIAO_ESP32S3** (Seeed XIAO ESP32S3).

If **XIAO_ESP32S3** is missing, add Seeed board package:
```
https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
```
Then install **Seeed ESP32 Boards** and pick **XIAO_ESP32S3**.

## 2. Open the sketch

```
arduino/step_node/step_node.ino
```

Edit the config block at the top:

| Define | Purpose |
|--------|---------|
| `WIFI_SSID` / `WIFI_PASS` | Lab Wi-Fi; **`WIFI_PASS ""`** = open network |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | ICM-20948 I2C (default D4/D5 → GPIO 5/6) |
| `PIN_DIO` | Digital input (default D0 → GPIO 1) |
| `ICM20948_ADDR` | `0x69` or `0x68` if AD0 grounded |
| **`ENABLE_ESPNOW`** | **`false` = single board (default); `true` = multi-node sync** |
| `NODE_IS_MASTER` | Only when `ENABLE_ESPNOW true` |
| `ENABLE_SD` | `true` on Sense board with SD wired |
| `ENABLE_TCP` | Open Ephys TCP server on port 5000 (`false` for USB-only) |
| **`USB_OPEN_EPHYS_MODE`** | **`true`** = USB serial → `serial_tcp_bridge.py` preset (overrides TCP/bench/binary flags) |
| `ENABLE_SERIAL_BENCH` | `true` = stream on USB Serial @115200 |
| `SERIAL_OUTPUT_BINARY` | `true` = Open Ephys binary on Serial; `false` = CSV |

## 3. Libraries (Board Manager / Library Manager)

Built-in with ESP32 core (no extra install):

- **WiFi**, **WiFiClient**, **WiFiServer**
- **esp_now**
- **Wire** (ICM-20948 I2C)
- **SD**, **SPI** (when `ENABLE_SD`)

Optional (not required for v1 sketch):

- **Adafruit ICM20948** — replace minimal I2C in sketch for full register map

## 4. Pin map — Seeed XIAO ESP32S3

| Signal | Default GPIO | XIAO pad | Notes |
|--------|--------------|----------|-------|
| I2C SDA | 5 | D4 | ICM-20948 |
| I2C SCL | 6 | D5 | ICM-20948 |
| DIO in | 1 | D0 | Pull-up input |
| SD CS | 21 | — | Sense expansion; verify schematic |
| 3V3 / GND | — | — | Common ground with IMU |

Confirm against [Seeed XIAO ESP32S3 wiki](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) if your carrier differs.

## 5. Flash

1. Connect USB-C, select correct **Port**.
2. **Tools → USB CDC On Boot → Enabled** (serial monitor).
3. **Upload**.
4. **Serial Monitor @ 115200** — expect `WiFi OK IP=...` and `TCP listen :5000`.

## 6. Red Pitaya–compatible TCP (Open Ephys)

Matches STEP / Red Pitaya behavior:

| Step | Action |
|------|--------|
| Connect | TCP to node IP, port **5000** |
| Handshake | Send `REDPITAYA\n` → `14 channels; sample_rate=<Hz>; node=esp32s3_arduino; filter=on\|off` |
| Start | Send `START\n` (Plugin may send `FREQ:<Hz>` first) |
| Payload | 22-byte little-endian header + **14 x int16** channel-major |

TCP text commands (v2.0, newline-terminated):

| Command | Example | Effect |
|---------|---------|--------|
| `REDPITAYA` | `REDPITAYA\n` | Handshake + `OK CHANNELS:14` |
| `START` | `START\n` | Begin binary stream |
| `FREQ:` | `FREQ:100\n` | Sample rate **50–200 Hz** (default 100) |
| `CFG` | `CFG 0 ACC 2\n` | ICM accel full-scale preset 0–3 |
| `CFG` | `CFG 0 GYR 1\n` | ICM gyro full-scale preset 0–3 |
| `FILTER` | `FILTER ON\n` / `FILTER OFF\n` | VQF on ch9-12 when ON (see channel map) |
| `STATUS` | `STATUS\n` | Wi-Fi / AP status (Serial or TCP) |

Channel map:

| Ch | `FILTER OFF` | `FILTER ON` |
|----|----------------|-------------|
| 0–2 | ax, ay, az (int16) | ax, ay, az |
| 3-5 | gx, gy, gz | gx, gy, gz |
| 6-8 | mx, my, mz | mx, my, mz |
| 9-12 | 0 | qw, qx, qy, qz (Q15 unit quaternion) |
| 13 | DIO: bit0 level, bits1-15 edge count | same |


Host test:

```bash
pip install numpy
set ESP32_NODE_HOST=<node-ip>
python host/esp32_tcp_client.py
```

**Open Ephys:** Ephys Socket plugin → TCP client → same IP/port and framing. Custom Red Pitaya plugin vs Ephys Socket: [open-ephys-plugin.md](open-ephys-plugin.md).

## 7. Serial bench mode (no Wi-Fi)

See **Test over USB (no Wi-Fi)** above. Legacy summary:

```cpp
#define ENABLE_TCP false
#define ENABLE_SERIAL_BENCH true
```

Use `host/serial_bench_reader.py COMx` on Windows or Serial Monitor @ 115200.

## 8. ESP-NOW multi-node (optional)

Requires **`ENABLE_ESPNOW true`** on every board. Skip this section for single-board v1 tests.

1. Flash **master** with `NODE_IS_MASTER true`, **slaves** with `false`.
2. All nodes on same Wi-Fi channel (ESP-NOW init after `WiFi.begin`).
3. Master broadcasts `{seq, time_us}` each sample; slaves log offset (extend sketch for production sync).

## 9. SD logging

1. Set `ENABLE_SD true`.
2. Session append file: `/step_session.bin` (seq + 16 bytes channels per frame).
3. Copy SD to PC for offline analysis.

## 10. ESP-IDF path (advanced)

Same architecture in `firmware/` for teams using **idf.py** (camera hooks, FreeRTOS tasks). Arduino is recommended for lab v1 bring-up.

## Troubleshooting

| Issue | Fix |
|-------|-----|
| ICM not found | Check wiring, I2C address 0x68/0x69, run with synthetic fallback |
| No TCP client | Firewall; `ping` node IP; after timeout use Soft AP `192.168.4.1`; run `host/esp32_tcp_client.py` |
| Wi-Fi connect timeout | See [Hotspot troubleshooting](#hotspot-troubleshooting-connect-timeout); join `STEP_ESP32` / `step1234` on PC |
| USB bench empty / bridge `no serial frames yet` | Set **`USB_OPEN_EPHYS_MODE true`**, re-flash; wait **>5 s** after boot; verify with `serial_bench_reader.py --binary`; close Serial Monitor; use latest `serial_tcp_bridge.py` (accepts firmware `bit_depth=16`; `--plugin` for Acq Board) |
| ESP-NOW no sync | Same RF channel; Wi-Fi must be started before `esp_now_init` |
| SD fail | Sense CS pin, FAT32 card, `ENABLE_SD` |

---
*v1 guide — camera deferred; see `.planning/ROADMAP.md`*

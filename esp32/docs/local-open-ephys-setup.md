# ESP32-S3 + Open Ephys — Local Integration Guide (Windows)

Step-by-step setup for **Seeed XIAO ESP32-S3** (ICM-20948 + DIO) with the custom **Acquisition Board** plugin from [Minkeejung0415/Plugin](https://github.com/Minkeejung0415/Plugin.git).

| Repo | Path |
|------|------|
| ESP32-S3 + host scripts | `C:\Users\justi\ESP32-S3` (pull `main`) |
| Plugin patches | `C:\Users\justi\Plugin` (need commits **`217425a`** + **`e298679`**) |

| Path | When | Open Ephys |
|------|------|------------|
| **USB + Plugin (recommended if Wi‑Fi broken)** | Board on **USB power/data** to PC; `USB_OPEN_EPHYS_MODE true` | `host\run_usb_plugin_bridge.ps1 COMx` → Acq Board **Node IP `127.0.0.1`**, port **5000**, **100 Hz**, **14 ch** — **Wi‑Fi not required** |
| **Wi‑Fi + Plugin** | `USB_OPEN_EPHYS_MODE false`, STA or Soft AP | Acq Board → **Serial Monitor IP** (e.g. `192.168.4.1`), port **5000** |
| **USB + Ephys Socket** | Same USB firmware; bridge **without** `--plugin` | Built-in Ephys Socket → **`127.0.0.1:5000`** |

**Architecture (USB + Plugin):** firmware → Open Ephys **binary on USB serial @ 100 Hz, 14 ch** → `serial_tcp_bridge.py --plugin` → TCP **`127.0.0.1:5000`** with `REDPITAYA` / `START` / `STARTED` / `SENSORS` replies → Plugin **Acq Board** (same handshake as direct ESP32 TCP). Running Wi‑Fi TCP and the bridge at once is unnecessary and shares one COM port; use the bridge for Plugin on USB.

See also: [open-ephys-plugin.md](open-ephys-plugin.md), [arduino-ide-guide.md](arduino-ide-guide.md), [.planning/PLUGIN-INTEGRATION.md](../.planning/PLUGIN-INTEGRATION.md).

---

## 1. Prerequisites

- **Hardware:** XIAO ESP32-S3, ICM-20948 (3.3 V I2C), USB-C, **2.4 GHz** Wi-Fi
- **Arduino IDE 2.x** — board **XIAO_ESP32S3**, **USB CDC On Boot: Enabled**
- **Python 3** — `pip install pyserial numpy`
- **Open Ephys GUI** — custom build (stock installer lacks ESP32 patches)
- **VS 2022 + CMake** — [Compile GUI](https://open-ephys.github.io/gui-docs/Developer-Guide/Compiling-the-GUI.html), [Compile plugins](https://open-ephys.github.io/gui-docs/Developer-Guide/Compiling-plugins.html)
- **OpenSim 4.5** (optional) — Plugin Python bridge scripts

---

## 2. Clone / pull

```powershell
cd C:\Users\justi\ESP32-S3
git pull origin main

cd C:\Users\justi\Plugin
git pull origin main
git log -2 --oneline   # expect 217425a, e298679
```

---

## 3. Flash ESP32

Edit `arduino/step_node/step_node.ino` — set `WIFI_SSID` / `WIFI_PASS` for Wi-Fi preset.

### Preset A — Plugin + Wi-Fi (primary)

```cpp
#define USB_OPEN_EPHYS_MODE false   // default
```

Upload → Serial @ 115200: `WiFi OK IP=…`, `TCP listen :5000`.  
Fallback AP (after ~30 s STA timeout): **`STEP_ESP32`** / **`step1234`** → **`192.168.4.1:5000`**.  
Firmware **1.3.2+** logs AP MAC, channel **6**, WPA2, broadcast ON. Serial command **`AP?`** repeats status.

**Windows cannot see/join STEP_ESP32?** Try in order:

1. Confirm Serial shows `WiFi OK Soft AP started` and `Stations connected: 0` (AP is up on the board).
2. **Settings → Network → Wi‑Fi → STEP_ESP32 → Properties** → set **Random hardware addresses** to **Off** for this network; **Forget** network, reboot board, retry.
3. Airplane mode **On** 10 s → **Off**; or disable/enable Wi‑Fi adapter in **Device Manager**.
4. Connect from admin PowerShell:  
   `netsh wlan connect name="STEP_ESP32" ssid="STEP_ESP32" interface="Wi-Fi"`
5. If still failing (common with some Intel/Realtek drivers): use **USB path** below — no Wi‑Fi needed.

### Preset B — USB + Ephys Socket (alternate)

```cpp
#define USB_OPEN_EPHYS_MODE true
```

→ `Serial bench active`, binary after **5 s** boot delay. Use **`--plugin`** on the bridge for AcqBoard (see §8b).

---

### Preset B2 — USB powered + Plugin Acq Board (no Wi‑Fi)

Use when the XIAO is **USB-powered from the PC** (no battery) and Wi‑Fi STA / `STEP_ESP32` is unreliable.

1. Flash with **`USB_OPEN_EPHYS_MODE true`** (sketch default in v1.3.4).
2. Serial @ 115200: `Wi-Fi skipped`, `Serial bench active`, `Format: Open Ephys binary on Serial`.
3. **Close Serial Monitor**; wait **>5 s** after reset.
4. Run bridge (replace `COM5` with Device Manager port):

```powershell
cd C:\Users\justi\ESP32-S3
pip install pyserial
.\host\run_usb_plugin_bridge.ps1 COM5
```

Or: `python host\serial_tcp_bridge.py COM5 --plugin`

5. **Custom Open Ephys GUI** (Plugin built per §6): **Sources → Acq Board** → **Node IP `127.0.0.1`**, **100 Hz** → Record → **14 channels**.

Plugin repo must include commits **`217425a`** + **`e298679`** (`isEsp32Node`, TCP binary stream). Wi‑Fi is **not** used on this path.

---

## 4. Wiring

| ICM | XIAO |
|-----|------|
| VCC | 3V3 |
| GND | GND |
| SDA | D4 (GPIO5) |
| SCL | D5 (GPIO6) |
| ADO | GND=0x68 / 3V3=0x69 — match `ICM20948_ADDR` |

DIO: **D0** (GPIO1), pull-up; button to GND for tests.

Full diagram: [wiring-diagram.md](wiring-diagram.md).

---

## 5. Pre-flight (no GUI)

Close Serial Monitor before host scripts (exclusive COM port).

```powershell
cd C:\Users\justi\ESP32-S3
pip install pyserial numpy

# USB IMU
python host\serial_bench_reader.py COM5

# USB binary (Preset B)
python host\serial_bench_reader.py COM5 --binary --limit 10

# Wi-Fi TCP (Preset A)
$env:ESP32_NODE_HOST = "192.168.x.x"
python host\esp32_tcp_client.py
```

Pass: handshake `14 channels; sample_rate=100; node=esp32s3_arduino`, then samples.

---

## 6. Build Open Ephys + Plugin

### Layout

```text
C:\Users\justi\open-ephys\
  plugin-GUI\          # branch development-juce8
  acquisition-board\   # open-ephys-plugins/acquisition-board
```

```powershell
mkdir C:\Users\justi\open-ephys -ErrorAction SilentlyContinue
cd C:\Users\justi\open-ephys
git clone -b development-juce8 https://github.com/open-ephys/plugin-GUI.git
git clone https://github.com/open-ephys-plugins/acquisition-board.git
```

### Copy Plugin sources into `acquisition-board\Source\`

| From `C:\Users\justi\Plugin\` | To `Source\` |
|-------------------------------|---------------|
| `acqboard.ccp` | `devices\redpitaya\AcqBoardRedPitaya.cpp` |
| `devices\redpitaya\AcqBoardRedPitaya.h` | `devices\redpitaya\` |
| `Acqboard.h` | `devices\AcquisitionBoard.h` |
| `devicethread.cpp` | `DeviceThread.cpp` |
| `device editor.cpp` / `.h` | `DeviceEditor.cpp` / `.h` |

```powershell
$P = "C:\Users\justi\Plugin"
$D = "C:\Users\justi\open-ephys\acquisition-board\Source"
Copy-Item "$P\acqboard.ccp" "$D\devices\redpitaya\AcqBoardRedPitaya.cpp" -Force
Copy-Item "$P\devices\redpitaya\AcqBoardRedPitaya.h" "$D\devices\redpitaya\" -Force
Copy-Item "$P\Acqboard.h" "$D\devices\AcquisitionBoard.h" -Force
Copy-Item "$P\devicethread.cpp" "$D\DeviceThread.cpp" -Force
Copy-Item "$P\device editor.cpp" "$D\DeviceEditor.cpp" -Force
Copy-Item "$P\device editor.h" "$D\DeviceEditor.h" -Force
```

### Build

```powershell
cd C:\Users\justi\open-ephys\plugin-GUI\Build
cmake -G "Visual Studio 17 2022" -A x64 ..
# Open open-ephys-GUI.sln → Build Release

cd C:\Users\justi\open-ephys\acquisition-board\Build
cmake -G "Visual Studio 17 2022" -A x64 ..
# Open OE_PLUGIN_acquisition-board.sln → Release → INSTALL
```

Launch the **GUI you built**, not the stock installer (unless you copy the plugin DLL manually).

---

## 7. Open Ephys GUI (Plugin path)

```powershell
$env:ESP32_NODE_HOST = "192.168.x.x"   # optional default
```

1. Launch custom GUI.
2. Add **Sources → Acq Board**.
3. Set **Node IP** in editor (triggers re-detect).
4. **Sample rate 100 Hz**.
5. Record — expect **14 channels**.

Detection tries `rp-*.local` first, then Node IP / `ESP32_NODE_HOST`.

---

## 8. USB bridge (no Wi‑Fi) — Plugin Acq Board step-by-step

Use when the board is **USB-powered on the PC** and you want the **original Acquisition Board plugin** without fixing Wi‑Fi.

**Quick launcher:** `host\run_usb_plugin_bridge.ps1 COM5` or `host\run_usb_plugin_bridge.bat COM5`

Use when Windows will not join **`STEP_ESP32`** or iPhone hotspot STA fails.

1. Edit `arduino/step_node/step_node.ino` → `#define USB_OPEN_EPHYS_MODE true` → Upload.
2. Serial @ 115200 should show `Serial bench active` and `Format: Open Ephys binary on Serial`.
3. **Close Serial Monitor** (exclusive COM port).
4. Find COM port: **Device Manager → Ports (COM & LPT)** → e.g. `COM5`.
5. Wait **>5 s** after boot for binary frames.

Verify USB first:

```powershell
cd C:\Users\justi\ESP32-S3
pip install pyserial
python host\serial_bench_reader.py COM5 --binary --limit 10
```

### 8a. Ephys Socket (built-in plugin, no Plugin repo build)

```powershell
python host\serial_tcp_bridge.py COM5
```

Open Ephys: **Ephys Socket** → TCP client → **`127.0.0.1:5000`**.  
The bridge streams binary immediately (no `REDPITAYA` / `START` on the GUI side).

### 8b. Plugin AcqBoard over USB (Minkeejung0415/Plugin)

```powershell
.\host\run_usb_plugin_bridge.ps1 COM5
```

Bridge listens on **`127.0.0.1:5000`** and answers like ESP32 Wi‑Fi firmware + Plugin expectations:

| Client sends | Bridge replies |
|--------------|----------------|
| `REDPITAYA\n` | `14 channels; sample_rate=100; node=esp32s3_arduino\n` and `OK CHANNELS:14\n` |
| `START\n` | `STARTED BIN:step_usb_bridge\n` and `SENSORS:0,ICM20948\n` |
| (then) | Open Ephys binary frames from USB serial |

Open Ephys (custom GUI + Plugin):

1. **Sources → Acq Board**
2. **Node IP:** `127.0.0.1` (not the ESP32 Wi‑Fi IP)
3. **Sample rate:** 100 Hz → Record → **14 channels**

Optional: forward `REDPITAYA` / `START` to the board over USB for Serial Monitor logging (`TCP streaming START` on `START`).

Test handshake without GUI:

```powershell
$env:ESP32_NODE_HOST = "127.0.0.1"
python host\esp32_tcp_client.py
```

**Limitations vs Wi‑Fi ESP32:** extra hop (USB→Python→TCP); no UDP 55001; no multi-client; COM port exclusive; ~same 100 Hz if USB keeps up. Plugin must include ESP32 TCP patches (`e298679`).

---

## 9. OpenSim (optional)

Scripts in **`C:\Users\justi\Plugin\`**:

```powershell
cd C:\Users\justi\Plugin
$env:OPENSIM_ESP32_8CH = "1"
pip install numpy imufusion
python ephys_to_opensim_bridge.py listen
```

Edit model paths in script headers for your PC. Live visualizer: **Python 3.8** + OpenSim 4.5.

---

## 10. Troubleshooting

Full step-by-step for “correct Wi-Fi, same network, still fails”: [arduino-ide-guide.md § Same Wi-Fi checklist](arduino-ide-guide.md#same-wi-fi--correct-password--still-fails-checklist).

| Symptom | Fix |
|---------|-----|
| `synthetic fallback` | I2C wiring, `ICM20948_ADDR`, NCS→3V3 |
| `Wi-Fi skipped` on Serial | `USB_OPEN_EPHYS_MODE` is **true** — PC must use `serial_tcp_bridge.py` → `127.0.0.1:5000`, not node Wi-Fi IP |
| Wi-Fi fails / `status=6 WL_DISCONNECTED` | Board auto-starts Soft AP — PC joins **`STEP_ESP32`** / **`step1234`** → **`192.168.4.1`**. **Fix STA:** iPhone *Maximize Compatibility* ON; exact `WIFI_SSID`; read Serial `disc_reason` (15/202 = bad password/WPA3, 201 = SSID/5 GHz/hidden). **PC won't join AP:** disable random MAC, `netsh wlan connect`, or **§8 USB bridge**. |
| STA OK but Plugin/Ephys fails | `ping` + `Test-NetConnection -Port 5000`; use **Serial IP** not mDNS guess; send `REDPITAYA`/`START` (Plugin) or use USB bridge for Ephys Socket only |
| TCP refused | Same LAN, no **client isolation** (campus/guest), Windows firewall allow Python, no VPN |
| COM busy | Close Serial Monitor |
| Bridge no frames | `USB_OPEN_EPHYS_MODE true`, wait >5 s |
| Plugin no board | Set **Node IP** or `ESP32_NODE_HOST`; Plugin patches per `docs/open-ephys-plugin.md` |
| Campus Wi-Fi | Use phone hotspot or Soft AP |

---

## 11. Version pins

| Component | Pin |
|-----------|-----|
| ESP32-S3 | `4264e32+` on `main` |
| Firmware | `1.3.4` in sketch |
| Plugin | **`217425a`** on `main` |
| Rate / port | **100 Hz**, TCP **5000** |

---

## Bring-up order

**USB + Plugin (typical when Wi‑Fi fails):**

1. Flash `USB_OPEN_EPHYS_MODE true` → `serial_bench_reader.py COMx --binary --limit 10`  
2. `run_usb_plugin_bridge.ps1 COMx` → `esp32_tcp_client.py` with `ESP32_NODE_HOST=127.0.0.1`  
3. Build GUI + Plugin (`217425a`, `e298679`) → Acq Board @ **127.0.0.1:5000**, 100 Hz, 14 ch  

**Wi‑Fi + Plugin:** `esp32_tcp_client.py` against node IP, then same GUI with that IP.

Optional: OpenSim bridge; USB + Ephys Socket (bridge without `--plugin`).

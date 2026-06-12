# ESP32-S3 STEP Acquisition Nodes

Wireless **Seeed XIAO ESP32-S3** node for the STEP gait stack: **ICM-20948** IMU, **DIO**, **Open Ephys TCP** — one board is enough for v1 bench.

## Quick test modes

### (A) USB only — 4-wire ICM, no Wi-Fi

See **[4-wire ICM20948 + USB](docs/arduino-ide-guide.md#4-wire-icm20948--usb-to-pc)** in the guide. Preset:

```cpp
#define ENABLE_TCP false
#define ENABLE_SERIAL_BENCH true
#define ENABLE_ESPNOW false
#define ENABLE_SD false
```

Wiring: ICM **VCC→3V3**, **GND→GND**, **SDA→D4**, **SCL→D5**. Flash **XIAO_ESP32S3**, USB CDC enabled:

**Full diagram:** [docs/wiring-diagram.md](docs/wiring-diagram.md)

```powershell
python host/serial_bench_reader.py COM5
```

**Open Ephys + OpenSim (v1.4):** flash with `SERIAL_OUTPUT_BINARY true`, run `host/serial_tcp_bridge.py`, see [docs/esp32-fusion-and-opensim.md](docs/esp32-fusion-and-opensim.md).

### (B) Open Wi-Fi — TCP, no password

```cpp
#define ENABLE_TCP true
#define ENABLE_SERIAL_BENCH false
#define WIFI_SSID "MyOpenAP"
#define WIFI_PASS ""
```

Flash, note IP on Serial Monitor, then:

```powershell
set ESP32_NODE_HOST=<node-ip>
python host/esp32_tcp_client.py
```

Lab / isolated network only — open AP has no encryption.

### (C) WPA Wi-Fi + TCP (default sketch)

Set `WIFI_SSID` / `WIFI_PASS`, leave `ENABLE_TCP true`, run `host/esp32_tcp_client.py`.

Full steps: **[docs/arduino-ide-guide.md](docs/arduino-ide-guide.md)**

## Hardware

| Component | v1 target | Notes |
|-----------|-----------|-------|
| MCU | Seeed XIAO ESP32-S3 | One board for bench |
| IMU | ICM-20948 (I2C) | 100 Hz, ch0–5 |
| DIO | GPIO input | ch6 |
| Sync | ESP-NOW (optional) | Off by default |
| SD | Sense expansion (optional) | `ENABLE_SD` |
| Camera | **Deferred v2** | [camera-feasibility.md](docs/camera-feasibility.md) |

**Wiring diagram (Mermaid + ASCII):** [docs/wiring-diagram.md](docs/wiring-diagram.md)

**Open Ephys / Plugin repo:** [docs/open-ephys-plugin.md](docs/open-ephys-plugin.md)

**Full local setup (Windows):** [docs/local-open-ephys-setup.md](docs/local-open-ephys-setup.md)

## Red Pitaya parity

| Parameter | Value |
|-----------|-------|
| TCP port | 5000 |
| Handshake | `REDPITAYA` → `START` |
| Packet | 22-byte LE header + int16 channel-major |
| Rate | 100 Hz |
| Channels 0–5 | ax, ay, az, gx, gy, gz |
| Channel 6 | DIO |
| Channel 7 | Reserved (0 in v1) |

## Advanced: ESP-IDF

Same modules in **`firmware/`** — menuconfig **STEP_ENABLE_ESPNOW** defaults off for single-node.

## Host scripts

| Script | Use when |
|--------|----------|
| `host/serial_bench_reader.py COMx` | USB serial bench (no Wi-Fi) |
| `host/serial_tcp_bridge.py COMx` | USB → localhost:5000 for Open Ephys — requires **`USB_OPEN_EPHYS_MODE true`** on board |
| `host/esp32_tcp_client.py` | TCP test (Wi-Fi node or 127.0.0.1 via bridge) |

## Repository

https://github.com/Minkeejung0415/ESP32-S3.git

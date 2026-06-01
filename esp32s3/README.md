# ESP32-S3 IMU + Camera Acquisition Node

Drop-in companion/replacement for the Red Pitaya system.  Uses the **same
Open Ephys plugin and OpenSim bridge** — no host-side changes needed.

## Hardware

| Item | Part | Interface |
|------|------|-----------|
| MCU | Seeed Studio ESP32-S3 (16 MB flash, 8 MB PSRAM) | — |
| IMU | ICM-20948 (9-axis) | SPI @ 7 MHz |
| Camera (primary) | TEVM-AR0234 (AR0234 global-shutter 2 MP) | DVP 8-bit |
| Camera (alt) | Technexion TEVI-AR0234CS-S32-IR | MIPI CSI-2 → DVP bridge† |
| SD card | microSD ≥ 8 GB | SDMMC 4-bit |
| DIO | Any 3.3 V TTL source | GPIO4 (in) / GPIO5 (out) |

†  The Technexion module uses the same AR0234 sensor and identical I2C register
   map.  Only the physical bus differs (MIPI CSI-2).  A carrier board with a
   **TI DS90UB954** or **Toshiba TC358746** MIPI→DVP bridge makes the driver
   code fully transparent.  The S32 lens (fixed focus, ~90° HFOV) is mechanically
   compatible with the TEVI mounting pattern.

## Pin map (Seeed XIAO ESP32-S3 Sense)

See `main/include/config.h` for the full pinout.

## Multi-node wiring

```
Master ESP32-S3  ──WiFi──►  Open Ephys host (TCP 5000 / UDP 55001)
      │                  ──WiFi──►  OpenSim bridge (UDP 5005)
      │
      └──ESPNow──► Slave 1 ESP32-S3
      └──ESPNow──► Slave 2 ESP32-S3
      └──ESPNow──► … (up to 16 nodes)
```

Every slave carries its own IMU + camera.  The master collects all frames,
time-corrects them to its clock via ESPNow RTT, and forwards them over WiFi.

## Build & flash

```bash
# Install ESP-IDF v5.3+
. $IDF_PATH/export.sh

cd esp32s3
idf.py set-target esp32s3
idf.py menuconfig          # set WiFi SSID/password under "Component config"
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Node provisioning

Set role and node ID via NVS before first use:

```bash
# Master (node 0)
idf.py -p /dev/ttyUSB0 nvs-set config role u8 0
idf.py -p /dev/ttyUSB0 nvs-set config node_id u8 0

# Slave 1
idf.py -p /dev/ttyUSB1 nvs-set config role u8 1
idf.py -p /dev/ttyUSB1 nvs-set config node_id u8 1
```

## Protocol compatibility with Red Pitaya

| Feature | Red Pitaya | ESP32-S3 |
|---------|-----------|----------|
| TCP control port | 5000 | 5000 ✓ |
| UDP data port | 55001 | 55001 ✓ |
| Packet header format | 22-byte | 22-byte ✓ |
| Channel encoding | int16 Q15 | int16 Q15 ✓ |
| Quaternion UDP (OpenSim v2) | port 5000 | port 5005* |
| SD binary log | `.bin` | `.bin` ✓ |
| SD CSV log | `.csv` | `.csv` ✓ |

*Port changed from 5000 to 5005 to avoid collision with the TCP control socket
on the same device.  Update `UDP_QUAT_PORT` in `opensim_live_realtime.py` if
you switch from Red Pitaya to this node.

## Action verification

A JPEG frame is captured whenever the IMU dynamic acceleration exceeds
`VERIFY_ACCEL_THRESH_G` (default 0.5 g) for `VERIFY_HOLD_SAMPLES` consecutive
samples.  Frames are saved to SD as `session_<epoch>_frame_<seq>.jpg` with a
matching index file.  The sequence number links each frame to the exact IMU
sample row in the `.bin`/`.csv` files.

## Camera notes

### TEVM-AR0234
- Default output: MIPI CSI-2 serial.  The driver writes AR0234 register
  `0x301A` to select parallel (DVP) output at init time.
- XCLK must be stable before I2C configuration; the esp_camera component
  handles this.

### Technexion TEVI-AR0234CS-S32 (with S32 lens)
- Same AR0234 sensor, same I2C register map → driver code unchanged.
- Requires MIPI→DVP bridge on carrier board.
- S32 lens: fixed focus ~0.6 m, 90° HFOV, CS-mount.
- IR-cut and no-IR-cut variants available; select based on lighting
  conditions in the lab.
- **Feasibility**: Confirmed compatible at the sensor register level.
  Carrier board bridge adds ~1 frame latency (negligible for action verify).

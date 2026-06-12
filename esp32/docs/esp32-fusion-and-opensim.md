# ESP32 fusion, filter mode, and OpenSim (v1.4+)

Firmware **v1.4.0** runs **Madgwick AHRS on the ESP32** before samples are streamed.

## Channel map (11 channels)

| Ch | Content |
|----|---------|
| 0–2 | Gravity-removed linear accel (`FILTER_PERMANENT` default) |
| 3–5 | Raw gyro |
| 6 | DIO |
| 7–10 | Quaternion qw–qz (int16 ÷ 32767) |

## Open Ephys Plugin — no C++ rebuild (recommended)

```powershell
# 1) Add plugin-patches/hosts.txt line to C:\Windows\System32\drivers\etc\hosts
# 2) Gateway (serial → TCP :5000 + UDP :55001 + SENSORS line)
python host\rp_compat_gateway.py COM5
# 3) Open Ephys: connect to rp-f0f85a.local as usual
```

See [plugin-patches/MANUAL.md](../plugin-patches/MANUAL.md) and `scripts/patch_plugin_esp32.py` if you prefer to patch `acqboard.ccp` instead.

## OpenSim

```powershell
python host\rp_compat_gateway.py COM5
set ESP32_NODE_HOST=127.0.0.1
python host\esp32_to_opensim_bridge.py
```

Adjust `OPENSIM_UDP_PORT` to match your listener.

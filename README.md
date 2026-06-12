# Plugin

Open Ephys acquisition plugin with OpenSim live-display tooling and integrated
ESP32-S3 acquisition-node support.

## Layout

- `acqboard.ccp`, `Acqboardredpitaya.h`, `device editor.cpp` - Open Ephys
  acquisition board integration.
- `opensim_live_realtime.py`, `opensim_joint_catalog.py` - OpenSim live bridge
  and joint-angle HUD support.
- `esp32/` - ESP32-S3 firmware, Arduino sketch, host bridge/test tools, and
  hardware setup docs imported from `Minkeejung0415/ESP32-S3`.
- `docs/` - Plugin and OpenSim setup notes.
- `tests/` - Local C tests for stream framing and fusion behavior.

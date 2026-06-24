---
phase: 08-slave-monitor-sync-csv
milestone: v1.1
created: 2026-06-24
status: planned
---

# Phase 08 Context: Slave Monitor, Sync Evidence, and CSV Export

## Goal

Make slave ESP32 nodes operationally visible from the existing Open Ephys Acquisition Board plugin, prove that slave SD recordings are synchronized enough to trust, and standardize post-acquisition conversion from slave SD `.bin` files to CSV.

## Background

The active topology is:

- Master ESP32 starts the `STEP_ESP32` SoftAP at `192.168.4.1` and is the only board Open Ephys talks to.
- Slave ESP32 nodes join `STEP_ESP32`, receive master commands over ESP-NOW, and record to their own SD cards.
- Master has no SD card in the intended setup; relay-only `REC START`/`REC STOP` is expected.
- A slave SD `.bin` recording has been confirmed, and `esp32/host/sd_bin_to_csv.py` converts copied SD binaries to CSV.
- Open Ephys currently cannot tell whether slaves are alive, SD-ready, sensor-responsive, recording, saving samples, or synchronized.

## Decisions

- D-01: Keep one Open Ephys plugin. Do not create a separate slave plugin; add a compact Slave Monitor panel inside the existing Acquisition Board UI.
- D-02: The plugin talks only to the master. Slaves report to the master over ESP-NOW; the master exposes slave state to the plugin.
- D-03: Slave monitor updates target about 5 Hz for live operator feedback. This is for human inspection, not high-rate streaming.
- D-04: The UI must fit the existing plugin height. It uses a fixed-height compact panel: aggregate status line, slave dropdown, selected-slave values only.
- D-05: Slaves have quaternion/VQF enabled by default. The master remains filter/quaternion off by default unless explicitly configured.
- D-06: Slaves record binary `.bin` to SD during acquisition. CSV is generated after acquisition on the PC, not written live by the ESP32.
- D-07: DIO is the physical sync marker input on `D0` with pull-up behavior. Operators can test it by briefly pulling `D0` to GND during recording.
- D-08: Phase 08 adds sync evidence first: DIO level, edge count, edge timing summary, ESP-NOW sync flag, clock offset, and CSV analysis. Interrupt-timestamped DIO is allowed if needed, but a minimal auditable implementation is acceptable.
- D-09: Multi-slave command relay uses ESP-NOW broadcast. Multiple slaves require unique identity handling; static `192.168.4.2` for every slave is not acceptable long term.
- D-10: Slave status must include enough evidence to trust a run: online age, SD ready, recording state, saved sample count, SD errors, IMU/mag OK, quaternion enabled/responding, DIO state/edges, sync received, and clock offset.

## Key Files

| File | Role |
|------|------|
| `esp32/arduino/step_node/step_node.ino` | Master command relay and slave status aggregation |
| `esp32/arduino/step_node_slave/step_node_slave.ino` | Slave monitor packets, SD status, sensor preview, DIO evidence |
| `acqboard.ccp` | Plugin communication with master and parsing slave monitor status |
| `Acqboardredpitaya.h` | Board state fields for slave monitor data |
| `device editor.cpp` | Compact Acquisition Board UI panel/dropdown |
| `esp32/host/sd_bin_to_csv.py` | Direct `.bin` to `.csv` conversion |
| `esp32/host/analyze_sample_rate.py` | Sequence/timestamp analysis foundation |
| `docs/esp32-acqboard-integration.md` | Operator workflow documentation |

## Out of Scope

- Separate Open Ephys plugin for slaves.
- PC automatic download of slave SD files through the master.
- Live high-rate slave data streaming into Open Ephys from every slave.
- Full network time protocol or multi-hop mesh.
- Large dashboard UI that grows with number of slaves.

## Success Criteria

1. The master can report at least one live slave with age, SD state, recording state, saved samples, sensor status, quaternion status, DIO, and sync fields.
2. The plugin shows a compact fixed-height Slave Monitor panel with a dropdown for detected slaves.
3. Moving/rotating a selected slave visibly changes the live monitor values at about 5 Hz.
4. Slave binary SD recordings can be batch-converted to CSV after acquisition.
5. CSV summary reports sample count, duration, lost sequence count, quaternion nonzero check, DIO edge count, and basic sync evidence.
6. Multi-slave readiness is addressed by avoiding same-IP conflicts and exposing unique slave identity.

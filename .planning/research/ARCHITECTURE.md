# Architecture Research — OpenSim View Angle on Trigger

**Project:** Open Ephys Red Pitaya Plugin  
**Researched:** 2026-06-10  
**Confidence:** HIGH

## Component Diagram

```
┌─────────────────────┐     save/load      ┌──────────────────┐
│  DeviceEditor (UI)  │◄──────────────────►│ settings XML     │
│  - view ComboBox    │                    └──────────────────┘
└─────────┬───────────┘
          │ selected preset
          ▼
┌─────────────────────┐   ACQBOARD TRIGGER   ┌──────────────────┐
│  DeviceThread       │◄─────────────────────│ External/GUI TTL │
│  handleBroadcast    │                      └──────────────────┘
└─────────┬───────────┘
          │ write opensim_view_config.json
          ▼
┌─────────────────────┐   UDP :5000 quats   ┌──────────────────────────┐
│  AcqBoardRedPitaya │─────────────────────►│ opensim_live_realtime.py │
│  (existing stream)  │                     │  - IK loop               │
└─────────────────────┘                     │  - config watcher thread │
                                            │  - SimbodyVisualizer     │
                                            └──────────────────────────┘
```

## Data Flow — View Change Event

1. Operator selects preset in device editor (e.g. `"Lateral Right"`)
2. Selection stored in memory + XML attribute `OpenSimViewAngle`
3. On trigger (`ACQBOARD TRIGGER` handled or TTL fired), plugin writes:
   ```json
   { "view_id": "lateral_r", "label": "Lateral Right", "trigger_ts": 1234567890.1, "seq": 42 }
   ```
   to `WORK_DIR/opensim_view_config.json` (atomic rename)
4. Python watcher detects mtime/seq change, applies camera preset map, updates label state
5. Main render loop draws label adjacent to sim time each frame

## Preset Catalog (Proposed)

| ID | Label | Camera intent |
|----|-------|---------------|
| `default` | Default | Current Simbody default |
| `anterior` | Anterior | Face subject from front |
| `posterior` | Posterior | From behind |
| `lateral_r` | Lateral Right | Right sagittal |
| `lateral_l` | Lateral Left | Left sagittal |
| `superior` | Superior | From above |
| `isometric` | Isometric | 3/4 overview |

Exact Simbody transforms determined in Phase 2 spike.

## Build Order

1. Config schema + Python reader (can test standalone)
2. Plugin UI + XML persistence
3. Trigger → write config
4. Simbody camera apply + label render
5. Integration test with live UDP

## Integration Points (Existing Files)

| File | Change type |
|------|-------------|
| `device editor.cpp` / `.h` | UI ComboBox, save/load XML |
| `devicethread.cpp` | On TRIGGER, call view config writer |
| `AcqBoardRedPitaya.h/.cpp` or helper | `writeOpenSimViewConfig()` |
| `opensim_live_realtime.py` | Watcher thread, camera map, label |
| `docs/opensim-udp-v2.md` | Document sidecar (optional) |

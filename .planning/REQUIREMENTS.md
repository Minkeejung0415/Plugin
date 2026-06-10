# Requirements: Joint Angle Display on Trigger

**Defined:** 2026-06-10  
**Revised:** 2026-06-10 (scope correction — joint angles, not camera presets)  
**Core Value:** When a trigger fires during live acquisition, the operator sees only the pre-selected joint angles beside the sim timer.

## v1 Requirements

### Joint Selection (Plugin UI)

- [ ] **JOIN-01**: Operator can select which joint coordinates to display via multi-select controls in the Red Pitaya device editor
- [ ] **JOIN-02**: Selectable joints come from a curated coordinate catalog derived from the Rajagopal OpenSense model (e.g. `hip_flexion_r`, `knee_angle_r`, `ankle_angle_r`, left-leg equivalents, pelvis angles)
- [ ] **JOIN-03**: Selected joint list persists when device settings are saved and reloaded (XML)
- [ ] **JOIN-04**: UI can suggest or highlight joints adjacent to currently active IMU sensor segments (optional convenience, not required for v1 acceptance)

### Trigger Integration

- [ ] **TRIG-01**: When an `ACQBOARD TRIGGER` broadcast is handled, the plugin writes the currently selected joint list to the OpenSim work directory config file
- [ ] **TRIG-02**: Display config write uses atomic file replace and includes a monotonic sequence number to avoid stale reads
- [ ] **TRIG-03**: Display config write does not block or delay IMU UDP quaternion streaming

### OpenSim Live Display

- [ ] **DISP-01**: OpenSim Live reads the config file and displays **only** the selected joint coordinate values (not all model coordinates)
- [ ] **DISP-02**: Filtered joint angle values are rendered beside the Simbody simulation time/clock in the visualizer window
- [ ] **DISP-03**: Display filter changes take effect within 200 ms of the config file update during live streaming
- [ ] **DISP-04**: Live IK visualization continues without regression when the display filter changes
- [ ] **DISP-05**: When no joints are selected (or config absent), display shows no joint readout (or a minimal "none" state) — not the full coordinate dump

### Operator Verification

- [ ] **OPS-01**: Operator can manually apply the current joint selection without a trigger (test/preview control in device editor)
- [ ] **OPS-02**: Documentation describes joint catalog, trigger workflow, config file location, and display format

## v2 Requirements

### Advanced Trigger Mapping

- **TRIG-10**: Different TTL trigger lines map to different joint display sets
- **TRIG-11**: Trigger applies a saved preset profile (named joint sets)

### Display Enhancements

- **DISP-10**: On-screen overlay with semi-transparent background for readability
- **DISP-11**: User-editable display labels per coordinate
- **DISP-12**: Auto-select joints from active sensor count / `opensim_sensor_map.json`

## Out of Scope

| Feature | Reason |
|---------|--------|
| Camera/view angle presets | User correction — not the requested feature |
| Simbody camera API work | Superseded by joint-angle HUD |
| Gen Motion offline pipeline | Scope is OpenSim Live Simbody window only |
| UDP v2 packet format changes | Risk to proven live stream; use JSON sidecar |
| Full model coordinate picker (all 80+ DOFs) | v1 uses curated catalog; expand in v2 if needed |

## Traceability

| Requirement | Phase | Status |
|-------------|-------|--------|
| JOIN-01 | Phase 2 | Pending |
| JOIN-02 | Phase 2 | Pending |
| JOIN-03 | Phase 2 | Pending |
| JOIN-04 | Phase 2 | Pending |
| TRIG-01 | Phase 3 | Pending |
| TRIG-02 | Phase 3 | Pending |
| TRIG-03 | Phase 3 | Pending |
| DISP-01 | Phase 4 | Pending |
| DISP-02 | Phase 4 | Pending |
| DISP-03 | Phase 1 | Pending |
| DISP-04 | Phase 5 | Pending |
| DISP-05 | Phase 4 | Pending |
| OPS-01 | Phase 3 | Pending |
| OPS-02 | Phase 5 | Pending |

**Coverage:**
- v1 requirements: 14 total
- Mapped to phases: 14
- Unmapped: 0 ✓

---
*Requirements defined: 2026-06-10*  
*Last updated: 2026-06-10 — rewritten for joint angle display control*

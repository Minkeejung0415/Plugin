# Requirements: OpenSim View Angle on Trigger

**Defined:** 2026-06-10  
**Core Value:** When a trigger fires during live acquisition, the operator sees the skeleton from the pre-selected view with the view name beside the sim timer.

## v1 Requirements

### View Selection (Plugin UI)

- [ ] **VIEW-01**: Operator can select a camera view preset from a dropdown in the Red Pitaya device editor
- [ ] **VIEW-02**: Available presets include Default, Anterior, Posterior, Lateral Right, Lateral Left, Superior, and Isometric
- [ ] **VIEW-03**: Selected preset persists when device settings are saved and reloaded (XML)

### Trigger Integration

- [ ] **TRIG-01**: When an `ACQBOARD TRIGGER` broadcast is handled, the plugin writes the currently selected view preset to the OpenSim work directory config file
- [ ] **TRIG-02**: View config write uses atomic file replace and includes a monotonic sequence number to avoid stale reads
- [ ] **TRIG-03**: View config write does not block or delay IMU UDP quaternion streaming

### OpenSim Live Display

- [ ] **DISP-01**: OpenSim Live applies the camera transform matching the preset ID received from the config file
- [ ] **DISP-02**: The human-readable preset label (e.g. "Lateral Right") is displayed beside the Simbody simulation time/clock in the visualizer window
- [ ] **DISP-03**: View changes take effect within 200 ms of the config file update during live streaming
- [ ] **DISP-04**: Live IK visualization continues without regression when view presets change

### Operator Verification

- [ ] **OPS-01**: Operator can manually apply the current preset without a trigger (test/preview control in device editor)
- [ ] **OPS-02**: Documentation describes preset list, trigger workflow, and config file location

## v2 Requirements

### Advanced Trigger Mapping

- **TRIG-10**: Different TTL trigger lines map to different view presets
- **TRIG-11**: Animated camera transition over configurable duration

### Display Enhancements

- **DISP-10**: On-screen overlay with semi-transparent background for label readability
- **DISP-11**: User-editable preset labels

## Out of Scope

| Feature | Reason |
|---------|--------|
| Joint coordinate angle display | User request is camera/view angle beside clock, not DOF readout |
| Gen Motion offline pipeline | Scope is OpenSim Live Simbody window only |
| Custom arbitrary camera drag-from-plugin | Simbody mouse controls remain manual fallback |
| Non-Windows platforms | Repo hardcodes Windows OpenSim paths today |
| UDP v2 packet format changes | Risk to proven live stream; use JSON sidecar |

## Traceability

| Requirement | Phase | Status |
|-------------|-------|--------|
| VIEW-01 | Phase 2 | Pending |
| VIEW-02 | Phase 2 | Pending |
| VIEW-03 | Phase 2 | Pending |
| TRIG-01 | Phase 3 | Pending |
| TRIG-02 | Phase 3 | Pending |
| TRIG-03 | Phase 3 | Pending |
| DISP-01 | Phase 1 | Pending |
| DISP-02 | Phase 4 | Pending |
| DISP-03 | Phase 1 | Pending |
| DISP-04 | Phase 5 | Pending |
| OPS-01 | Phase 3 | Pending |
| OPS-02 | Phase 5 | Pending |

**Coverage:**
- v1 requirements: 12 total
- Mapped to phases: 12
- Unmapped: 0 ✓

---
*Requirements defined: 2026-06-10*  
*Last updated: 2026-06-10 after roadmap creation*

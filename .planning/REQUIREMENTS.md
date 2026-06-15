# Requirements: SD Card Reliability and Lossless Acquisition

**Defined:** 2026-06-12
**Milestone:** v1.1
**Core Value:** Save acquisition data on SD card with verifiable sample continuity, then quantify and document the maximum reliable frequency under realistic filter and streaming loads.

## v1.1 Requirements

### Device-Side SD Recording

- [x] **SD-01**: Firmware mounts/detects SD card at startup and reports whether SD logging is enabled, disabled, or failed.
- [x] **SD-02**: Firmware writes acquisition samples to SD without blocking the acquisition loop on per-sample card latency.
- [x] **SD-03**: SD file format records `seq`, `timestamp_us`, channel data, and enough session metadata to decode sample rate, channel layout, firmware mode, and logging mode.
- [x] **SD-04**: Firmware records generated sample count, saved sample count, SD queue drops, max SD queue depth, max write latency, and acquisition-loop overrun count.
- [x] **SD-05**: SD recording can be explicitly enabled/disabled for stress comparisons without changing unrelated firmware behavior.

### Sample-Loss Accounting

- [x] **LOSS-01**: Every generated sample has a monotonic sequence number that is persisted to SD.
- [ ] **LOSS-02**: Analyzer reports duplicate sequences, missing sequences, timestamp gaps, and duration/rate mismatch from SD files.
- [x] **LOSS-03**: Stress output compares generated vs saved vs streamed counts when the firmware exposes those counters.
- [x] **LOSS-04**: "No samples lost" is accepted only when SD sequence continuity is clean and firmware counters agree.

### Stress Testing

- [ ] **STRESS-01**: `esp32/host/stress_test_serial.py` can sweep requested frequency across configurable ranges and durations.
- [ ] **STRESS-02**: Stress sweep supports mode combinations: filter on/off, SD on/off, Open Ephys/serial streaming on/off when firmware commands support them.
- [ ] **STRESS-03**: Stress summary reports highest passing frequency and recommended operating cap.
- [ ] **STRESS-04**: Stress artifacts are saved per run in machine-readable CSV/JSON/Markdown so results can be compared across firmware changes.
- [ ] **STRESS-05**: Analyzer includes latency and stall metrics, not only sequence gaps.

### Stall Isolation

- [x] **STALL-01**: Firmware measures acquisition-loop duration and flags loop overruns relative to requested period.
- [x] **STALL-02**: Firmware measures SD write latency separately from sensor-read/filter latency.
- [x] **STALL-03**: Test workflow distinguishes hardware/sensor/I2C limits from filter CPU cost, SD-card stalls, USB/TCP/Open Ephys buffering, and UDP/host transport loss.
- [ ] **STALL-04**: Open Ephys buffer stalls are tested as a downstream symptom and not treated as proof that acquisition samples were lost on device.

### Operator Verification

- [x] **OPS-01**: Documentation explains how to run SD-first acquisition reliability tests.
- [ ] **OPS-02**: Documentation defines the pass/fail threshold for a safe operating frequency.
- [ ] **OPS-03**: Documentation includes a minimal field checklist: format SD, run baseline, run filter+SD stress, verify SD continuity, then enable Open Ephys streaming.

## Deferred Requirements

### OpenSim / Visualization

- **HUD-10**: OpenSim HUD/window-title/live-angle display follow-up from v1.0.
- **HUD-11**: Any new IK or joint-display behavior.

### Future Acquisition Enhancements

- **ACQ-10**: Multi-board SD log synchronization beyond existing sequence/timestamp fields.
- **ACQ-11**: Binary-to-host replay from SD into Open Ephys.
- **ACQ-12**: Automated SD-card benchmark suite across multiple card models.

## Out of Scope

| Feature | Reason |
|---------|--------|
| UDP as data-integrity proof | UDP can drop packets and should only be used as a visualization/transport signal |
| OpenSim HUD fixes | Explicitly paused while acquisition fundamentals are addressed |
| UI polish unrelated to recording integrity | v1.1 needs measurement and loss accounting first |
| Claiming lossless streaming through Open Ephys alone | Ground truth must come from device-side SD logs |

## Traceability

| Requirement | Phase | Status |
|-------------|-------|--------|
| SD-01 | Phase 1, Phase 2 | Logger status implemented; full SD mount pin configuration pending hardware confirmation |
| SD-02 | Phase 2 | Buffered writer implemented; firmware build/hardware verification pending |
| SD-03 | Phase 1, Phase 2 | Header plus `oe_sample_t` sample records implemented; analyzer pending Phase 3 |
| SD-04 | Phase 1, Phase 2 | Firmware counters implemented; firmware build/hardware verification pending |
| SD-05 | Phase 1, Phase 2, Phase 3 | Plugin-triggered RECORD ON/OFF implemented; stress mode tooling pending Phase 3 |
| LOSS-01 | Phase 1, Phase 2 | Monotonic seq persisted when SD recording is active; firmware build/hardware verification pending |
| LOSS-02 | Phase 3 | Planned |
| LOSS-03 | Phase 3 | Planned |
| LOSS-04 | Phase 5 | Planned |
| STRESS-01 | Phase 3 | Planned |
| STRESS-02 | Phase 3 | Planned |
| STRESS-03 | Phase 3, Phase 5 | Planned |
| STRESS-04 | Phase 3 | Planned |
| STRESS-05 | Phase 1, Phase 3 | Metric contract defined; analyzer implementation pending Phase 3 |
| STALL-01 | Phase 1, Phase 2 | Acquisition-loop overrun counter implemented; firmware build/hardware verification pending |
| STALL-02 | Phase 1, Phase 2 | SD enqueue/write latency counters implemented; firmware build/hardware verification pending |
| STALL-03 | Phase 4 | Planned |
| STALL-04 | Phase 4 | Planned |
| OPS-01 | Phase 5 | Planned |
| OPS-02 | Phase 5 | Planned |
| OPS-03 | Phase 5 | Planned |

**Coverage:**
- v1.1 requirements: 21 total
- Mapped to phases: 21
- Unmapped: 0

---
*Requirements defined: 2026-06-12 - SD card reliability and lossless acquisition*

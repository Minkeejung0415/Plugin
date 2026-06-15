# Roadmap: SD Card Reliability and Lossless Acquisition

**Project:** Open Ephys ESP32 Acquisition Reliability
**Milestone:** v1.1
**Created:** 2026-06-12
**Phases:** 5
**Requirement coverage:** 21/21 v1.1 requirements mapped

## Overview

| # | Phase | Goal | Requirements | Status |
|---|-------|------|--------------|--------|
| 1 | Measurement Contract & Baseline | Define file/counter contract and measure today's failure modes | SD-01, SD-03, SD-04, SD-05, LOSS-01, STALL-01, STALL-02 | Complete 2026-06-12 |
| 2 | Buffered SD Logger | Make SD recording non-blocking and measurable | SD-01-05, LOSS-01, STALL-01, STALL-02 | Planned |
| 3 | Stress Harness & Analyzer | Sweep rate/modes and analyze SD/stream continuity | LOSS-02-03, STRESS-01-05 | Planned |
| 4 | Open Ephys Stall Isolation | Separate device loss from host/Open Ephys/transport buffering | STALL-03, STALL-04 | Planned |
| 5 | Hardware UAT & Operator Docs | Prove no-loss operating envelope and document checklist | LOSS-04, STRESS-03, OPS-01-03 | Planned |

---

## Phase Checklist

- [x] **Phase 1: Measurement Contract & Baseline** (completed 2026-06-12)
- [ ] **Phase 2: Buffered SD Logger**
- [ ] **Phase 3: Stress Harness & Analyzer**
- [ ] **Phase 4: Open Ephys Stall Isolation**
- [ ] **Phase 5: Hardware UAT & Operator Docs**

---

## Phase Details

### Phase 1: Measurement Contract & Baseline

**Goal:** Define the sample-log/counter contract and run baseline diagnostics against the current synchronous SD logger and latest-sample stream path.

**Success Criteria:**
1. Document exact SD sample record format, session metadata, and firmware counters.
2. Add or identify commands needed to toggle filter, SD logging, and streaming for stress comparisons.
3. Capture current behavior: acquisition loop latency, SD write behavior, serial/Open Ephys visible gaps, and whether `s_latest` overwrites intermediate samples.
4. Produce a short baseline report that states which failure layers are already evident from code and logs.

**Key files:** `esp32/firmware/main/main.c`, `sd_logger.c`, `sd_logger.h`, `open_ephys_stream.c`, `open_ephys_stream.h`, `esp32/host/stress_test_serial.py`

---

### Phase 2: Buffered SD Logger

**Goal:** Replace per-sample synchronous SD writes with a buffered writer task that preserves acquisition-loop timing and exposes health counters.

**Success Criteria:**
1. Acquisition loop enqueues samples or fixed-size blocks without waiting on card flush latency.
2. SD writer batches writes and flushes on controlled intervals/session close rather than every sample.
3. Queue overflow, write failure, max queue depth, max write latency, generated count, saved count, and overrun count are tracked.
4. Startup and runtime logs make SD status unambiguous.

**Key files:** `esp32/firmware/main/sd_logger.c`, `sd_logger.h`, `main.c`, `Kconfig.projbuild`

---

### Phase 3: Stress Harness & Analyzer

**Goal:** Upgrade host stress tests so they can determine the maximum safe frequency with filter and SD modes included.

**Success Criteria:**
1. `stress_test_serial.py` runs sweeps over frequency and mode combinations.
2. Test artifacts include per-rate CSV/JSON/Markdown summaries.
3. Analyzer can validate SD files and host-visible streams using the same sequence/timestamp rules.
4. Summary reports highest passing rate and recommended cap with a clear reason for first failure.

**Key files:** `esp32/host/stress_test_serial.py`, `esp32/host/analyze_sample_rate.py`, `esp32/docs/stress-test-sample-rate.md`

---

### Phase 4: Open Ephys Stall Isolation

**Goal:** Determine whether Open Ephys stalls are caused by device acquisition, SD writes, serial/TCP transport, plugin buffering, or host-side processing.

**Success Criteria:**
1. Test matrix compares SD-only, stream-only, SD+stream, filter-off, and filter-on runs.
2. Open Ephys/export results are compared against SD ground truth.
3. If Open Ephys stalls while SD continuity is clean, classify the issue as downstream buffering/transport rather than acquisition loss.
4. If SD continuity fails, classify the bottleneck with firmware counters and mode comparisons.

**Key files:** `acqboard.ccp`, `devicethread.cpp`, `esp32/host/serial_tcp_bridge.py`, `esp32/host/stress_test_serial.py`, SD artifacts from Phase 3

---

### Phase 5: Hardware UAT & Operator Docs

**Goal:** Produce the no-loss operating envelope and the repeatable checklist for real sessions.

**Success Criteria:**
1. Hardware run proves SD sequence continuity at the chosen operating frequency with filter and required channels enabled.
2. Recommended frequency cap is documented from the stress results.
3. Operator checklist covers SD preparation, stress preflight, acquisition run, SD continuity verification, and Open Ephys comparison.
4. Residual risks are explicit, including SD-card model dependence and host streaming limitations.

**Key files:** `esp32/docs/stress-test-sample-rate.md`, `docs/esp32-acqboard-integration.md`, generated stress results

---

## Phase Ordering Rationale

1. Define measurements before changing behavior so baseline failures are not hidden.
2. Make SD logging safe before declaring it ground truth.
3. Expand stress tests once firmware exposes the counters and modes they need.
4. Isolate Open Ephys after SD ground truth exists.
5. Finish with hardware UAT and operator docs because acceptance depends on real card/device behavior.

## Dependencies

```text
Phase 1 -> Phase 2 -> Phase 3 -> Phase 4 -> Phase 5
```

## Superseded Planning

The prior v1.0 roadmap for OpenSim joint-angle display and HUD live-update remediation is historical. Do not continue HUD work inside this milestone unless acquisition reliability is complete and explicitly resumed.

### Phase 6: Plugin-controlled ESP32 SD recording with reconnect-safe local retrieval

**Goal:** [To be planned]
**Requirements**: TBD
**Depends on:** Phase 5
**Plans:** 0/3 plans executed

Plans:
- [ ] TBD (run /gsd-plan-phase 6 to break down)

---
*Roadmap created: 2026-06-12 - v1.1 SD card reliability and lossless acquisition*

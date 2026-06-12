# Phase 1: Measurement Contract & Baseline - Context

**Gathered:** 2026-06-12
**Status:** Ready for planning

<domain>
## Phase Boundary

Phase 1 defines the SD-card sample-log contract, firmware health counters, mode switches, and baseline diagnosis for the current acquisition path. It should document what counts as device-side ground truth and identify today's obvious risk layers before Phase 2 changes the logger implementation.

This phase may add lightweight documentation or small diagnostics if needed, but it should not attempt the full buffered SD logger implementation, the full stress-harness rewrite, or hardware UAT.

</domain>

<decisions>
## Implementation Decisions

### Ground-Truth Recording Contract
- SD-card data is the authoritative acquisition record for v1.1.
- Every saved record must include monotonic `seq`, `timestamp_us`, channel payload, and session metadata sufficient to decode sample rate, channel layout, firmware mode, and logging mode.
- Firmware counters must include at minimum generated samples, saved samples, dropped/overflowed samples, acquisition-loop overruns, max queue depth, and max write latency once the buffered logger exists.
- Open Ephys, serial/TCP, and UDP paths are comparison sinks only. They can expose downstream stalls, but they cannot prove device-side sample integrity.

### Test-Mode Matrix
- Phase 1 should identify or specify commands/toggles for frequency, SD logging on/off, filter on/off, and streaming on/off.
- SD-only, stream-only, SD+stream, filter-off, and filter-on runs are all first-class isolation modes.
- If a toggle does not exist yet, Phase 1 should record it as a required firmware command/interface for Phase 2 or Phase 3.
- Stress results must be interpretable by layer: sensor/I2C acquisition, filter CPU, SD writes, USB/TCP/Open Ephys buffering, and UDP/host transport.

### Baseline Evidence
- Phase 1 can rely on code inspection plus small local diagnostics; it does not block on a full real-hardware stress sweep.
- The baseline report should explicitly call out current code risks: synchronous per-sample SD flush, latest-sample-only Open Ephys stream handoff, and host-visible stress tests lacking SD ground-truth comparison.
- Hardware runs belong later unless they are quick sanity checks available during the phase.
- The output should make the next implementation step obvious: buffered/measured SD writer first, then stress tooling that compares SD and stream outputs.

### Artifacts
- Create a phase-local baseline/contract document under `.planning/phases/01-measurement-contract-baseline/`.
- Keep operator-facing ESP32 docs stable during Phase 1 unless Phase 1 uncovers a change operators must know immediately.
- Phase 2 and Phase 3 should consume the Phase 1 contract rather than rediscovering required counters and modes.

### the agent's Discretion
The agent may choose the exact document name and section structure for the baseline/contract artifact. It may add small code comments or no-op diagnostic declarations if they clarify the contract, but should avoid broad firmware changes in Phase 1.

</decisions>

<code_context>
## Existing Code Insights

### Reusable Assets
- `esp32/firmware/main/open_ephys_stream.h` defines `oe_sample_t` with `timestamp_us`, `seq`, and 8 int16 channels.
- `esp32/host/analyze_sample_rate.py` already detects consecutive duplicate sequences, identical rows, sequence gaps, and timestamp gaps.
- `esp32/host/stress_test_serial.py` already sweeps requested frequency and writes per-rate CSV plus a Markdown summary.
- Official ESP-IDF storage docs provide the expected SD/FatFS direction: SDMMC/SDSPI mount through VFS/FatFS and examples under `storage/sd_card/*`; real applications should use proper mount/probe/error handling.

### Established Patterns
- The firmware acquisition loop currently creates one `oe_sample_t` per period in `esp32/firmware/main/main.c`.
- The loop uses `OE_STREAM_SAMPLE_HZ` and `vTaskDelay(period)`, so loop-overrun measurement must compare elapsed acquisition work against the requested period.
- Existing host tools prefer simple CSV/Markdown artifacts in `esp32/host/stress_results`.
- GSD phase outputs should keep diagnostics and contracts in `.planning/phases/<phase>/` until implementation stabilizes.

### Integration Points
- `sd_logger_append()` currently performs `fwrite()` and `fflush()` per sample, directly coupling card latency to acquisition timing.
- `open_ephys_stream_set_sample()` stores only `s_latest`; if streaming lags, intermediate samples can be overwritten before the stream task sends them.
- `stream_task()` sends one current sample per stream period and should be treated as a live transport path, not a lossless queue.
- Future firmware mode toggles likely belong in the serial/control command surface used by `stress_test_serial.py` or in Kconfig defaults if runtime commands do not exist yet.

</code_context>

<specifics>
## Specific Ideas

- Treat SD as the proof source and Open Ephys as a downstream comparison path.
- Preserve sequence continuity as the central acceptance mechanism.
- Make the current synchronous `fflush()` behavior visible as a suspected stall source.
- Make the latest-sample stream handoff visible as an intentional non-lossless behavior unless redesigned later.

</specifics>

<deferred>
## Deferred Ideas

- Buffered SD writer implementation belongs in Phase 2.
- Stress harness mode sweep and SD-file analyzer support belong in Phase 3.
- Open Ephys export comparison and stall classification belong in Phase 4.
- Hardware UAT and operator checklist belong in Phase 5.

</deferred>

---
phase: 6
reviewers: [codex]
reviewed_at: 2026-06-15T16:54:55-07:00
plans_reviewed:
  - 06-01-PLAN.md
  - 06-02-PLAN.md
  - 06-03-PLAN.md
---

# Cross-AI Plan Review - Phase 6

## Codex Review

## Summary

The three-plan sequence is directionally sound: it starts with a shared protocol contract, then implements firmware/bridge behavior, then wires plugin UX and end-to-end validation. The plans correctly preserve the core milestone principle that SD is the ground truth and host/plugin transport is control plus retrieval, not the lossless acquisition path. Main risks are under-specification of the retrieval protocol, ambiguity around ESP-IDF vs Arduino firmware ownership, and potential blocking/threading issues in the Open Ephys plugin during stop/finalize/retrieve.

## 06-01: Recording Protocol and Status Contract

### Strengths

- Puts protocol/state design before implementation, which is necessary because firmware, bridge, and plugin all need the same vocabulary.
- Explicitly prevents premature "saved" status before firmware-confirmed finalization and retrieval.
- Includes reconnect semantics and retry behavior, which are essential for this phase.
- Correctly treats direct TCP and USB bridge as first-class targets.

### Concerns

- **HIGH:** Retrieval framing is not specified deeply enough. "Checksum or equivalent" and "file chunk retrieval" need concrete framing rules, chunk IDs, length fields, escaping/binary safety, EOF behavior, and resume/retry semantics.
- **HIGH:** The plan does not explicitly define how command/status traffic is separated from live Open Ephys binary sample traffic. This is a major integration risk.
- **MEDIUM:** The lifecycle states include `retrieved`, but local retrieval path is not firmware state. The contract should distinguish firmware session state from plugin/local export state.
- **MEDIUM:** It does not define versioning/capability negotiation. Old firmware, Arduino firmware, and ESP-IDF firmware may expose different command sets.
- **MEDIUM:** Security/integrity details are thin. At minimum, file path handling should avoid arbitrary path reads and only allow retrieval of known finalized session files.
- **LOW:** OPS-01 is listed, but this plan only creates phase-local protocol docs. Operator documentation probably belongs mainly in 06-03.

### Suggestions

- Define a concrete protocol table with request, response, fields, error codes, timeout behavior, and retry behavior.
- Add a protocol version/capability command, for example `REC CAPABILITIES` or `STATUS` field `proto=rec-v1`.
- Split states into:
  - Firmware recording state: `idle`, `recording`, `grace`, `finalizing`, `finalized`, `failed`.
  - Transfer state: `none`, `metadata`, `chunking`, `complete`, `aborted`, `failed`.
  - Plugin local state: `retrieving`, `retrieved`, `verify_pass`, `verify_fail`.
- Require file metadata to include session id, SD path, byte size, sample count, header/session metadata, finalization reason, and checksum.
- Explicitly forbid arbitrary filename reads; retrieval should use session id or latest finalized recording.

### Risk Assessment

**MEDIUM.** The plan has the right sequencing and scope, but if the protocol remains abstract, the later implementation can easily diverge across plugin, bridge, and firmware.

## 06-02: Firmware and Bridge Recording Control

### Strengths

- Targets the key firmware behavior: SD logging continues through temporary host disconnects.
- Includes timeout finalization and preserved status for the next connection.
- Correctly includes bridge relay updates, especially adding `RECORD` and related commands.
- Calls out transfer framing separation from live Open Ephys frames.
- Verification covers reconnect-before-timeout, timeout finalization, bridge relay, and retrieval.

### Concerns

- **HIGH:** "Relevant firmware path" is ambiguous. The project has ESP-IDF firmware and Arduino `step_node.ino`; both are referenced. The plan needs to say whether both must be updated in this phase or one is canonical with the other compatibility-only.
- **HIGH:** File retrieval over serial may be very slow and may interfere with live streaming if not explicitly paused or isolated. The plan should define expected behavior during retrieval.
- **HIGH:** Disconnect detection may differ between TCP socket mode and serial bridge mode. Killing the plugin, killing the bridge, unplugging USB, and TCP client disconnect are different failure modes.
- **MEDIUM:** The plan does not explicitly require acquisition-loop non-blocking behavior during finalization/retrieval. Retrieval should happen only after recording stops, but status queries during recording must not block the acquisition loop.
- **MEDIUM:** It does not specify SD file lifetime after timeout finalization. The firmware should retain metadata and allow later retrieval without overwriting the file on reconnect.
- **MEDIUM:** It lacks explicit error handling for SD full, SD removed, file open failure, partial write, checksum failure, abort during transfer, and reconnect during transfer.
- **LOW:** `git diff --check` is useful but insufficient. Firmware build checks should be named if the environment supports them.

### Suggestions

- Make firmware targets explicit:
  - ESP-IDF direct TCP path: `open_ephys_stream.c`, `sd_logger.c/h`.
  - Arduino USB bridge path: `step_node.ino`, if still required by operator workflow.
  - Bridge relay: `serial_tcp_bridge.py`.
- Define disconnect sources separately: direct TCP client disconnect, bridge disconnect from plugin, serial device loss, and firmware reboot.
- Require status counters to persist after finalization until explicitly cleared or a new session starts.
- Add file transfer failure modes and expected responses: `not_finalized`, `busy_recording`, `not_found`, `offset_out_of_range`, `checksum_mismatch`, `sd_error`, `aborted`.
- Require retrieval to be read-only against the finalized SD file and never delete/truncate the SD source after transfer.
- Add build verification where practical: ESP-IDF build and at least syntax/static validation for Python bridge changes.

### Risk Assessment

**HIGH.** This is the hardest plan. It crosses firmware timing, transport protocol, bridge relay, and file transfer. The plan is achievable, but ambiguity around firmware variants and disconnect semantics could cause partial implementation that works in one mode only.

## 06-03: Plugin UI, Local Retrieval, and End-to-End Verification

### Strengths

- Correctly changes plugin behavior from optimistic local status to firmware-confirmed status.
- Includes removal or suppression of the legacy ESP32 PC-side CSV path for normal recording.
- Adds retry behavior for retrieval after finalization, which is important for field use.
- Verification is aligned with the phase goal: no-SD, SD smoke, analyzer pass, reconnect, timeout, responsiveness.
- Calls out Open Ephys responsiveness during finalize/retrieval.

### Concerns

- **HIGH:** The plan does not explicitly require retrieval/finalization to run off the audio/acquisition thread and off the UI message thread. This is critical for Open Ephys responsiveness.
- **HIGH:** "Invoking or documenting" the analyzer is too weak for LOSS-02/LOSS-04. If the phase goal includes verified local retrieval, the plugin or workflow should produce a clear analyzer result artifact, even if manual invocation remains supported.
- **MEDIUM:** It does not specify where local files are saved, naming conventions, collision handling, or metadata sidecar output.
- **MEDIUM:** It does not define UX for reconnect-timeout finalized sessions discovered after reconnect. The plugin should offer retrieval of the preserved session, not just show status.
- **MEDIUM:** It does not mention progress reporting or cancellation for long transfers.
- **MEDIUM:** It should preserve user-facing distinction between "SD finalized" and "local copy verified." Those are different success levels.
- **LOW:** It does not explicitly require compatibility behavior when firmware lacks the new protocol. The plugin should fail clearly instead of falling back to misleading legacy recording.

### Suggestions

- Add a task requiring asynchronous finalize/retrieve implementation with clear thread ownership.
- Define local output structure, for example timestamped/session-id directory containing `.bin`, metadata JSON, transfer log, and analyzer summary.
- Make analyzer integration stronger: either run it automatically after retrieval or write a documented command plus machine-readable handoff file. Prefer automatic if practical.
- Add progress/cancel/retry UI states for retrieval.
- Add explicit behavior for old firmware: show "recording protocol unsupported" and do not claim SD recording.
- Add verification for plugin restart/reconnect after timeout-finalized recording: plugin discovers finalized session and retrieves it.

### Risk Assessment

**MEDIUM-HIGH.** The plan is well scoped but touches Open Ephys plugin threading and operator-facing truthfulness. The biggest risk is accidentally blocking critical threads or reporting success before local file integrity is verified.

## Cross-Plan Concerns

- **HIGH:** The transfer protocol needs concrete design before 06-02 starts. Without exact framing, error codes, checksums, offsets, and state rules, firmware/plugin implementations may be incompatible.
- **HIGH:** Firmware target ambiguity could split effort. Decide whether ESP-IDF and Arduino are both required deliverables for Phase 6.
- **HIGH:** The plans should explicitly test "recording continues while host is gone" using SD continuity, not just status counters.
- **MEDIUM:** Retrieval performance may be poor over 115200 serial. The plan should set expectations, support progress, and avoid blocking normal acquisition workflows.
- **MEDIUM:** The phase could drift into building a full file-transfer subsystem. Keep it minimal: finalized session metadata, sequential chunk read, checksum, retry from offset, abort.
- **MEDIUM:** Documentation should define pass/fail language carefully:
  - "SD finalized"
  - "Local retrieval complete"
  - "Transfer checksum passed"
  - "Analyzer continuity passed"
  These should not be collapsed into one "saved" state.

## Overall Risk Assessment

**Overall Risk: MEDIUM-HIGH.**

The plan set is coherent and addresses the phase goals, but implementation risk is real because this phase crosses real-time firmware, SD logging, serial/TCP bridge behavior, binary transfer, plugin UI, and Open Ephys threading. The most important improvements are to concretize the protocol in 06-01, explicitly define firmware targets in 06-02, and require asynchronous plugin transfer plus analyzer-backed verification in 06-03.

---

## Consensus Summary

This review used one selected external reviewer (`codex`). The synthesis below treats repeated themes across that review's plan-specific and cross-plan sections as the priority concerns to feed back into planning.

### Agreed Strengths

- The wave ordering is sensible: protocol contract first, firmware/bridge implementation second, plugin/retrieval UX third.
- The plans preserve SD as the acquisition ground truth and avoid treating host/plugin transport as the lossless recording path.
- The plans recognize reconnect-before-timeout, timeout finalization, retrieval retry, and truthful status as core operator requirements.
- The verification list already includes no-SD, SD smoke, reconnect, timeout, retrieval, analyzer, and responsiveness checks.

### Agreed Concerns

- **HIGH:** The transfer protocol must be concrete before implementation starts: framing, chunk numbering, byte lengths, EOF, checksums, offsets, retry, abort, and error codes.
- **HIGH:** Command/status/file-transfer traffic must be separated safely from live Open Ephys binary sample traffic.
- **HIGH:** Phase 6 must decide whether ESP-IDF direct TCP and Arduino USB bridge firmware are both required deliverables, or whether one is canonical.
- **HIGH:** Disconnect behavior must be tested across the actual failure modes: direct TCP client disconnect, plugin-to-bridge disconnect, bridge process loss, USB/serial device loss, and firmware reboot.
- **HIGH:** Retrieval/finalization must not block the Open Ephys audio/acquisition path or UI message thread.
- **HIGH:** The retrieved local file needs a clear verification artifact, preferably analyzer-backed, before the workflow reports full success.

### Divergent Views

- No reviewer disagreement was observed because only the Codex reviewer was invoked for this run.

### Current Unresolved HIGH Concerns

- Retrieval framing is too abstract and needs concrete chunking, checksum, EOF, resume/retry, abort, and error-code rules.
- Command/status/file-transfer traffic is not yet explicitly separated from live Open Ephys binary sample traffic.
- Firmware target ownership is ambiguous between ESP-IDF direct TCP and Arduino USB bridge paths.
- Serial retrieval performance and isolation are not yet specified, including whether live streaming pauses or uses a separate path.
- Disconnect detection semantics differ across direct TCP, bridge, plugin, USB/serial, and firmware reboot scenarios and need explicit handling.
- Plugin finalize/retrieve work must be specified as asynchronous and kept off audio/acquisition and UI message threads.
- Analyzer-backed local retrieval verification is too weak if it remains optional or merely documented.
- Verification must prove recording continues while the host is gone using SD continuity, not status counters alone.

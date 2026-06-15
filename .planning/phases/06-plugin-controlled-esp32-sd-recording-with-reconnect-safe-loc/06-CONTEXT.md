# Phase 06: Plugin-Controlled ESP32 SD Recording With Reconnect-Safe Local Retrieval - Context

**Gathered:** 2026-06-15
**Status:** Ready for planning

<domain>
## Phase Boundary

This phase turns the Open Ephys plugin RECORD button into the operator-facing control for ESP32 SD recording and post-record file retrieval. Pressing RECORD starts device-side SD logging on the ESP32. Pressing RECORD again stops/flushed the ESP32 log and retrieves the completed recording to the local PC.

The phase also defines the connection-loss behavior: if the plugin/bridge disconnects while recording, the ESP32 continues logging autonomously for a configurable grace period, then flushes/closes the SD file and stops recording if the host has not reconnected. If the host reconnects before the grace period expires, recording continues normally.

This phase does not replace the SD-card stress harness or the SD-file continuity analyzer. It consumes those as verification tools.

</domain>

<decisions>
## Implementation Decisions

### Recording Ownership
- **D-01:** Normal ESP32 recording is board-side SD logging, not PC-side CSV recording.
- **D-02:** The plugin RECORD button is the operator control for ESP32 SD recording: first press sends a start command, second press sends a stop/finalize command.
- **D-03:** The plugin must not claim "Recording saved" merely because it sent `RECORD OFF`; it must show firmware-confirmed status such as saved sample count, error count, and retrieved local file path.

### Local Retrieval
- **D-04:** After the user stops recording from the plugin, the ESP32 must flush/close the SD recording and the PC-side tool/plugin must retrieve the completed file to local storage.
- **D-05:** Local-machine saving is a post-record export/retrieval step. It is not the primary lossless acquisition path during live recording.
- **D-06:** The retrieved local file must be analyzable by the existing SD binary analyzer or a documented successor so continuity can be verified after transfer.

### Connection-Loss Back-Out
- **D-07:** If host/plugin/bridge connection is lost while recording, the ESP32 continues SD logging instead of stopping immediately.
- **D-08:** The reconnect grace period must be defined near the top of the relevant firmware file as an easily changed constant, defaulting to one or two minutes.
- **D-09:** If the host reconnects within the grace period, the session continues without creating a false stop/start boundary.
- **D-10:** If the host does not reconnect within the grace period, the ESP32 flushes/closes the SD file, stops logging, and preserves enough status metadata for the next connection to report what happened.

### Protocol and Status Feedback
- **D-11:** The control protocol must acknowledge `RECORD ON`, `RECORD OFF`, disconnect timeout finalization, and file retrieval outcomes explicitly.
- **D-12:** Status exposed to the plugin/bridge must include at minimum SD ready/open state, recording state, path/session id, generated samples, saved samples, write errors, queue drops, max write latency, and whether finalization was manual or timeout-triggered.
- **D-13:** USB bridge mode and direct TCP mode must behave consistently. In particular, bridge command relay must include recording commands and status/file-transfer commands, not only frequency/filter commands.

### the agent's Discretion
The planner may choose the exact transfer mechanism after research. Acceptable directions include a simple command-channel file dump, a sidecar TCP transfer endpoint, chunked serial transfer for USB bridge mode, or a staged copy tool, as long as recording remains device-side and the transfer has integrity checks.

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Prior Phase Decisions
- `.planning/phases/01-measurement-contract-baseline/01-CONTEXT.md` - Establishes SD card data as acquisition ground truth and Open Ephys/USB/TCP as comparison transports.
- `.planning/phases/02-buffered-sd-logger/02-CONTEXT.md` - Locks board-side SD recording as the normal ESP32 recording path and defines buffered logger counters.
- `.planning/REQUIREMENTS.md` - Defines SD, loss accounting, stress, stall isolation, and operator verification requirements for the milestone.
- `.planning/ROADMAP.md` - Defines phase ordering and existing Phase 2/3/4 boundaries.

### Code Paths
- `acqboard.ccp` - Plugin AcqBoard connection, ESP32 detection, RECORD command sending, legacy PC-side CSV recording path, and live stream handling.
- `Acqboardredpitaya.h` - ESP32 board state, command socket, recording paths, and legacy `esp32RecordStream` members.
- `device editor.cpp` - RECORD button UI behavior and status messages shown to the operator.
- `esp32/host/serial_tcp_bridge.py` - USB serial to TCP bridge used by plugin mode; currently relays some plugin commands to serial.
- `esp32/host/run_usb_plugin_bridge.ps1` - Operator wrapper for bridge mode.
- `esp32/firmware/main/sd_logger.c` and `esp32/firmware/main/sd_logger.h` - ESP-IDF buffered SD logger and counters.
- `esp32/firmware/main/open_ephys_stream.c` - ESP-IDF TCP command handling for `RECORD ON/OFF`.
- `esp32/arduino/step_node/step_node.ino` - Arduino firmware command/status surface used by serial stress tests and USB bridge mode.
- `esp32/host/stress_test_serial.py` and `esp32/host/analyze_sample_rate.py` - Verification tools for stress sweeps and SD binary continuity checks.

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `sd_logger_get_stats()` already exposes the ESP-IDF logger counters needed for truthful plugin status.
- Arduino `step_node.ino` already prints `SD_STATUS` and `STATUS` fields including `sd_saved`, `sd_errors`, `max_sd_write_us`, loop overruns, and recording state.
- `stress_test_serial.py --sd-file` already routes copied SD binary files through `analyze_sample_rate.py`.

### Established Patterns
- ESP32 plugin detection is based on handshake text containing ESP32 markers, after which `sendRecordOnCommand()` sends `RECORD ON`.
- Plugin UI status currently derives from locally stored `lastRecordingPath` values; this should be replaced or augmented with firmware-confirmed status.
- USB bridge mode uses `serial_tcp_bridge.py --plugin` to make the ESP32 look like a TCP board at `127.0.0.1:5000`.

### Integration Points
- `sendRecordOnCommand()` and `sendRecordOffCommand()` are the immediate plugin hooks for start/stop recording behavior.
- `relay_plugin_commands()` in `serial_tcp_bridge.py` currently forwards `FREQ`, `CFG`, `FILTER`, and `STOP`, but not `RECORD`; bridge-mode recording must be included in this phase.
- The existing `esp32RecordStream` path is explicitly legacy PC-side CSV recording and should not be the normal save path for ESP32 SD recording.
- Firmware needs a disconnect/reconnect state machine tied to recording state, not merely socket lifetime.

</code_context>

<specifics>
## Specific Ideas

- Put the reconnect timeout constant near the top of the firmware file, with a default around one to two minutes.
- Treat connection loss as a temporary host problem: keep logging first, then finalize only after the grace period expires.
- If the host reconnects in time, continue the original recording session as if nothing happened.
- On timeout finalization, preserve status so the plugin can tell the operator the file was safely closed due to disconnect.
- Retrieval should happen after stop/finalize, not continuously during the session.

</specifics>

<deferred>
## Deferred Ideas

- Full SD-card model benchmarking remains deferred to future acquisition enhancements unless the existing stress harness makes it cheap.
- Multi-board SD log synchronization is still out of scope for this phase.
- Replay from SD back into Open Ephys as a live reconstruction feature remains out of scope unless chosen as the simplest retrieval representation.

</deferred>

---

*Phase: 06-plugin-controlled-esp32-sd-recording-with-reconnect-safe-loc*
*Context gathered: 2026-06-15*

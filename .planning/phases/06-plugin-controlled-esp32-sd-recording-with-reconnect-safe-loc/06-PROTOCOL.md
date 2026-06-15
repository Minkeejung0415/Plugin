---
phase: 06-plugin-controlled-esp32-sd-recording-with-reconnect-safe-loc
protocol: rec-v1
status: contract
applies_to:
  - esp32/firmware/main/open_ephys_stream.c
  - esp32/firmware/main/sd_logger.c
  - esp32/firmware/main/sd_logger.h
  - esp32/arduino/step_node/step_node.ino
  - esp32/host/serial_tcp_bridge.py
  - acqboard.ccp
  - Acqboardredpitaya.h
  - device editor.cpp
requirements: [SD-01, SD-03, SD-04, SD-05, LOSS-03, LOSS-04, STALL-03, OPS-01]
---

# Phase 06 Recording Protocol and Status Contract

This contract defines `rec-v1`, the shared command, status, reconnect, and finalized-file retrieval protocol for ESP32 SD recording controlled by the Open Ephys plugin. It applies to both direct TCP firmware mode and USB bridge mode. Direct TCP and USB bridge implementations may use different physical transports, but they must expose the same command names, status fields, recording semantics, transfer framing, and error behavior.

## Protocol Decisions

| Decision | Contract |
| --- | --- |
| D-01 | Normal ESP32 recording is board-side SD logging. Host-side CSV capture is legacy diagnostic behavior and is not the normal ESP32 save path. |
| D-02 | Plugin `RECORD` is the operator control for ESP32 SD recording. First press sends `REC START`; second press sends `REC STOP`. |
| D-03 | The plugin must not report `saved`, `recording saved`, or equivalent completion until the workflow reaches analyzer-backed local verification. |
| D-04 | Stop workflow is `REC STOP` -> firmware finalization -> finalized-session metadata -> local retrieval -> transfer checksum -> analyzer pass. |
| D-05 | Local-machine saving is post-record retrieval from a finalized SD file. It is not primary live acquisition. |
| D-06 | Retrieved files must be compatible with the existing SD analyzer or with a documented successor declared by protocol capabilities. |
| D-07 | Host, plugin, or bridge connection loss while recording keeps ESP32 SD logging active during the firmware-side grace period. |
| D-08 | Supported firmware must define a top-of-file reconnect grace constant, defaulting to 60-120 seconds. |
| D-09 | Reconnect within the grace period resumes control of the same session id without creating a false stop/start boundary. |
| D-10 | Grace timeout finalizes the SD file, stops recording, and retains finalized-session metadata for the next connection. |
| D-11 | Start, stop/finalize, timeout finalization, retrieval completion, retrieval abort, and retrieval failure require explicit acknowledgements. |
| D-12 | Status includes protocol, SD, recording, transfer, counters, finalization, integrity, latency, and local verification fields listed below. |
| D-13 | Direct TCP and USB bridge modes use consistent record, status, and transfer command expectations. |

## Version and Capability Negotiation

Every host connection must begin with:

```text
REC HELLO protocol_min=rec-v1 client=<plugin|bridge|tool> mode=<direct_tcp|usb_bridge>
```

Successful response:

```text
REC HELLO_OK protocol=rec-v1 firmware=<id> transport=<direct_tcp|usb_bridge> capabilities=<csv-list> max_chunk=<bytes> analyzer=<sd-bin-v1|successor-id> grace_ms=<ms>
```

Unsupported response:

```text
REC ERR code=unsupported_protocol detail=<reason>
```

Minimum `rec-v1` capabilities are:

| Capability | Meaning |
| --- | --- |
| `record_control` | Firmware accepts `REC START` and `REC STOP`. |
| `status_v1` | Firmware reports all minimum status fields in this contract. |
| `finalized_metadata` | Firmware retains metadata for latest finalized sessions. |
| `chunk_transfer_v1` | Firmware can send framed chunks by session id and byte offset. |
| `whole_file_checksum` | Firmware reports a whole-file checksum for finalized files. |
| `reconnect_grace` | Firmware continues SD logging through host loss until the grace timeout. |

Old firmware that lacks `rec-v1` must be treated as unsupported for plugin-controlled ESP32 SD recording. The plugin may show live stream status, but it must not fall back to claiming SD recording success through legacy PC-side CSV behavior.

## Lifecycle Surfaces

The protocol has three separate lifecycle surfaces. Implementations must not collapse them into one `recording` or `saved` boolean.

### Firmware Recording State

| State | Meaning | Allowed next states |
| --- | --- | --- |
| `idle` | No SD file is open for recording. | `starting`, `failed` |
| `starting` | Start command accepted; firmware is mounting/opening/preparing the SD file. | `recording`, `failed`, `finalizing` |
| `recording` | Samples are being generated and logged to the board-side SD file. | `host-disconnected-grace`, `finalizing`, `failed` |
| `host-disconnected-grace` | Host/bridge/plugin control path is gone; SD logging continues under the firmware grace timer. | `recording`, `finalizing`, `failed` |
| `finalizing` | Firmware is flushing and closing the SD file. No new file bytes may be transferred until this finishes. | `finalized`, `failed` |
| `finalized` | SD source file is closed read-only and metadata is retained for retrieval. | `idle`, `starting` |
| `failed` | Recording cannot continue or finalize cleanly. Status must include `last_error` and retained counters. | `idle`, `starting` |

### Transfer State

| State | Meaning | Allowed next states |
| --- | --- | --- |
| `none` | No finalized-file transfer is active. | `metadata`, `failed` |
| `metadata` | Host is querying session metadata before chunk retrieval. | `chunking`, `aborted`, `failed`, `none` |
| `chunking` | Firmware is serving binary-safe frames for a finalized SD file. | `complete`, `aborted`, `failed`, `none` |
| `complete` | Firmware sent EOF and host acknowledged retrieval completion. | `none` |
| `aborted` | Host or firmware aborted the transfer. SD source remains intact. | `none`, `metadata` |
| `failed` | Transfer failed due to checksum, offset, transport, SD, or session errors. SD source remains intact. | `none`, `metadata` |

### Plugin and Local State

| State | Meaning |
| --- | --- |
| `command-sent` | Plugin sent start/stop/retrieve command but has not received firmware acknowledgement. |
| `recording-confirmed` | Firmware acknowledged recording and reports a session id. |
| `finalizing` | Plugin requested stop or detected timeout-finalized state and is waiting for firmware finalization metadata. |
| `sd-finalized` | Firmware reports a finalized SD file with session metadata. |
| `retrieving` | Local process is writing framed chunks to a local temp file and transfer log. |
| `local-copy-written` | Local file is fully written and atomically moved into its final local path. |
| `transfer-checksum-passed` | Whole-file checksum from firmware matches local checksum and transfer log has no unresolved gaps. |
| `analyzer-passed` | SD analyzer continuity checks passed for the local file. This is the first state that may be called fully saved/verified. |
| `analyzer-failed` | Transfer integrity may be intact, but SD continuity verification failed. The UI must present this as not fully verified. |

## Commands and Responses

Commands are ASCII control messages terminated by newline. Binary file bytes are never embedded directly in command responses; finalized-file data is carried only in frames defined in `Binary Retrieval Framing`.

| Command | Request fields | Success response | Error codes | Timeout/retry behavior |
| --- | --- | --- | --- | --- |
| `REC HELLO` | `protocol_min`, `client`, `mode` | `REC HELLO_OK protocol firmware transport capabilities max_chunk analyzer grace_ms` | `unsupported_protocol`, `busy`, `internal_error` | Host retries once after reconnect; unsupported is terminal for SD recording. |
| `REC START` | `requested_session=<optional token>`, `sample_rate_hz`, `channels`, `format`, `sd_required=<true|false>` | `REC STARTED session_id sd_path_token recording_state=recording generated_samples saved_samples` | `unsupported`, `sd_not_ready`, `already_recording`, `open_failed`, `invalid_arg` | If no response, host must query `REC STATUS` before retrying to avoid duplicate sessions. |
| `REC STATUS` | none or `session_id` | `REC STATUS_OK` plus minimum status fields | `unsupported`, `not_found`, `internal_error` | Safe to poll. Must not block acquisition loop. |
| `REC STOP` | `session_id`, `reason=manual_stop` | `REC FINALIZING session_id` followed by status reaching `recording_state=finalized`, or `REC FINALIZED ...` if already complete | `not_recording`, `session_mismatch`, `finalize_failed`, `sd_error` | If transport drops after `REC STOP`, reconnect and query `REC STATUS`/`REC SESSION` before resending. |
| `REC SESSION` | `session_id=<id>|latest_finalized` | `REC SESSION_OK session_id sd_path_token file_size file_checksum checksum_type sample_count finalized_at finalization_reason analyzer_format` | `not_finalized`, `not_found`, `busy_recording`, `unsupported` | May be retried after reconnect. Must not open arbitrary paths. |
| `REC GET` | `session_id`, `offset`, `length`, `chunk_index` | One or more `SDRF` frames for the requested byte range | `not_finalized`, `not_found`, `offset_out_of_range`, `busy_recording`, `transfer_busy`, `sd_error` | Host retries by offset. Firmware may resend duplicate chunks. |
| `REC COMPLETE` | `session_id`, `file_size`, `file_checksum`, `checksum_type`, `local_transfer_id` | `REC COMPLETE_OK session_id transfer_state=complete` | `checksum_mismatch`, `size_mismatch`, `session_mismatch`, `not_active` | Host sends only after local whole-file checksum passes. |
| `REC ABORT` | `session_id`, `reason` | `REC ABORTED session_id transfer_state=aborted` | `not_active`, `session_mismatch`, `internal_error` | Safe to retry; SD source file remains read-only and retained. |
| `REC CLEAR` | `scope=<errors|last_finalized|transfer>` | `REC CLEAR_OK scope` | `busy_recording`, `invalid_scope`, `unsupported` | Must not delete or truncate SD source files. `last_finalized` only clears retained status pointers after retrieval policy permits. |

All error responses use:

```text
REC ERR code=<code> session_id=<optional> retryable=<true|false> detail=<short-token>
```

## Minimum Status Fields

`REC STATUS_OK` must include these fields, even when values are `unknown` because older compatible firmware cannot compute them. Implementers should prefer explicit `unknown` over omission.

| Field | Meaning |
| --- | --- |
| `protocol` | Protocol version, expected `rec-v1`. |
| `capabilities` | Comma-separated capability set. |
| `transport` | `direct_tcp` or `usb_bridge`. |
| `sd_ready` | SD card mounted and usable. |
| `sd_open` | An SD recording file is currently open. |
| `recording_state` | One firmware recording state from this contract. |
| `transfer_state` | One transfer state from this contract. |
| `session_id` | Firmware-issued opaque session id for current or last finalized recording. |
| `sd_path_token` | Opaque firmware-issued retrieval token, not a host-supplied filesystem path. |
| `generated_samples` | Number of samples generated by the acquisition loop for the session. |
| `saved_samples` | Number of samples durably accepted by SD logging for the session. |
| `queue_drops` | Samples dropped before SD write due to queue/backpressure. |
| `write_errors` | SD write or flush errors for the session. |
| `max_write_latency_us` | Maximum measured SD write latency for the session. |
| `overrun_count` | Acquisition-loop overrun count for the session. |
| `finalization_reason` | `none`, `manual_stop`, `disconnect_timeout`, `sd_error`, `firmware_shutdown`, `reboot_detected`, or `unknown`. |
| `file_byte_size` | Finalized SD file size in bytes, or `0` while not finalized. |
| `file_checksum` | Whole-file checksum after finalization, or `none` while not finalized. |
| `checksum_type` | `crc32c`, `sha256`, or another negotiated capability value. |
| `last_error` | Last protocol or SD error token, or `none`. |
| `grace_ms_remaining` | Remaining reconnect grace time when in `host-disconnected-grace`, else `0`. |
| `local_result_path` | Host/plugin local result path if known to the local side; firmware and bridge report `unknown`. |
| `local_analyzer_result` | `not_run`, `passed`, `failed`, or `unknown`; firmware and bridge report `unknown`. |

The plugin may display `SD finalized` after `recording_state=finalized`. It may display `Local retrieval complete` after `local-copy-written`. It may display `Transfer checksum passed` after `transfer-checksum-passed`. It may display `Recording saved` only after `analyzer-passed`.

## Binary Retrieval Framing

All finalized-file payloads use the `SDRF` frame format. This prevents file bytes from being interpreted as command text or live Open Ephys sample frames.

### Frame Header

Multi-byte integer fields are little-endian. Header bytes are fixed-size and followed by `payload_length` bytes. Header CRC covers every header byte from `magic` through `reserved`, with `header_crc32c` set to zero for calculation.

| Offset | Size | Field | Required value / meaning |
| --- | ---: | --- | --- |
| 0 | 4 | `magic` | ASCII `SDRF`. |
| 4 | 1 | `frame_version` | `1`. |
| 5 | 1 | `frame_type` | `0x01=data`, `0x02=eof`, `0x03=abort`, `0x04=metadata`, `0x05=error`. |
| 6 | 2 | `header_length` | `64` for `rec-v1`. |
| 8 | 16 | `session_id` | 16-byte binary session id or canonical binary representation of firmware-issued id. |
| 24 | 4 | `chunk_index` | Zero-based chunk index. EOF repeats the next expected chunk index. |
| 28 | 8 | `byte_offset` | Absolute byte offset in the finalized file. |
| 36 | 4 | `payload_length` | Number of payload bytes following the header. Must be `0` for EOF and abort unless an error payload is negotiated. |
| 40 | 8 | `total_file_size` | Finalized file size in bytes. |
| 48 | 4 | `header_crc32c` | CRC32C of the header with this field zeroed. |
| 52 | 4 | `payload_checksum` | CRC32C of payload for data frames; `0` for EOF/abort with no payload. |
| 56 | 4 | `flags` | Bit 0 `retry`, bit 1 `duplicate`, bit 2 `last_data_before_eof`, other bits zero. |
| 60 | 4 | `reserved` | Zero for `rec-v1`. |

### Frame Types

| Type | Behavior |
| --- | --- |
| `data` | Carries exactly `payload_length` bytes from `byte_offset`. Host writes only if offset and checksum match expected transfer log. |
| `metadata` | Optional machine-readable metadata payload before data. Must not replace `REC SESSION`. |
| `eof` | Marks that all bytes through `total_file_size` have been sent. Host must still validate whole-file checksum before `REC COMPLETE`. |
| `abort` | Firmware aborted transfer. Host stops writing and records failure. SD source remains intact. |
| `error` | Firmware reports a transfer error frame; payload is ASCII key/value error text. Host may resume by `REC GET` if retryable. |

### Chunk, Retry, and Resume Rules

- Host requests chunks by absolute `offset`, `length`, and `chunk_index`.
- Firmware must not send a data frame whose `byte_offset` differs from the requested offset unless it first sends an error response.
- Duplicate chunks are valid after retry or reconnect. Host accepts a duplicate only if `byte_offset`, `payload_length`, and `payload_checksum` match the already logged chunk; otherwise it records `checksum_mismatch` and retries from the first unresolved offset.
- Missing chunks are detected by transfer log gaps. Host resumes with `REC GET session_id=<id> offset=<first_missing_offset>`.
- Offset mismatch is fatal to the current chunk request. Host must discard that frame, query `REC STATUS`, then retry from the first unresolved offset.
- Payload checksum mismatch requires discarding the payload and retrying the same offset. After three mismatches for the same offset, host marks transfer `failed`.
- Header CRC mismatch requires discarding the whole frame and retrying the same offset.
- EOF is valid only when all bytes from `0` through `total_file_size - 1` are present and chunk checksums have passed.
- Whole-file checksum mismatch after EOF leaves the SD source file intact and permits a new retrieval attempt from byte offset `0` or from the first known bad offset if the checksum algorithm and transfer log support range isolation.
- Reconnect during transfer does not affect the finalized SD source file. After reconnect, host sends `REC HELLO`, `REC SESSION session_id=<id>`, then resumes with `REC GET` from the first missing or unverified offset.
- Abort may be initiated by host with `REC ABORT` or by firmware with an abort frame. Abort never deletes, truncates, or marks the SD source file consumed.

## Traffic Separation From Live Open Ephys Sample Data

Command/status/file-transfer traffic and live Open Ephys sample traffic must be isolated so finalized-file bytes are never parsed as live sample frames.

Required behavior:

1. Live sample streaming remains a separate data mode from `rec-v1` control.
2. File retrieval is allowed only after firmware recording state is `finalized` or `failed` with a retained finalized source file.
3. During retrieval, the implementation must use one of these explicit isolation modes:
   - `command_only_transfer`: the connection is switched into command/transfer mode and live sample streaming is paused or absent.
   - `sidecar_endpoint`: file frames use a separate TCP endpoint or bridge channel negotiated by `REC HELLO_OK`.
   - `paused_isolated_stream`: live stream is explicitly paused and all received bytes are parsed only as `SDRF` frames until EOF/abort.
4. Direct TCP firmware must advertise the chosen mode in capabilities, for example `transfer_isolation=sidecar_endpoint`.
5. USB bridge mode must not multiplex raw file bytes into the plugin's live sample parser. The bridge must either expose a command-only retrieval channel to the plugin/tool or pause/isolate the plugin-facing stream while framing remains active.
6. Any byte sequence that does not start with `SDRF` while transfer state is `chunking` is a transfer framing error, not a sample frame.

## Reconnect and Failure Semantics

| Failure mode | Firmware behavior | Host/plugin/bridge behavior |
| --- | --- | --- |
| Direct TCP client disconnect while `recording` | Enter `host-disconnected-grace`, keep SD logging, start grace timer. | Reconnect, send `REC HELLO`, query `REC STATUS`; if within grace, continue same `session_id`. |
| Plugin-to-bridge disconnect while `recording` | Firmware may not know plugin is gone if bridge remains connected; bridge must keep serial control passive and not send stop. | Bridge preserves last known session/status and allows plugin reconnect. Plugin queries status and resumes same session. |
| Bridge process loss while `recording` | Firmware sees serial/TCP control path loss if detectable, enters grace if supported, and keeps SD logging. | Restart bridge; bridge sends `REC HELLO` and relays preserved status/session to plugin. |
| USB/serial device loss | Firmware may continue only if powered and SD logging task is independent. If firmware detects host loss, enter grace. If power is lost, this becomes firmware reboot. | On reconnect, bridge/plugin treat status as unknown until `REC HELLO`/`REC STATUS`. |
| Transfer disconnect | Finalized SD file remains read-only and retained. Transfer state returns to `none` or `failed`; recording state remains `finalized`. | Reconnect and resume with `REC SESSION` plus `REC GET` from first missing offset. |
| Firmware reboot | Active unfinalized session is `failed` or `reboot_detected` unless firmware can prove a finalized file and metadata survived. | Plugin must not claim saved. It may offer retrieval only if `REC SESSION latest_finalized` returns a valid finalized token and checksum. |
| Grace timeout | Firmware transitions `host-disconnected-grace` -> `finalizing` -> `finalized`, with `finalization_reason=disconnect_timeout`. | On next connection, plugin shows timeout-finalized session and offers local retrieval. |

The reconnect grace constant must be near the top of supported firmware files, named clearly, and default to 60-120 seconds. Example names are `REC_RECONNECT_GRACE_MS` for ESP-IDF and `REC_RECONNECT_GRACE_MS` or equivalent in Arduino firmware.

## Path and Integrity Safety

- Retrieval commands accept only `session_id` or `latest_finalized`. They must not accept host-supplied SD filenames or directory paths.
- `sd_path_token` is opaque. It may be displayed for diagnostics, but host code must not interpret it as an arbitrary path to read.
- Firmware must serve only finalized files that it created for recording sessions.
- Retrieval is read-only against finalized SD source files.
- Successful or failed local transfer must never delete, truncate, rename, or mark the SD source file as consumed.
- `REC CLEAR scope=last_finalized` may clear retained status pointers only after policy allows it; it still must not delete the SD file.
- Local writers must use a temp file plus transfer log and atomically move the file into the final local result path only after EOF and chunk checks pass.

## Local Integrity and Analyzer Verification

The post-record workflow has four distinct success levels:

| Level | Required proof | User-facing language |
| --- | --- | --- |
| SD finalized | Firmware `recording_state=finalized`, `file_byte_size`, `file_checksum`, and finalization reason are present. | `SD finalized` |
| Local transfer complete | EOF received, no transfer-log gaps, local file written atomically. | `Local retrieval complete` |
| Transfer checksum passed | Local whole-file checksum matches firmware `file_checksum`. | `Transfer checksum passed` |
| Analyzer continuity passed | SD analyzer or negotiated successor reports sequence/timestamp continuity pass and counters agree. | `Recording saved and verified` |

Required local artifacts:

- Retrieved binary file in analyzer-compatible format or documented successor format.
- Metadata sidecar containing session id, firmware id, protocol version, file size, checksum, finalization reason, generated/saved counters, write errors, queue drops, max write latency, and overrun count.
- Transfer log with chunk index, byte offset, payload length, per-chunk checksum, retries, duplicate chunks, and unresolved gaps.
- Analyzer result file with pass/fail status and continuity findings.

`LOSS-04` is satisfied only when the analyzer reports clean SD sequence continuity and firmware counters agree. A transfer checksum pass alone does not prove no samples were lost during acquisition.

## Direct TCP and USB Bridge Expectations

Direct TCP firmware path:

- Implements `REC HELLO`, `REC START`, `REC STATUS`, `REC STOP`, `REC SESSION`, `REC GET`, `REC COMPLETE`, `REC ABORT`, and `REC CLEAR`.
- Keeps acquisition-loop status queries non-blocking.
- Uses a declared transfer isolation mode so file frames never collide with live Open Ephys samples.
- Preserves finalized-session metadata across reconnect until cleared or superseded by a new explicit session policy.

USB bridge path:

- Relays all `rec-v1` commands and responses, including record, status, session metadata, and retrieval.
- Does not support only frequency/filter commands; bridge-mode recording control is first-class.
- Preserves the same visible protocol behavior as direct TCP even if the physical transport is serial.
- Uses `SDRF` framing for file payload and keeps framed file bytes out of the plugin live sample parser.
- Reports bridge-specific failures as `REC ERR` details while preserving firmware error codes when available.

Firmware target ownership:

- ESP-IDF direct TCP support is required for direct plugin connection.
- Arduino USB bridge support is required when the operator workflow uses `serial_tcp_bridge.py --plugin`.
- If a firmware variant cannot support finalized-file retrieval, it must advertise lack of `chunk_transfer_v1`; the plugin must then report recording protocol unsupported for local retrieval rather than claiming saved.

## Unsupported, Error, and Timeout Behavior

Common error codes:

| Code | Retryable | Meaning |
| --- | --- | --- |
| `unsupported_protocol` | false | Firmware or bridge does not support `rec-v1`. |
| `unsupported` | false | Command is not supported by this mode/capability set. |
| `sd_not_ready` | true | SD card is not mounted or currently unavailable. |
| `open_failed` | true | Firmware could not create/open the SD recording file. |
| `already_recording` | false | Start requested while a session is already recording. |
| `not_recording` | false | Stop requested when no session is recording. |
| `session_mismatch` | false | Command session id does not match current/retained session. |
| `not_finalized` | true | Retrieval requested before finalization is complete. |
| `not_found` | false | Session id/token is unknown or no longer retained. |
| `offset_out_of_range` | false | Requested offset is outside finalized file size. |
| `checksum_mismatch` | true | Chunk or whole-file checksum failed. |
| `size_mismatch` | true | Host-reported local size differs from firmware file size. |
| `transfer_busy` | true | A transfer is already active. |
| `sd_error` | true | SD read/write/flush error occurred. |
| `finalize_failed` | true | Firmware could not cleanly finalize; status must preserve counters and error. |
| `internal_error` | true | Unexpected implementation error. |

Timeout rules:

- `REC START` acknowledgement timeout: host queries `REC STATUS` before retrying.
- `REC STOP` finalization timeout: host polls `REC STATUS` until `finalized` or `failed`; it must not report saved while waiting.
- `REC GET` chunk timeout: host retries the same offset up to three times, then marks transfer `failed`.
- Reconnect after any timeout begins with `REC HELLO` and `REC STATUS` or `REC SESSION`; host must not assume the previous command failed or succeeded without status proof.

## Verification Hooks for Downstream Plans

Downstream firmware, bridge, and plugin work must prove:

- Every state in the three lifecycle surfaces is either implemented or explicitly unreachable for the selected mode.
- `Recording saved` is impossible before analyzer pass.
- Direct TCP and USB bridge expose the same `rec-v1` command/status/file-transfer semantics.
- Reconnect before grace timeout preserves the same session id and SD file.
- Grace timeout finalizes with `finalization_reason=disconnect_timeout` and retained metadata.
- Retrieval retry after a finalized SD file survives transfer disconnects, checksum failures, and process restart.
- File bytes cannot be interpreted as live Open Ephys sample frames.
- Unsupported firmware produces a clear protocol unsupported result instead of legacy optimistic recording success.
- Retrieval cannot read arbitrary SD paths and cannot delete or truncate source files.
- Analyzer continuity pass, not transfer completion alone, is the final verification state.

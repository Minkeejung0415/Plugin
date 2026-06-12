# Summary 02-01: ESP-IDF Buffered SD Logger

**Date:** 2026-06-12
**Status:** Implemented with firmware build pending local ESP-IDF environment

## Implemented

- Replaced acquisition-loop SD file writes with `sd_logger_append()` non-blocking queue enqueue.
- Added `sd_writer_task()` as the only firmware path that opens, writes, flushes, and closes the SD log file.
- Added a binary session header before `oe_sample_t` sample records.
- Added SD logger counters for generated, enqueued, saved, queue drops, write errors, max queue depth, max enqueue latency, max write/flush latency, acquisition overruns, and max acquisition-loop duration.
- Added `RECORD ON` / `RECORD OFF` handling in the ESP-IDF TCP command path.
- Changed the Open Ephys plugin ESP32 RECORD button to send `RECORD ON` / `RECORD OFF` to the board instead of opening a PC-side CSV file.
- Installed a rebuilt Debug `acquisition-board.dll` into the dev Open Ephys GUI plugin folder.

## Important Boundary

The SD logger still assumes `/sdcard` is already mounted. Full ESP-IDF SDSPI mount/pin configuration is intentionally deferred until board-specific SD wiring is confirmed.

## Verification

- `git diff --check` passed for changed firmware, plugin, and phase files.
- Visual Studio/CMake plugin build passed:
  `cmake --build . --config Debug --target acquisition-board -- /m`
- Visual Studio/CMake plugin install passed:
  `cmake --build . --config Debug --target INSTALL -- /m`
- Installed DLL:
  `C:\Users\justi\dev\GUI\Build\Debug\plugins\acquisition-board.dll`

## Not Verified

- ESP-IDF firmware build was not run because `idf.py` is not on PATH and `IDF_PATH` is not set in this shell.
- Hardware SD recording was not run in this phase.

# AGENTS.md

## Cursor Cloud specific instructions

### Codebase overview

This is an **Open Ephys Acquisition Board Plugin** — a C/C++ plugin for the Open Ephys GUI neuroscience data acquisition platform. It interfaces with Red Pitaya, OpalKelly, ONI, and simulated hardware boards. The repo contains:

- **C++ plugin code** (`.cpp`/`.h` files at repo root) — requires the Open Ephys GUI SDK and JUCE framework to compile; cannot be built standalone in this environment.
- **Red Pitaya server** (`RedPitaya_justin.c`) — bare-metal C for ARM Linux on the Red Pitaya FPGA board; requires `rp.h` and related SDK headers not present in this repo.
- **Standalone test** (`tests/redpitaya_stream_framing_test.c`) — the only buildable/runnable artifact in this environment.

### Running tests

The only test that can run without external hardware or the Open Ephys SDK:

```bash
cc -std=c99 -Wall -Wextra -o /tmp/rp_framing_test tests/redpitaya_stream_framing_test.c && /tmp/rp_framing_test
```

This validates the binary stream header framing and resync logic. Expected output: `OK: variable payload framing and resync`.

### What cannot run in this environment

- The C++ plugin requires the Open Ephys GUI SDK (external CMake build system) and JUCE framework — these are not in this repo.
- `RedPitaya_justin.c` requires the Red Pitaya SDK (`rp.h`, `axi_header.h`, `sensor_fusion.h`, `vqf.h`) and ARM cross-compilation.
- Full end-to-end testing requires physical Red Pitaya hardware connected over TCP (port 5000).

### Build and lint

No formal lint or build system is checked into this repo. The C test file can be compiled with any C99-compatible compiler (`cc` or `gcc`). For the full plugin build, see the Open Ephys plugin build documentation (external).

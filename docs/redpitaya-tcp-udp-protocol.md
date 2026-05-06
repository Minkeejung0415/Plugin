# Red Pitaya Open Ephys transport (TCP control + UDP samples)

This document is the **normative handshake** between `RedPitaya_justin.c` (firmware) and `AcqBoardRedPitaya` (`acqboard.ccp`). Follow the order exactly.

---

## 1. Ports and sockets

| Role | Protocol | Port | Notes |
|------|-----------|------|--------|
| Commands + text responses | TCP | **5000** (host listens) | Single byte stream for control only during acquisition. |
| Sample frames | UDP | **Ephemeral** on PC | PC binds UDP; board sends with `sendto` to PC IP + announced port. |

---

## 2. Connection lifecycle (plugin / PC)

1. **TCP connect** to Red Pitaya `*:5000`.
2. Optional idle commands: `FREQ:…`, `CFG …`, `REDPITAYA`, etc. (same as before).
3. **Before `START`**: bind a **UDP** socket on an ephemeral port (OS chooses port).
4. Send on **TCP**: `STREAM_UDP <port>\n` where `<port>` is the bound UDP port (decimal).
5. Send on **TCP**: `START\n`.
6. Read **TCP** lines until stream ends: first response line after `START` is success or error; second line is `SENSORS:…` when successful.
7. Read **UDP** datagrams (one complete frame per datagram) until acquisition stops.
8. **Stop**: close **UDP** receive socket first (unblocks reader thread), then close **TCP** (server exits `run_stream` on send failure / EOF).

---

## 3. Firmware (`RedPitaya_justin.c`) behavior

1. Listen **TCP** on port 5000; create one **UDP** transmit socket for `sendto` (no need to bind for TX).
2. On **TCP accept**, clear any previous UDP peer (`stream_udp_ready = false`).
3. On line containing **`STREAM_UDP <port>`**: use **`getpeername`** on the TCP socket for the client IPv4 address; set UDP destination port from `<port>`; set `stream_udp_ready = true`.
4. On **`START`**: open SD/CSV files as today; if **`stream_udp_ready` is false**, respond with **`ERROR_STREAM_UDP\n`**, close files, stay in command loop (do not enter `run_stream`).
5. In **`run_stream`**: send each sample frame with **`sendto`** only; **`send` failures do not stop acquisition** (SD/CSV remain authoritative). TCP is still used for **`process_stream_commands`** only.

---

## 4. Text responses (TCP)

| Line | When |
|------|------|
| `OK CHANNELS:<n> UDP:1\n` | Reply to `REDPITAYA` (UDP extension supported). Clients may ignore `UDP:1` if they only parse `CHANNELS:`. |
| `STARTED` or `STARTED BIN:… CSV:…\n` | Successful start of `run_stream`. |
| `ERROR_STREAM_UDP\n` | `START` received but no valid **`STREAM_UDP`** handshake for this TCP session. |
| `ERROR_FILE\n` | Recording files could not be opened (same as legacy). |
| `SENSORS:…\n` | Immediately after the `STARTED` line (snapshot at stream start). |
| `STOPPED\n` | After `run_stream` returns normally (TCP command loop continues). |

---

## 5. Binary frame format (UDP payload)

- **One UDP datagram = one frame** = **22-byte header** + **little-endian int16 payload** (same layout as legacy TCP streaming).
- Plugin must **not** treat UDP as a byte stream: no sliding-window resync across datagrams; invalid or truncated datagrams are **discarded**.

---

## 6. Mitigations summary

| Risk | Mitigation |
|------|------------|
| Wrong UDP target | IP from **`getpeername`**, port from **`STREAM_UDP`** only. |
| `START` without handshake | **`ERROR_STREAM_UDP`**, no partial stream. |
| Plugin hang on stop | Close **UDP** socket before **TCP** in `stopAcquisition`. |
| MTU / fragmentation | Prefer frame sizes under typical Ethernet MTU; increase receive buffer size if channel count grows. |

---

## 7. Quick client checklist (Open Ephys)

1. `StreamingSocket::connect` → port 5000  
2. `FREQ:` (optional)  
3. `DatagramSocket::bindToPort(0)`  
4. `STREAM_UDP <getBoundPort()>`  
5. `START`  
6. Read TCP until `STARTED` / error; then read `SENSORS:` line  
7. `run()` reads **UDP** until thread exit  

---

## 8. MSVC / JUCE build notes

- Use **`#include <juce_core/juce_core.h>`** once (same content as repo root **`acqboard.ccp`** if your local file is **`AcqBoardRedPitaya.cpp`** — filenames do not matter).
- **Additional Include Directories** must contain JUCE’s **`modules`** folder (parent of **`juce_core`**).
- **`juce::DatagramSocket`** exposes **`shutdown()`**, not **`close()`**, before destroying the socket (unblocks **`run()`** on stop).

---

## 9. Why TCP “just worked” and UDP often does not

Commit **`ac19596`** only adds **monotonic timing instrumentation** in **`RedPitaya_justin.c`**; it does **not** implement or change the transport. The stable **all-TCP** setup was **main before the UDP split**: binary samples used the **same TCP socket** as text commands, so there was **no second socket**, **no inbound UDP firewall rule**, and **no `STREAM_UDP` handshake**.

| TCP-only (legacy) | TCP + UDP (current) |
|-------------------|---------------------|
| Single connection; ordered byte stream | Separate **UDP receive** socket on the PC |
| Firewall usually allows **outbound** TCP you initiated | **Inbound UDP** from the Red Pitaya IP may be **blocked** even when TCP works |
| Back-pressure could stall the GUI thread | **`STREAM_UDP`** must precede **`START`**; **firmware + plugin must match** |

### If live traces stay flat / streaming appears dead

1. Confirm **Windows Firewall** allows **inbound UDP** to the ephemeral port (test with firewall off briefly).
2. Confirm **`REDPITAYA`** returns **`UDP:1`** and both sides are rebuilt from the same branch.
3. Plugin binds **`bindToPort(0, "0.0.0.0")`** (IPv4 any) so **`AF_INET`** datagrams from the board are accepted.
4. Console **`ERROR_STREAM_UDP`** means the device never accepted the handshake (wrong/old firmware or **`STREAM_UDP`** not sent).

---

*Last updated: aligned with TCP+UDP split in `RedPitaya_justin.c` and `acqboard.ccp`.*

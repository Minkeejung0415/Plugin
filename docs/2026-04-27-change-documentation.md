
# If red pitaya does not boot

1. check if you can access the [rp-f0f85a.local](http://rp-f0f85a.local/) page
2. check if you can ssh into the red pitaya
3. check if blue and green lights are on and red light is blinking
4. If the led lights are doing fine but you can't ssh into the red pitaya, check if the red pitaya is correctly wired to the router which is connected to the PC
5. if those are fine but you can't access the red pitaya, go to terminal and use "arp -a" command and check if you can see f0f85a or f0cd35 as one of the addresses
6. If you can find them but can't access both of them check if the router is connected to the power on the wall
7. If that is also good, just unplug everything and let is rest for a bit

---

## Final summary (Red Pitaya + Open Ephys plugin)

This document describes the **current** behavior after the acquisition, streaming, UI, and stability work. Rebuild and deploy **both** the Open Ephys plugin and the Red Pitaya server binary when you change either side.

### Red Pitaya server (`RedPitaya_justin.c`)

| Topic | Behavior |
|--------|----------|
| **Recordings** | Streamed sessions still write **`.bin`** and **`.csv`** under `/root/Measurements/` as before. |
| **TCP RX after stream** | After `run_stream()` returns, **`drain_client_rx()`** clears leftover bytes in the client socket receive buffer so the next command loop `read()` does not see tail data from binary packets. |
| **Frame sequence** | First binary frame of each session uses sequence **`0`** in the 22-byte header (`ns` starts at `-1`, increments once per frame before acquire/send). |
| **Listen socket** | **`SO_REUSEADDR`** (existing) and **`SO_REUSEPORT`** when the OS defines it, to reduce bind issues on rapid restart. |

### Open Ephys plugin (Red Pitaya board)

| Topic | Behavior |
|--------|----------|
| **Start handshake** | `startAcquisition()` requires a **`STARTED`** or **`STARTED …`** line from the board before `startThread()`. On failure, the command socket is closed and cleared. |
| **Stop / restart** | **`stopAcquisition()`** does **not** send `STOP\n` on the same socket while `run()` reads binary data (that corrupted framing). Order: signal thread → **close socket** → `stopThread()` → delete socket. |
| **Fresh TCP each run** | Each **`startAcquisition()`** opens a **new** TCP connection before sending `START\n`. |
| **Variable frame size** | **`run()`** reads **`bytes_per_frame`** from byte **offset 4** of each 22-byte header and reads exactly that many payload bytes, with byte sliding resync if the magic bytes (`dtype` at offset 8–9) are wrong. Maps up to the configured channel count; pads with zeros if the frame is short. |
| **MSVC** | Nested lambdas in `run()` use explicit capture / naming (`socketReadFully`, `parseHeaderBytesPerFrame` with `[=]`) so Visual Studio builds cleanly. |

### Device editor (UI layout + acquisition)

The acquisition board strip has a **fixed height**; controls must stay within roughly **y &lt; ~126** in the narrow strip or they are **clipped**.

| Control | Position (relative to `xOffset`; `xOffset` is 0, or memory-monitor width on ONI) |
|---------|-------------------------------------------------------------------------------------|
| **Sample rate** | Title `(80, 22)`, editable value **`1000`** `(80, 38)` — this is the streaming **frequency** (Hz), not a separate combo. |
| **FILTER** | Title `(155, 22)`, button **`FILTER`** `(155, 38)`. |
| **Analog In Gain** | Title `(155, 62)`, value `(155, 74)`. |
| **Analog Out (V)** | Title `(155, 94)`, value `(155, 108)`. |
| **RECORD** | **`(6, 108, 65×18)`** — must stay at this **y** so it stays visible (moving it lower hid the button). |

**During acquisition:** after `ChannelCanvas::beginAnimation()`, **`startAcquisition()`** calls **`toFront(false)`** on the sample-rate, filter, analog, and RECORD widgets so they remain **clickable** even though the canvas overlaps the right column (`x ≈ 155`).

**Note:** A trial layout that moved filter/analog/RECORD into one tall left column (**“Option A”**) was **reverted** because **RECORD** at `y = 148` fell below the visible editor area.

---

## Changes in the last week of April (detail)

### `RedPitaya_justin.c`

- Save recordings to `.bin` and `.csv`.
- After each streaming session, **drain the TCP receive buffer** on the client socket so leftover binary tail bytes are not read as the next command by the outer `read()` loop.
- **Stream sequence (`ns`):** First binary frame of each session uses sequence **0** in the header; counter increments once per frame before send (CSV row index matches the header).
- **`SO_REUSEPORT`:** On the listen socket when the OS supports it (in addition to `SO_REUSEADDR`).

### Open Ephys plugin (bullet list)

- Fixed channel layout: raw sensor, VQF/filter channels, analog waveform inputs (e.g. per sensor: 9 raw + 4 filter + 2 analog where applicable).
- Filter/VQF toggling; two analog waveform input channels; UI for record, filter, sample rate, analog in gain, analog out voltage.
- **Acquisition start validation:** `STARTED` required before reader thread; socket reset on failure.
- **Stop / restart:** Close-first teardown; new TCP per start; no in-band `STOP` on the binary stream.
- **Header-driven payload** in `run()` for fusion/frame-size changes.
- **Device editor:** Original geometry + `toFront()` during acquisition (see table above).

![[Screenshot 2026-04-27 142836 1.png]]

---

## VQF / Filter feature

The **`FILTER`** button controls whether VQF/filter values are written into the reserved VQF channels.

**When filter is on:** VQF channels contain calculated values.

**When filter is off:** VQF channels are still present and are filled with zeros.

The Red Pitaya server applies filter commands between frames when possible. **Caveat:** filter commands share the same TCP connection as the binary stream from the plugin’s perspective; the plugin and server were hardened for start/stop and framing, but toggling filter very rapidly during streaming can still stress framing—prefer toggling when idle if you see glitches.

---

## Analog output voltage control

The UI has an editable **`Analog Out (V)`** field.

**Valid range:** `0.0` to `1.8` V

**Command sent to the board:**

```
AOUT:<value>
```

The Red Pitaya server receives and logs this command. Analog output is a **command**, not a measured waveform channel; to show the real voltage on-screen you would need to sense it on an input channel.

---

# What more needs to be done

- Test analog input and output with ADC hardware.
- Fix the Makefile issue where new files using `vqf.c` or `sensor_fusion.c` must be wired in manually instead of being picked up automatically.

---

## Tests (no Open Ephys build required)

From the repo root:

```bash
cc -std=c99 -Wall -Wextra -o /tmp/rp_framing_test tests/redpitaya_stream_framing_test.c && /tmp/rp_framing_test
```

This checks variable-payload framing and header resync logic used by the plugin reader.

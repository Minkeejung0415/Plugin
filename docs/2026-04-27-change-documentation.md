
# If red pitaya does not boot

1. check if you can access the [rp-f0f85a.local](http://rp-f0f85a.local/) page
2. check if you can ssh into the red pitaya
3. check if blue and green lights are on and red light is blinking
4. If the led lights are doing fine but you can't ssh into the red pitaya, check if the red pitaya is correctly wired to the router which is connected to the PC
5. if those are fine but you can't access the red pitaya, go to terminal and use "arp -a" command and check if you can see f0f85a or f0cd35 as one of the addresses
6. If you can find them but can't access both of them check if the router is connected to the power on the wall
7. If that is also good, just unplug everything and let is rest for a bit

 
## Changes in the last week of April

### `RedPitaya_justin.c`
new changes: save recordings to .bin and .csv
- After each streaming session, the server **drains the TCP receive buffer** on the client socket so leftover binary tail bytes are not read as the next command by the outer `read()` loop.
- **Stream sequence (`ns`):** The first binary frame of each session uses sequence **0** in the header; the counter increments once per frame before send (CSV row index matches the header).
- **`SO_REUSEPORT`:** Set on the listen socket when the OS supports it (in addition to `SO_REUSEADDR`) to reduce bind issues during rapid restart.

# Open Ephys Plugin Changes

- Added fixed channel layout support for raw sensor data, VQF/filter channels, and analog waveform inputs. ex) sensor = 1 : 9 (raw) + 4 (filter) + 2 (analog input)
- Added filter/VQF toggling during acquisition. 
- Added two analog waveform input channels.
- Added UI controls for recording, filtering, sample rate, analog input gain, and analog output voltage.
- **Acquisition start validation (Red Pitaya plugin):** `startAcquisition()` now requires a `STARTED` (or `STARTED …`) line from the board before starting the reader thread. Empty, timed-out, or unexpected replies no longer return success; the command socket is closed so the next start gets a clean connection. This addresses the issue where acquisition sometimes needed a stop and second start to work.
- **Stop / restart (stale stream):** `stopAcquisition()` no longer sends `STOP\n` on the same socket while `run()` may still be parsing binary frames (that could corrupt framing and leave garbage for the next session). Teardown is **close socket → join thread → delete**. Each `startAcquisition()` opens a **new TCP connection** before `START\n` so the board always starts from a clean session.
- **Variable payload per packet:** `run()` now uses the **22-byte header’s `bytes_per_frame`** (field at offset 4) for each TCP read instead of assuming `numAdcChannels * 2`. That keeps the client aligned when the server changes frame size (e.g. filter/fusion toggling changes channel count). Extra samples in a frame are truncated to the Open Ephys channel count; missing channels are zero-filled.
- **Device editor layout (Red Pitaya):** RECORD and sample rate use their **original** positions (`RECORD` at `y=108`; filter/analog in the `x=155` column). **Option A** (stacking everything in the left column) was reverted because the fixed editor height **clips** widgets below ~`y=126`, which hid **RECORD**. While acquisition is active, `startAcquisition()` still calls **`toFront()`** on the control strip so filter/analog stay clickable over the channel canvas.

![[Screenshot 2026-04-27 142836 1.png]]


## VQF / Filter Feature

*The `FILTER` button controls whether VQF/filter values are written into the reserved VQF channels.*

*When filter is on:*

- *VQF channels are filled with calculated values.*

*When filter is off:*

- *VQF channels are still present;*
- *VQF channels are filled with zeros.*

*The filter can be changed while data is being collected. The Red Pitaya server checks for filter commands before building the next frame, so the next frame uses the latest filter state.*

## *Analog Output Voltage Control*

*The UI has an editable `Analog Out (V)` field.*

*Valid range:*

```
0.0 to 1.8 V
```

*When changed, Open Ephys sends:*

```
AOUT:<value>
```

*The Red Pitaya server currently receives and logs this command.*

*Analog output is a command, not a measured waveform channel. If the physical output voltage needs to be displayed, it must be measured back through an input channel.*

# What more needs to be done

- test analog input and output with ADC 
- fix the Makefile issue where when the new code file uses the vqf.c or sensor_fusion.c it has to be manually implemented in the makefile and not automatically dealt with

## Tests (no Open Ephys build required)

From the repo root, run the TCP framing self-test:

```bash
cc -std=c99 -Wall -Wextra -o /tmp/rp_framing_test tests/redpitaya_stream_framing_test.c && /tmp/rp_framing_test
```

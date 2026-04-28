# Red Pitaya Open Ephys Plugin Changes

This document gives a basic overview of what changed in the Red Pitaya/Open Ephys plugin work and what the main functions/features now do.

## What Changed

The plugin was updated to use a Red Pitaya as a network-based acquisition source for Open Ephys.

The main changes are:

- Added a Red Pitaya acquisition board implementation.
- Added a Red Pitaya TCP server program.
- Added sensor discovery for AXI I2C and SPI sensors.
- Added binary sample streaming from Red Pitaya to Open Ephys.
- Added `.bin` and `.csv` recording on the Red Pitaya.
- Added fixed channel layout support for raw sensor data, VQF/filter channels, and analog waveform inputs.
- Added live filter/VQF toggling during acquisition.
- Added two analog waveform input channels.
- Added UI controls for recording, filtering, sample rate, analog input gain, and analog output voltage.
- Updated Red Pitaya channels so LFP Viewer can display them.
- Fixed startup crashes caused by restoring old audio/electrode button settings.

## Main Files

### `RedPitaya_justin.c`

Runs on the Red Pitaya.

Main responsibilities:

- Detect sensors connected through AXI I2C/SPI.
- Read raw sensor values.
- Run optional VQF/filter processing.
- Read Red Pitaya analog input waveform channels.
- Build stream frames.
- Send frames to Open Ephys.
- Save recordings to `.bin` and `.csv`.
- Receive commands from Open Ephys.

### `acqboard.ccp`

Runs inside the Open Ephys plugin.

Main responsibilities:

- Connect to the Red Pitaya server.
- Detect whether the Red Pitaya is available.
- Read the channel count from the server.
- Start and stop acquisition.
- Read streamed samples into the Open Ephys data buffer.
- Send control commands such as filter, record, sample rate, analog gain, and analog output voltage.

### `Acqboardredpitaya.h`

Defines the Red Pitaya acquisition board class.

Important state:

- number of streamed channels;
- sample buffers;
- last recording paths;
- filter state;
- analog input gain;
- analog output voltage;
- Red Pitaya command socket.

### `devicethread.cpp`

Connects the acquisition board to Open Ephys.

Main responsibilities:

- Detect the active board.
- Initialize the board.
- Build Open Ephys stream metadata.
- Create continuous channels for display.
- Mark Red Pitaya channels as ephys/electrode-style channels so LFP Viewer can display them.

### `device editor.cpp` / `device editor.h`

Defines the Open Ephys editor UI.

Main controls:

- `RECORD`
- `FILTER`
- sample rate input
- analog input gain input
- analog output voltage input

## Red Pitaya Detection

Open Ephys connects to the Red Pitaya at:

```text
rp-f0f85a.local:5000
```

The plugin sends:

```text
REDPITAYA
```

The Red Pitaya replies:

```text
OK CHANNELS:<count>
```

The `<count>` value tells Open Ephys how many continuous channels to create.

## Channel Layout

The channel count is dynamic. It depends on detected sensors.

The layout is:

```text
raw sensor channels + VQF channels + analog waveform channels
```

More specifically:

```text
total channels = raw sensor channels + (number of sensors * 4) + analog waveform channels
```

Each sensor gets four VQF channels:

```text
qw, qx, qy, qz
```

The plugin currently adds two analog waveform channels:

```text
AnalogInput1
AnalogInput2
```

Examples:

```text
1 sensor with 9 raw channels:
9 + 4 + 2 = 15 channels

2 sensors with 9 raw channels each:
9 + 9 + 4 + 4 + 2 = 28 channels

1 sensor with 6 raw channels:
6 + 4 + 2 = 12 channels
```

The channel layout stays the same while streaming. This is important because Open Ephys expects the number of channels to stay stable during acquisition.

## VQF / Filter Feature

The `FILTER` button controls whether VQF/filter values are written into the reserved VQF channels.

When filter is on:

- VQF channels are filled with calculated values.

When filter is off:

- VQF channels are still present;
- VQF channels are filled with zeros.

The filter can be changed while data is being collected. The Red Pitaya server checks for filter commands before building the next frame, so the next frame uses the latest filter state.

Commands:

```text
FILTER ON
FILTER OFF
```

## Analog Waveform Inputs

The stream now includes two Red Pitaya analog input waveform channels:

```text
AnalogInput1
AnalogInput2
```

These are read from Red Pitaya input channels:

```text
RP_CH_1
RP_CH_2
```

The server uses the Red Pitaya acquisition API:

```c
rp_AcqGetLatestDataRaw(...)
```

If analog acquisition cannot initialize, these channels still exist but are filled with zeros.

## Analog Input Gain Control

The UI has an editable `Analog In Gain` field.

Valid range:

```text
0.1 to 100.0
```

When changed, Open Ephys sends:

```text
AIN_GAIN:<value>
```

The Red Pitaya server currently receives and logs this command.

## Analog Output Voltage Control

The UI has an editable `Analog Out (V)` field.

Valid range:

```text
0.0 to 1.8 V
```

When changed, Open Ephys sends:

```text
AOUT:<value>
```

The Red Pitaya server currently receives and logs this command.

Analog output is a command, not a measured waveform channel. If the physical output voltage needs to be displayed, it must be measured back through an input channel.

## Recording Feature

The Red Pitaya saves recordings locally under:

```text
/root/Measurements
```

Each recording creates:

```text
recording_YYYYMMDD_HHMMSS.bin
recording_YYYYMMDD_HHMMSS.csv
```

The `.bin` file stores raw binary sample frames.

The `.csv` file stores readable sample rows with:

- sample index;
- elapsed time;
- raw sensor channels;
- VQF channels;
- analog waveform channels.

The Red Pitaya sends the recording paths back to Open Ephys:

```text
STARTED BIN:<bin path> CSV:<csv path>
```

Open Ephys shows status messages with those paths.

## Runtime Commands

Open Ephys can send these commands to the Red Pitaya:

```text
REDPITAYA
START
STOP
RECORD ON
RECORD OFF
FILTER ON
FILTER OFF
FUSION ON
FUSION OFF
FREQ:<value>
AIN_GAIN:<value>
AOUT:<value>
```

## LFP Viewer Support

Red Pitaya channels are exposed to Open Ephys as ephys/electrode-style continuous channels.

This allows LFP Viewer to display the incoming voltage waves.

## Startup Crash Fix

The plugin previously crashed while loading saved XML parameters because old audio/electrode button settings were restored even when those UI buttons were not present.

The fix removed that old restore path and added guard checks so missing buttons are not accessed.


/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2024 Open Ephys

    ------------------------------------------------------------------

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "DeviceThread.h"
#include "DeviceEditor.h"

#include "devices/oni/AcqBoardONI.h"
#include "devices/opalkelly/AcqBoardOpalKelly.h"
#include "devices/redpitaya/AcqBoardRedPitaya.h"
#include "devices/simulated/AcqBoardSim.h"

/*
    forceSimulationMode — set to true when you want to test the plugin UI and
    signal-chain wiring without real hardware attached. When true, detectBoard()
    skips all hardware probes and immediately creates an AcqBoardSim that
    generates synthetic sine-wave data. Useful for CI testing or UI development.
    Leave false for normal lab use.
*/
static const bool forceSimulationMode = false;

BoardType DeviceThread::boardType = ACQUISITION_BOARD; // initialize static member

DeviceThread::DeviceThread (SourceNode* sn, BoardType boardType_) : DataThread (sn)
{
    boardType = boardType_;

    sourceBuffers.add (new DataBuffer (2, 10000)); // start with 2 channels and automatically resize
    acquisitionBoard.reset (detectBoard()); // detect which board is connected

    if (acquisitionBoard == nullptr) // no board detected, and not running in simulation mode
    {
        deviceFound = false;
        return;
    }

    deviceFound = acquisitionBoard->initializeBoard(); // returns false if initialization fails

    if (! deviceFound)
        return;

    acquisitionBoard->scanPorts(); // check for connected headstages
}

DeviceThread::~DeviceThread()
{
}

DataThread* DeviceThread::createDataThread (SourceNode* sn)
{
    return new DeviceThread (sn, boardType);
}

std::unique_ptr<GenericEditor> DeviceThread::createEditor (SourceNode* sn)
{
    std::unique_ptr<DeviceEditor> editor = std::make_unique<DeviceEditor> (sn, acquisitionBoard.get());

    return editor;
}

/*
    detectBoard() — the hardware detection waterfall.

    Works like checking each drawer for a key: tries each board type in order
    and returns the first one that answers. The order matters — faster/more
    common boards are tried first to avoid unnecessary delays.

    Waterfall order:
      1. OpalKelly (USB FPGA) — classic Intan RHD2000 board, tried first
      2. ONI (Open Neuro Interface) — newer USB3 platform
      3. RedPitaya (TCP handshake to rp-*.local or a configured IP) — our wireless IMU board
      4. If none found: asks the user whether to run in simulation mode

    Returns a heap-allocated AcquisitionBoard* (caller takes ownership), or nullptr
    if the user declines simulation mode.
*/
AcquisitionBoard* DeviceThread::detectBoard()
{
    // Developer override: skip all hardware and use the fake board immediately
    if (forceSimulationMode)
    {
        return new AcqBoardSim();
    }

    // --- Try OpalKelly USB FPGA board (classic Intan RHD2000 hardware) ---
    std::unique_ptr<AcqBoardOpalKelly> opalKellyBoard = std::make_unique<AcqBoardOpalKelly>();

    if (opalKellyBoard->detectBoard())
    {
        return opalKellyBoard.release(); // found it — hand ownership to caller
    }
    else
    {
        opalKellyBoard.reset(); // not found — clean up before trying next type
    }

    // --- Try ONI (Open Neuro Interface) USB3 board ---
    std::unique_ptr<AcqBoardONI> oniBoard = std::make_unique<AcqBoardONI>();

    if (oniBoard->detectBoard())
    {
        return oniBoard.release();
    }
    else
    {
        oniBoard.reset();
    }

    // --- Try Red Pitaya / ESP32-S3 STEP node (TCP handshake on port 5000) ---
    // AcqBoardRedPitaya::detectBoard() tries rp-*.local mDNS names first,
    // then falls back to any user-configured IP from the nodeHostLabel.
    std::unique_ptr<AcqBoardRedPitaya> redPitayaBoard = std::make_unique<AcqBoardRedPitaya>();

    if (redPitayaBoard->detectBoard())
    {
        return redPitayaBoard.release();
    }
    else
    {
        redPitayaBoard.reset();
    }

    // --- No hardware found — ask the user if they want a fake board ---
    bool response = AlertWindow::showOkCancelBox (AlertWindow::NoIcon,
                                                  "No device found.",
                                                  "An acquisition board could not be found. Do you want to run this plugin in simulation mode?",
                                                  "Yes",
                                                  "No",
                                                  0,
                                                  0);

    if (response)
    {
        return new AcqBoardSim();
    }

    // if we reach this point, we have no device connected
    return nullptr;
}

bool DeviceThread::foundInputSource()
{
    return deviceFound;
}

bool DeviceThread::isReady()
{
    return acquisitionBoard->isReady();
}

bool DeviceThread::startAcquisition()
{
    return acquisitionBoard->startAcquisition();
}

bool DeviceThread::stopAcquisition()
{
    return acquisitionBoard->stopAcquisition();
}

void DeviceThread::updateSettings (OwnedArray<ContinuousChannel>* continuousChannels,
                                   OwnedArray<EventChannel>* eventChannels,
                                   OwnedArray<SpikeChannel>* spikeChannels,
                                   OwnedArray<DataStream>* sourceStreams,
                                   OwnedArray<DeviceInfo>* devices,
                                   OwnedArray<ConfigurationObject>* configurationObjects)
{
    if (! deviceFound)
        return;

    continuousChannels->clear();
    eventChannels->clear();
    spikeChannels->clear();
    sourceStreams->clear();
    devices->clear();
    configurationObjects->clear();
    sourceBuffers.clear();

    if (acquisitionBoard->getNumChannels() > 0)
    {
        sourceBuffers.add (acquisitionBoard->getBuffer());

        bool generatesTimestamps = acquisitionBoard->getBoardType() == AcquisitionBoard::BoardType::ONI;

        DataStream::Settings dataStreamSettings {
            "acquisition_board",
            "Continuous and event data from an Open Ephys Acquisition Board",
            "acq-board.rhythm",
            static_cast<float> (acquisitionBoard->getSampleRate()),
            generatesTimestamps
        };

        DataStream* stream = new DataStream (dataStreamSettings);

        sourceStreams->add (stream);

        for (auto headstage : acquisitionBoard->getHeadstages())
        {
            for (int ch = 0; ch < headstage->getNumActiveChannels(); ch++)
            {
                ContinuousChannel::Settings channelSettings {
                    ContinuousChannel::ELECTRODE,
                    headstage->getChannelName (ch),
                    "Headstage channel from an Open Ephys Acquisition Board",
                    "acq-board.rhythm.continuous.ephys",

                    acquisitionBoard->getBitVolts (ContinuousChannel::Type::ELECTRODE),

                    stream
                };

                continuousChannels->add (new ContinuousChannel (channelSettings));
                continuousChannels->getLast()->setUnits ("uV");

                if (headstage->hasValidImpedance (ch))
                {
                    continuousChannels->getLast()->impedance.magnitude = headstage->getImpedanceMagnitude (ch);
                    continuousChannels->getLast()->impedance.phase = headstage->getImpedancePhase (ch);
                }
            }
        }

        if (acquisitionBoard->areAuxChannelsEnabled())
        {
            const ContinuousChannel::InputRange accelerationRange { -100.0f, 100.0f };

            for (auto headstage : acquisitionBoard->getHeadstages())
            {
                for (int ch = 0; ch < 3; ch++)
                {
                    ContinuousChannel::Settings channelSettings {
                        ContinuousChannel::AUX,
                        headstage->getStreamPrefix() + "_AUX" + String (ch + 1),
                        "Aux input channel from an Open Ephys Acquisition Board",
                        "acq-board.rhythm.continuous.aux",

                        acquisitionBoard->getBitVolts (ContinuousChannel::Type::AUX),

                        stream
                    };

                    continuousChannels->add (new ContinuousChannel (channelSettings));
                    continuousChannels->getLast()->setUnits ("mV");
                    continuousChannels->getLast()->inputRange = accelerationRange;
                }
            }
        }

        /*
            ADC channel naming differs between board types:
              - ESP32-S3 STEP node: 13 fixed-purpose channels with semantic names
                (ax, ay, az = accelerometer; gx, gy, gz = gyroscope;
                mx, my, mz = magnetometer; qw, qx, qy, qz = VQF quaternion output).
                These come pre-defined in esp32Names[].
              - Red Pitaya: many raw IMU channels named "Channel N", with the last two
                renamed "AnalogInput1 / AnalogInput2" for the waveform capture channels.
              - All other boards (OpalKelly, ONI): plain "Channel N" ADC naming.
        */
        if (acquisitionBoard->getBoardType() == AcquisitionBoard::BoardType::RedPitaya
            && acquisitionBoard->areAdcChannelsEnabled())
        {
            const int numadcchannels = acquisitionBoard->getNumDataOutputs (ContinuousChannel::ADC);
            auto* rpBoard = dynamic_cast<AcqBoardRedPitaya*> (acquisitionBoard.get());
            const bool esp32Layout = (rpBoard != nullptr && rpBoard->getIsEsp32Node());

            for (int ch = 0; ch < numadcchannels; ch++)
            {
                String name;

                if (esp32Layout)
                {
                    // ESP32-S3 STEP node has 14 columns with fixed meanings:
                    // ax/ay/az = accel (g), gx/gy/gz = gyro (dps),
                    // mx/my/mz = magnetometer (uT), qw/qx/qy/qz = VQF quaternion,
                    // dio = digital input state
                    static const char* esp32Names[] = {
                        "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz", "qw", "qx", "qy", "qz", "dio"
                    };
                    name = (ch < (int) (sizeof (esp32Names) / sizeof (esp32Names[0])))
                               ? esp32Names[ch]
                               : ("ch" + String (ch + 1));
                }
                else
                {
                    // Red Pitaya: last ANALOG_WAVEFORM_CHANNELS are the analog voltage inputs;
                    // all preceding channels are IMU data named sequentially.
                    const int analogStartChannel = jmax (0, numadcchannels - AcqBoardRedPitaya::ANALOG_WAVEFORM_CHANNELS);
                    name = ch >= analogStartChannel
                               ? "AnalogInput" + String (ch - analogStartChannel + 1)
                               : "Channel" + String (ch + 1);
                }

                ContinuousChannel::Settings channelSettings {
                    ContinuousChannel::ELECTRODE,
                    name,
                    "Red Pitaya input channel",
                    "acq-board.rhythm.continuous.ephys",

                    acquisitionBoard->getBitVolts (ContinuousChannel::Type::ADC),

                    stream
                };

                continuousChannels->add (new ContinuousChannel (channelSettings));
                continuousChannels->getLast()->setUnits ("uV");
            }
        }
        else if (acquisitionBoard->areAdcChannelsEnabled())
        {
            const int numadcchannels = acquisitionBoard->getNumDataOutputs (ContinuousChannel::ADC);
            for (int ch = 0; ch < numadcchannels; ch++)
            {
                String name = "Channel" + String (ch + 1);

                ContinuousChannel::Settings channelSettings {
                    ContinuousChannel::ADC,
                    name,
                    "ADC input channel from an Open Ephys Acquisition Board",
                    "acq-board.rhythm.continuous.adc",

                    acquisitionBoard->getBitVolts (ContinuousChannel::Type::ADC),

                    stream
                };

                continuousChannels->add (new ContinuousChannel (channelSettings));
                continuousChannels->getLast()->setUnits ("");
            }
        }

        EventChannel::Settings settings {
            EventChannel::Type::TTL,
            "Acquisition Board TTL Input",
            "Events on digital input lines of an Open Ephys Acquisition Board",
            "acq-board.rhythm.events",
            stream,
            8
        };

        eventChannels->add (new EventChannel (settings));
    }

    OwnedArray<DataStream> otherStreams;
    OwnedArray<ContinuousChannel> otherChannels;
    OwnedArray<DataBuffer> otherBuffers;
    acquisitionBoard->createCustomStreams (otherBuffers);
    sourceBuffers.addArray (otherBuffers);
    otherBuffers.clearQuick (false);
    acquisitionBoard->updateCustomStreams (otherStreams, otherChannels);
    sourceStreams->addArray (otherStreams);
    otherStreams.clearQuick (false);
    continuousChannels->addArray (otherChannels);
    otherChannels.clearQuick (false);
}

/*
    handleBroadcastMessage() — receives inter-plugin broadcast messages.

    Other Open Ephys plugins (e.g. a stimulus controller or a keyboard-shortcut
    plugin) can send text messages to all other plugins via CoreServices::sendStatusMessage.
    We listen for the "ACQBOARD TRIGGER <line> <ms>" message format.

    What "ACQBOARD TRIGGER" does:
      1. Fires a TTL pulse on the specified digital output line for <ms> milliseconds.
         This creates a real electrical pulse on the hardware BNC output, used to
         synchronise external equipment.
      2. If the board is a Red Pitaya, also calls writeJointDisplayConfig() so the
         Python OpenSim visualizer (opensim_live_realtime.py) immediately updates its
         HUD to show only the operator's pre-selected joint angles.

    Message format: "ACQBOARD TRIGGER <line> <durationMs>"
      <line>       = TTL output line number, 1-based (UI labels from 1; we subtract 1
                     here because internal hardware indexing starts at 0)
      <durationMs> = pulse width in milliseconds (clamped 10–5000 ms)
*/
void DeviceThread::handleBroadcastMessage (const String& msg, const int64 messageTimeMilliseconds)
{
    StringArray parts = StringArray::fromTokens (msg, " ", "");

    if (parts[0].equalsIgnoreCase ("ACQBOARD"))
    {
        if (parts.size() > 1)
        {
            String command = parts[1];

            if (command.equalsIgnoreCase ("TRIGGER"))
            {
                if (parts.size() == 4)
                {
                    // UI shows lines as 1-8; hardware wants 0-7 — subtract 1 to convert
                    int ttlLine = parts[2].getIntValue() - 1;

                    if (ttlLine < 0 || ttlLine > 7)
                        return; // ignore out-of-range line numbers

                    int eventDurationMs = parts[3].getIntValue();

                    // Sanity check: pulse must be at least 10 ms (reliable hardware timing)
                    // and at most 5 seconds (anything longer is probably a bug)
                    if (eventDurationMs < 10 || eventDurationMs > 5000)
                        return;

                    acquisitionBoard->triggerDigitalOutput (ttlLine, eventDurationMs);

                    // dynamic_cast safely checks if the board is a RedPitaya at runtime.
                    // If it is not a RedPitaya, the cast returns nullptr and we skip this.
                    // If it IS a RedPitaya, rp points to the real object and we can call
                    // Red-Pitaya-specific methods that don't exist on AcquisitionBoard.
                    if (auto* rp = dynamic_cast<AcqBoardRedPitaya*> (acquisitionBoard.get()))
                        rp->writeJointDisplayConfig(); // snapshot joint selection to JSON for Python
                }
            }
        }
    }
}

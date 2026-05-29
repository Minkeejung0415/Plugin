/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI

    ------------------------------------------------------------------
*/

#include "Acqboardredpitaya.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace
{
    const char* kOpenSimWorkDir = "C:\\Users\\KIN Student\\Open-Sim--Bio-Mech";
}

AcqBoardRedPitaya::AcqBoardRedPitaya()
    : AcquisitionBoard()
{
    boardType = BoardType::RedPitaya;

    // Default sample rate; can be overridden with setSampleRate().
    settings.boardSampleRate = 1000.0f;

    // By default, expose ADC channels only.
    acquireAdc = true;
    acquireAux = false;
}

AcqBoardRedPitaya::~AcqBoardRedPitaya()
{
    stopAcquisition();
}
/*
bool AcqBoardRedPitaya::detectBoard()
{
    std::cout << "detectBoard called" << std::endl;

    deviceFound = false;
    StreamingSocket socket;

    // Use a slightly larger timeout for discovery since the RP might be scanning AXI
    if (! socket.connect ("rp-f0f85a.local", 5000, 1000))
    {
        std::cout << "connect failed" << std::endl;
        return false;
    }

    std::cout << "connected" << std::endl;

    const char* msg = "REDPITAYA\n";
    socket.write (msg, (int) strlen (msg));

    if (! socket.waitUntilReady (true, 500))
    {
        std::cout << "no reply" << std::endl;
        socket.close();
        return false;
    }

    // Increased buffer size to catch "OK CHANNELS:XX"
    char buffer[64] = { 0 };
    int n = socket.read (buffer, sizeof (buffer) - 1, false);

    if (n > 0)
    {
        buffer[n] = '\0';
        String response (buffer);
        std::cout << "Received response: " << response << std::endl;

        // Check for the "OK" flag
        if (response.contains ("OK"))
        {
            // Parse the channel count
            if (response.contains ("CHANNELS:"))
            {
                int startIdx = response.indexOf ("CHANNELS:") + 9;

                // substring(startIdx) gets everything after the colon
                // getIntValue() stops parsing at the newline/space
                this->numAdcChannels = response.substring (startIdx).getIntValue();

                std::cout << "Detected Red Pitaya with " << numAdcChannels << " channels." << std::endl;
                deviceFound = true;
            }
            else
            {
                // Fallback for legacy code if needed
                this->numAdcChannels = 6;
                deviceFound = true;
            }
        }
    }

    socket.close();
    return deviceFound;
} */

bool AcqBoardRedPitaya::detectBoard()
{
    std::cout << "detectBoard called" << std::endl;

    deviceFound = false;

    if (commandSocket != nullptr && ! commandSocket->isConnected())
    {
        delete commandSocket;
        commandSocket = nullptr;
    }

    if (commandSocket == nullptr)
        commandSocket = new StreamingSocket();

    if (! commandSocket->connect ("rp-f0f85a.local", 5000, 1000))
    {
        std::cout << "connect failed" << std::endl;
        delete commandSocket;
        commandSocket = nullptr;
        return false;
    }

    std::cout << "connected" << std::endl;

    const char* msg = "REDPITAYA\n";
    commandSocket->write (msg, (int) strlen (msg));

    if (! commandSocket->waitUntilReady (true, 500))
    {
        std::cout << "no reply" << std::endl;
        commandSocket->close();
        delete commandSocket;
        commandSocket = nullptr;
        return false;
    }

    // Increased buffer size to catch "OK CHANNELS:XX"
    char buffer[64] = { 0 };
    int n = commandSocket->read (buffer, sizeof (buffer) - 1, false);

    if (n > 0)
    {
        buffer[n] = '\0';
        String response (buffer);
        std::cout << "Received response: " << response << std::endl;

        // Check for the "OK" flag
        if (response.contains ("OK"))
        {
            // Parse the channel count
            if (response.contains ("CHANNELS:"))
            {
                int startIdx = response.indexOf ("CHANNELS:") + 9;

                this->numAdcChannels = response.substring (startIdx).getIntValue();

                std::cout << "Detected Red Pitaya with " << numAdcChannels << " channels." << std::endl;
                deviceFound = true;
            }
            else
            {
                // Fallback for legacy code if needed
                this->numAdcChannels = 6;
                deviceFound = true;
            }
        }
    }
    if (! deviceFound)
    {
        commandSocket->close();
        delete commandSocket;
        commandSocket = nullptr;
    }

    return deviceFound;
}

bool AcqBoardRedPitaya::initializeBoard()
{
    if (! deviceFound)
        return false;

    // Any Red Pitaya-specific configuration (e.g. network setup) can be
    // performed here. At this stage we simply consider the device ready.

    return true;
}

bool AcqBoardRedPitaya::foundInputSource() const
{
    return deviceFound;
}

Array<const Headstage*> AcqBoardRedPitaya::getHeadstages()
{
    // Red Pitaya is modeled as an ADC-only device with no headstages.
    Array<const Headstage*> none;
    return none;
}

Array<int> AcqBoardRedPitaya::getAvailableSampleRates()
{
    Array<int> sampleRates;

    sampleRates.add (100);
    sampleRates.add (250);
    sampleRates.add (500);
    sampleRates.add (1000);
    sampleRates.add (2000);

    return sampleRates;
}

void AcqBoardRedPitaya::setSampleRate (int sampleRateHz)
{
    settings.boardSampleRate = static_cast<float> (sampleRateHz);
}

float AcqBoardRedPitaya::getSampleRate() const
{
    return settings.boardSampleRate;
}

void AcqBoardRedPitaya::scanPorts()
{
    // No-op: there are no headstages to scan for Red Pitaya.
}

void AcqBoardRedPitaya::enableAuxChannels (bool enabled)
{
    acquireAux = enabled;
}

bool AcqBoardRedPitaya::areAuxChannelsEnabled() const
{
    return acquireAux;
}

void AcqBoardRedPitaya::enableAdcChannels (bool enabled)
{
    acquireAdc = enabled;
}

bool AcqBoardRedPitaya::areAdcChannelsEnabled() const
{
    return acquireAdc;
}

float AcqBoardRedPitaya::getBitVolts (ContinuousChannel::Type channelType) const
{
    if (channelType == ContinuousChannel::ADC)
        return adcBitVolts;

    return 1.0f;
}

void AcqBoardRedPitaya::measureImpedances()
{
}

void AcqBoardRedPitaya::impedanceMeasurementFinished()
{
}

void AcqBoardRedPitaya::saveImpedances (File& /*file*/)
{
}

void AcqBoardRedPitaya::setNamingScheme (ChannelNamingScheme scheme)
{
    channelNamingScheme = scheme;
}

ChannelNamingScheme AcqBoardRedPitaya::getNamingScheme()
{
    return channelNamingScheme;
}

bool AcqBoardRedPitaya::isReady()
{
    return deviceFound;
}

bool AcqBoardRedPitaya::startAcquisition()
{
    if (! deviceFound)
        return false;

    streamSensorNames.clear();

    if (isThreadRunning())
    {
        std::cout << "Red Pitaya WARNING: previous reader thread still running; stopping before restart." << std::endl;
        signalThreadShouldExit();

        if (commandSocket != nullptr)
            commandSocket->close();

        stopThread (2000);

        if (isThreadRunning())
        {
            std::cout << "Red Pitaya ERROR: previous reader thread did not stop; refusing to start a new acquisition." << std::endl;
            return false;
        }
    }

    // Always use a new TCP session for each acquisition so the board's command
    // loop never inherits stale RX data from a previous stream (same issue class
    // as sending STOP while our reader thread still shares this socket).
    if (commandSocket != nullptr)
    {
        commandSocket->close();
        delete commandSocket;
        commandSocket = nullptr;
    }

    commandSocket = new StreamingSocket();
    if (! commandSocket->connect ("rp-f0f85a.local", 5000, 1000))
    {
        std::cout << "Red Pitaya ERROR: Could not connect to board." << std::endl;
        delete commandSocket;
        commandSocket = nullptr;
        return false;
    }

    {
        char freqMsg[32];
        int targetHz = jlimit (1, 2000, (int) settings.boardSampleRate);
        snprintf (freqMsg, sizeof (freqMsg), "FREQ:%d\n", targetHz);
        commandSocket->write (freqMsg, (int) strlen (freqMsg));
    }

    const char* msg = "START\n";
    commandSocket->write (msg, (int) strlen (msg));

    lastRecordingPath = {};
    lastRecordingCsvPath = {};

    String responseText;

    while (commandSocket->waitUntilReady (true, 1000))
    {
        char c = 0;
        if (commandSocket->read (&c, 1, false) <= 0 || c == '\n')
            break;

        responseText += c;
    }

    auto failStartAndResetSocket = [this]()
    {
        if (commandSocket != nullptr)
        {
            commandSocket->close();
            delete commandSocket;
            commandSocket = nullptr;
        }
    };

    if (responseText.startsWith ("ERROR_FILE"))
    {
        std::cout << "Red Pitaya ERROR: Server could not open recording files." << std::endl;
        failStartAndResetSocket();
        return false;
    }

    if (! (responseText == "STARTED" || responseText.startsWith ("STARTED ")))
    {
        std::cout << "Red Pitaya ERROR: Expected STARTED from board, got: "
                  << (responseText.isEmpty() ? String ("(empty / timeout)") : responseText) << std::endl;
        failStartAndResetSocket();
        return false;
    }

    if (responseText == "STARTED")
    {
        std::cout << "Red Pitaya: Streaming started." << std::endl;
    }
    else
    {
        const String pathText = responseText.fromFirstOccurrenceOf ("STARTED ", false, false).trim();

        if (pathText.contains ("BIN:") && pathText.contains (" CSV:"))
        {
            lastRecordingPath = pathText.fromFirstOccurrenceOf ("BIN:", false, false)
                                  .upToFirstOccurrenceOf (" CSV:", false, false)
                                  .trim();
            lastRecordingCsvPath = pathText.fromFirstOccurrenceOf (" CSV:", false, false).trim();
        }
        else
        {
            lastRecordingPath = pathText;
        }

        std::cout << "Red Pitaya: Streaming to " << lastRecordingPath;
        if (lastRecordingCsvPath.isNotEmpty())
            std::cout << " and " << lastRecordingCsvPath;
        std::cout << std::endl;
    }

    // Second line: SENSORS:0,Name;1,Name2 (snapshot at stream start)
    {
        String sensorsLine;
        while (commandSocket->waitUntilReady (true, 1000))
        {
            char c = 0;
            if (commandSocket->read (&c, 1, false) <= 0 || c == '\n')
                break;
            sensorsLine += c;
        }

        if (sensorsLine.startsWith ("SENSORS:"))
        {
            const String body = sensorsLine.fromFirstOccurrenceOf ("SENSORS:", false, false).trim();
            StringArray segments;
            segments.addTokens (body, ";", "");

            for (int si = 0; si < segments.size(); ++si)
            {
                const String seg = segments[si].trim();
                const int comma = seg.indexOfChar (',');

                if (comma > 0)
                    streamSensorNames.add (seg.substring (comma + 1).trim());
                else if (seg.isNotEmpty())
                    streamSensorNames.add (seg);
            }
        }
    }

    // Sync current filter state to server before data starts flowing.
    // The UI may have toggled the filter while acquisition was stopped, so we
    // always send the authoritative state here regardless of what the server assumes.
    {
        const char* filterMsg = filterEnabled ? "FILTER ON\n" : "FILTER OFF\n";
        commandSocket->write (filterMsg, (int) strlen (filterMsg));
    }

    startThread();
    return true;
}

bool AcqBoardRedPitaya::stopAcquisition()
{
    // 1. Ask run() to exit on its next poll.
    if (isThreadRunning())
        signalThreadShouldExit();

    // 2. Close the TCP connection. The Red Pitaya server then sees EOF /
    //    send failure and leaves run_stream; we must NOT write STOP (or anything)
    //    on this socket while run() is still consuming the same byte stream as
    //    binary packets — that corrupts framing and leaves stale bytes for the
    //    next START (classic "second start works" failure).
    if (commandSocket != nullptr)
        commandSocket->close();

    // 3. Wait for run() to finish before deleting the socket object.
    //    The backend can keep streaming if the old reader/socket lifecycle does
    //    not complete cleanly before Open Ephys starts a new acquisition.
    stopThread (2000);

    if (isThreadRunning())
    {
        std::cout << "Red Pitaya WARNING: reader thread did not stop; keeping socket object alive." << std::endl;
        return false;
    }

    if (commandSocket != nullptr)
    {
        delete commandSocket;
        commandSocket = nullptr;
    }

    openSimEnabled = false;
    openSimSocket.reset();

    if (openSimLiveProcess != nullptr)
    {
        openSimLiveProcess->kill();
        openSimLiveProcess.reset();
    }

    streamSensorNames.clear();

    return true;
}
bool AcqBoardRedPitaya::sendRecordOnCommand()
{
    if (commandSocket == nullptr)
        return false;

    String msg;

    if (lastRecordingPath.isNotEmpty())
    {
        msg = "RECORD ON\n";
    }
    else
    {
        const String basePath = "/root/Measurements/recording_" + String (Time::currentTimeMillis());
        lastRecordingPath = basePath + ".bin";
        lastRecordingCsvPath = basePath + ".csv";
        msg = "RECORD ON " + basePath + "\n";
    }

    int written = commandSocket->write (msg.toRawUTF8(), (int) msg.getNumBytesAsUTF8());

    if (written <= 0)
        return false;

    return true;
}

bool AcqBoardRedPitaya::sendRecordOffCommand()
{
    if (commandSocket == nullptr)
        return false;

    const char* msg = "RECORD OFF\n";

    int written = commandSocket->write (msg, (int) strlen (msg));

    if (written <= 0)
        return false;

    return true;
}

void AcqBoardRedPitaya::updateSampleFrequency (int newFreq)
{
    if (! deviceFound)
        return;

    int targetHz = jlimit (1, 2000, newFreq);
    settings.boardSampleRate = static_cast<float> (targetHz);

    // 1. SELF-HEALING: Trash dead sockets
    if (commandSocket != nullptr && ! commandSocket->isConnected())
    {
        commandSocket->close();
        delete commandSocket;
        commandSocket = nullptr;
    }

    // 2. SELF-HEALING: Build a new connection if needed
    if (commandSocket == nullptr)
    {
        commandSocket = new StreamingSocket();

        if (! commandSocket->connect ("rp-f0f85a.local", 5000, 1000))
        {
            std::cout << "Red Pitaya ERROR: Could not connect to board." << std::endl;
            delete commandSocket;
            commandSocket = nullptr;
            return;
        }
    }

    // 3. FIRE AND FORGET
    char msg[32];
    snprintf (msg, sizeof (msg), "FREQ:%d\n", targetHz);

    int written = commandSocket->write (msg, (int) strlen (msg));

    if (written > 0)
    {
        std::cout << "Red Pitaya: Sent command -> " << msg;
    }
    else
    {
        std::cout << "Red Pitaya Backend ERROR: Socket write failed." << std::endl;
        commandSocket->close();
    }
}

void AcqBoardRedPitaya::setFilterEnabled (bool enabled)
{
    if (commandSocket == nullptr)
        return;

    const char* msg = enabled ? "FILTER ON\n" : "FILTER OFF\n";

    int written = commandSocket->write (msg, (int) strlen (msg));

    if (written > 0)
        std::cout << "Red Pitaya: Sent command -> " << msg;
    else
        std::cout << "Red Pitaya Backend ERROR: Socket write failed." << std::endl;

    filterEnabled = enabled;
}

void AcqBoardRedPitaya::setAnalogInGain (float gain)
{
    if (commandSocket == nullptr)
        return;

    char msg[32];
    snprintf (msg, sizeof (msg), "AIN_GAIN:%.2f\n", gain);

    int written = commandSocket->write (msg, (int) strlen (msg));

    if (written > 0)
        std::cout << "Red Pitaya: Sent command -> " << msg;
    else
        std::cout << "Red Pitaya Backend ERROR: Socket write failed." << std::endl;

    analogInGain = gain;
}

void AcqBoardRedPitaya::setAnalogOutVoltage (float voltage)
{
    if (commandSocket == nullptr)
        return;

    char msg[32];
    snprintf (msg, sizeof (msg), "AOUT:%.2f\n", voltage);

    int written = commandSocket->write (msg, (int) strlen (msg));

    if (written > 0)
        std::cout << "Red Pitaya: Sent command -> " << msg;
    else
        std::cout << "Red Pitaya Backend ERROR: Socket write failed." << std::endl;

    analogOutVoltage = voltage;
}

String AcqBoardRedPitaya::getStreamSensorName (int index) const
{
    if (index < 0 || index >= streamSensorNames.size())
        return {};
    return streamSensorNames.getReference (index);
}

/* LSB per physical unit for each preset index (matches server-side tables).
 * ACC: LSB per g.   GYR: LSB per °/s. */
static const float kAccSensitivity[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
static const float kGyrSensitivity[4] = { 131.072f,  65.536f, 32.768f, 16.384f };

bool AcqBoardRedPitaya::sendSensorCfgAcc (int sensorIndex, int presetId)
{
    if (sensorIndex >= 0 && sensorIndex < 6)
        sensorAccPreset[sensorIndex] = presetId;

    if (commandSocket == nullptr)
        return false;

    char msg[48];
    snprintf (msg, sizeof (msg), "CFG %d ACC %d\n", sensorIndex, presetId);
    return commandSocket->write (msg, (int) strlen (msg)) > 0;
}

bool AcqBoardRedPitaya::sendSensorCfgGyr (int sensorIndex, int presetId)
{
    if (sensorIndex >= 0 && sensorIndex < 6)
        sensorGyrPreset[sensorIndex] = presetId;

    if (commandSocket == nullptr)
        return false;

    char msg[48];
    snprintf (msg, sizeof (msg), "CFG %d GYR %d\n", sensorIndex, presetId);
    return commandSocket->write (msg, (int) strlen (msg)) > 0;
}

bool AcqBoardRedPitaya::sendSensorCfgSrate (int sensorIndex, int targetHz)
{
    if (commandSocket == nullptr)
        return false;

    char msg[48];
    snprintf (msg, sizeof (msg), "CFG %d SRATE %d\n", sensorIndex, targetHz);
    return commandSocket->write (msg, (int) strlen (msg)) > 0;
}

double AcqBoardRedPitaya::setUpperBandwidth (double upperBandwidth)
{
    upperBandwidthHz = upperBandwidth;
    return upperBandwidthHz;
}

double AcqBoardRedPitaya::setLowerBandwidth (double lowerBandwidth)
{
    lowerBandwidthHz = lowerBandwidth;
    return lowerBandwidthHz;
}

double AcqBoardRedPitaya::setDspCutoffFreq (double freq)
{
    dspCutoffFreqHz = freq;
    return dspCutoffFreqHz;
}

double AcqBoardRedPitaya::getDspCutoffFreq() const
{
    return dspCutoffFreqHz;
}

void AcqBoardRedPitaya::setDspOffset (bool /*enabled*/)
{
}

void AcqBoardRedPitaya::setTTLOutputMode (bool enabled)
{
    ttlOutputMode = enabled;
}

void AcqBoardRedPitaya::setDAChpf (float /*cutoff*/, bool /*enabled*/)
{
}

void AcqBoardRedPitaya::setFastTTLSettle (bool /*state*/, int /*channel*/)
{
}

int AcqBoardRedPitaya::setNoiseSlicerLevel (int level)
{
    return level;
}

void AcqBoardRedPitaya::enableBoardLeds (bool /*enabled*/)
{
}

int AcqBoardRedPitaya::setClockDivider (int divide_ratio)
{
    clockDivideRatio = divide_ratio;
    return clockDivideRatio;
}

void AcqBoardRedPitaya::connectHeadstageChannelToDAC (int /*headstageChannelIndex*/, int /*dacChannelIndex*/)
{
}

void AcqBoardRedPitaya::setDACTriggerThreshold (int /*dacChannelIndex*/, float /*threshold*/)
{
}

bool AcqBoardRedPitaya::isHeadstageEnabled (int /*hsNum*/) const
{
    return false;
}

int AcqBoardRedPitaya::getActiveChannelsInHeadstage (int /*hsNum*/) const
{
    return 0;
}

int AcqBoardRedPitaya::getChannelsInHeadstage (int /*hsNum*/) const
{
    return 0;
}

int AcqBoardRedPitaya::getNumDataOutputs (ContinuousChannel::Type channelType)
{
    if (channelType == ContinuousChannel::ELECTRODE)
        return 0;

    if (channelType == ContinuousChannel::AUX)
    {
        if (acquireAux)
            return 0;

        return 0;
    }

    if (channelType == ContinuousChannel::ADC)
    {
        if (acquireAdc)
            return numAdcChannels; // The DeviceThread assumes up to 6 ADC channels.

        return 0;
    }

    return 0;
}

void AcqBoardRedPitaya::setNumHeadstageChannels (int /*headstageIndex*/, int /*channelCount*/)
{
}

void AcqBoardRedPitaya::launchOpenSimMotion()
{
    const juce::String workDir = kOpenSimWorkDir;
    const juce::String bridgePath = workDir + "\\ephys_to_opensim_bridge.py";
    const juce::String logPath = workDir + "\\ephys_bridge_run.log";
    juce::File batFile = juce::File::getSpecialLocation (juce::File::tempDirectory)
                             .getChildFile ("opensim_motion_hidden_launch.bat");

    batFile.replaceWithText (
        "@echo off\r\n"
        "cd /d \"" + workDir + "\"\r\n"
        "set PATH=C:\\OpenSim 4.5\\bin;%PATH%\r\n"
        "set PYTHONPATH=C:\\OpenSim 4.5\\sdk\\Python;%PYTHONPATH%\r\n"
        "py -3.12 \"" + bridgePath + "\" listen --until-idle 5"
        + " > \"" + logPath + "\" 2>&1\r\n");

    const juce::String command = "cmd.exe /c \"\"" + batFile.getFullPathName() + "\"\"";

    openSimProcess = std::make_unique<juce::ChildProcess>();
    if (openSimProcess->start (command))
    {
        std::cout << "OpenSim Motion generation launched hidden. Log: "
                  << logPath << std::endl;
    }
    else
    {
        openSimProcess.reset();
        std::cout << "OpenSim Motion generation failed to launch." << std::endl;
        juce::AlertWindow::showMessageBoxAsync (
            juce::AlertWindow::WarningIcon,
            "Gen Motion",
            "Could not launch the OpenSim bridge. See ephys_bridge_run.log.");
        return;
    }

    openSimSocket = std::make_unique<DatagramSocket>();
    openSimEnabled = true;
    std::cout << "Red Pitaya: OpenSim UDP forwarding enabled -> 127.0.0.1:5000" << std::endl;
}

void AcqBoardRedPitaya::launchOpenSimLive()
{
    const juce::String workDir = kOpenSimWorkDir;
    const juce::String scriptPath = workDir + "\\opensim_live_realtime.py";
    juce::File batFile = juce::File::getSpecialLocation (juce::File::tempDirectory)
                             .getChildFile ("opensim_live_visible.bat");

    batFile.replaceWithText (
        "@echo off\r\n"
        "cd /d \"" + workDir + "\"\r\n"
        "set PATH=C:\\OpenSim 4.5\\bin;%PATH%\r\n"
        "set PYTHONPATH=C:\\OpenSim 4.5\\sdk\\Python;%PYTHONPATH%\r\n"
        "set OPENSIM_LIVE_SOURCE=real_redpitaya\r\n"
        "set OPENSIM_LIVE_GYRO_TEST=normal\r\n"
        "for /f \"tokens=5\" %%p in ('netstat -ano ^| findstr \":5000\"') do taskkill /PID %%p /F >nul 2>&1\r\n"
        "start \"OpenSim Live\" cmd /k py -3.8 -u \"" + scriptPath + "\"\r\n");

    const juce::String command = "cmd.exe /c \"\"" + batFile.getFullPathName() + "\"\"";

    openSimLiveProcess = std::make_unique<juce::ChildProcess>();
    if (! openSimLiveProcess->start (command))
    {
        openSimLiveProcess.reset();
        std::cout << "OpenSim Live: failed to launch." << std::endl;
        juce::AlertWindow::showMessageBoxAsync (
            juce::AlertWindow::WarningIcon,
            "OpenSim Live",
            "Could not start the live bridge.");
        return;
    }

    std::cout << "OpenSim Live: launched visible console (py -3.8 -u)." << std::endl;

    openSimSocket = std::make_unique<DatagramSocket>();
    openSimEnabled = true;
    std::cout << "Red Pitaya: OpenSim UDP forwarding enabled -> 127.0.0.1:5000" << std::endl;
}

void AcqBoardRedPitaya::sendOpenSimImuPacket (float timestamp, const float* data, int numImus)
{
    if (! openSimEnabled || openSimSocket == nullptr || data == nullptr || numImus < 1)
        return;

    const int totalFloats = 1 + numImus * 6;
    juce::HeapBlock<float> pkt (totalFloats);
    pkt[0] = timestamp;

    for (int i = 0; i < numImus * 6; ++i)
        pkt[1 + i] = data[i];

    openSimSocket->write ("127.0.0.1", 5000, pkt, totalFloats * (int) sizeof (float));

    static int openSimUdpSendCount = 0;
    ++openSimUdpSendCount;

    if (openSimUdpSendCount == 1 || (openSimUdpSendCount % 1000) == 0)
    {
        const float imu2Gy = (numImus >= 3) ? data[2 * 6 + 4] : std::numeric_limits<float>::quiet_NaN();
        std::cout << "OpenSim UDP sent #" << openSimUdpSendCount
                  << " t=" << timestamp
                  << " n_imus=" << numImus
                  << " gy[2]=" << imu2Gy << std::endl;
    }
}


void AcqBoardRedPitaya::run()
{
    if (commandSocket == nullptr)
        return;

    int64 sampleNumber = 0;
    double elapsedSeconds = 0.0;
    const int64 samplesPerBuffer = jmax (
        int64 (1),
        int64 (std::lround (static_cast<double> (settings.boardSampleRate) / 1000.0)));
    uint64 eventCode = 0;

    const int numAdcChannelsLocal = getNumDataOutputs (ContinuousChannel::ADC);

    if (numAdcChannelsLocal <= 0 || numAdcChannelsLocal > MAX_CHANNELS || samplesPerBuffer <= 0)
        return;

    // Open a dedicated UDP socket for the data stream.
    // The TCP commandSocket remains open exclusively for control commands
    // (STOP signalled by TCP close in stopAcquisition()).
    DatagramSocket udpSocket;
    if (! udpSocket.bindToPort (55001))
    {
        std::cout << "Red Pitaya ERROR: Failed to bind UDP socket on port 55001" << std::endl;
        return;
    }

    constexpr int headerSize      = 22;
    const int     payloadSize     = numAdcChannelsLocal * 2;  // int16 per channel
    const int     packetSize      = headerSize + payloadSize;
    constexpr int packetsPerChunk = 1;                        // must match CHUNK_SAMPLES in RedPitaya_justin.c
    const int     chunkSize       = packetSize * packetsPerChunk;

    uint8_t chunkBuffer[65507]; // max UDP payload

    while (! threadShouldExit())
    {
        DataBuffer* currentBuffer = buffer;

        if (currentBuffer == nullptr)
        {
            Thread::sleep (1);
            continue;
        }

        // Rebuild per-channel scale factors each buffer so mid-stream range
        // changes (sendSensorCfgAcc/Gyr) take effect within one buffer.
        float channelScale[MAX_CHANNELS];
        for (int i = 0; i < MAX_CHANNELS; ++i)
            channelScale[i] = 1.0f;

        {
            int chanOffset = 0;
            const int numSensors = streamSensorNames.size();

            for (int si = 0; si < numSensors && si < 6; ++si)
            {
                const String& sname = streamSensorNames.getReference (si);
                const float accScale = 1.0f / kAccSensitivity[jlimit (0, 3, sensorAccPreset[si])];
                const float gyrScale = 1.0f / kGyrSensitivity[jlimit (0, 3, sensorGyrPreset[si])];
                const bool is6axis   = (sname == "MPU6050");
                const bool isBNO     = (sname == "BNO055");
                const int  numRaw    = is6axis ? 6 : 9;

                // acc axes are always the first three channels for all sensors
                channelScale[chanOffset + 0] = accScale;
                channelScale[chanOffset + 1] = accScale;
                channelScale[chanOffset + 2] = accScale;

                if (isBNO)
                {
                    // BNO055 layout: [0-2]=acc, [3-5]=mag, [6-8]=gyr
                    channelScale[chanOffset + 6] = gyrScale;
                    channelScale[chanOffset + 7] = gyrScale;
                    channelScale[chanOffset + 8] = gyrScale;
                    // mag [3-5] left at 1.0 — no user-settable range
                }
                else
                {
                    // MPU family: [0-2]=acc, [3-5]=gyr, [6-8]=mag (9-axis only)
                    channelScale[chanOffset + 3] = gyrScale;
                    channelScale[chanOffset + 4] = gyrScale;
                    channelScale[chanOffset + 5] = gyrScale;
                    // mag [6-8] left at 1.0 — no user-settable range
                }
                // quaternion channels [numRaw..numRaw+3] left at 1.0

                chanOffset += numRaw + 4;
            }
            // analog waveform channels after sensors left at 1.0
        }

        // Poll with 100ms timeout so threadShouldExit() is checked regularly.
        if (! udpSocket.waitUntilReady (true, 100))
            continue;

        int n = udpSocket.read ((char*) chunkBuffer, chunkSize, false);

        // Skip malformed or empty datagrams.
        if (n <= 0 || (n % packetSize) != 0)
            continue;

        const int numPackets = n / packetSize;

        for (int p = 0; p < numPackets && ! threadShouldExit(); ++p)
        {
            uint8_t*       pkt             = chunkBuffer + (p * packetSize);
            const int16_t* channels        = reinterpret_cast<const int16_t*> (pkt + headerSize);
            const int      channelsInPacket = payloadSize / 2;
            const int      sampleIndex      = p % (int) samplesPerBuffer;

            for (int adc = 0; adc < numAdcChannelsLocal; ++adc)
            {
                const float raw = (adc < channelsInPacket) ? float (channels[adc]) : 0.0f;
                samples[(adc * samplesPerBuffer) + sampleIndex] = raw * channelScale[adc];
            }

            {
                const int numImusLocal = numAdcChannelsLocal / 6;

                if (numImusLocal >= 1)
                {
                    const int nCh = numImusLocal * 6;
                    float imuPkt[MAX_CHANNELS];

                    for (int ch = 0; ch < nCh; ++ch)
                        imuPkt[ch] = samples[ch * samplesPerBuffer + sampleIndex];

                    if (sampleNumber == 0)
                    {
                        std::cout << "Red Pitaya: real " << numImusLocal << "-IMU packet with "
                                  << nCh << " floats — IMU0 ax=" << imuPkt[0] << std::endl;
                    }

                    sendOpenSimImuPacket ((float) elapsedSeconds, imuPkt, numImusLocal);
                }
            }

            const double currentSampleRate = jmax (1.0, static_cast<double> (settings.boardSampleRate));
            sampleNumbers[sampleIndex] = sampleNumber;
            timestamps[sampleIndex]    = elapsedSeconds;
            event_codes[sampleIndex]   = eventCode;

            ++sampleNumber;
            elapsedSeconds += 1.0 / currentSampleRate;

            if (sampleIndex == (int) samplesPerBuffer - 1)
            {
                currentBuffer = buffer;
                if (currentBuffer != nullptr && ! threadShouldExit())
                    currentBuffer->addToBuffer (samples,
                                                sampleNumbers,
                                                timestamps,
                                                event_codes,
                                                (int) samplesPerBuffer);
            }
        }
    }
}

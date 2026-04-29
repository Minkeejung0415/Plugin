/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI

    ------------------------------------------------------------------
*/

#include "AcqBoardRedPitaya.h"
#include <cstdint>
#include <cstring>

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

    // Populate with a simple set of options; these can be adapted
    // to match the Red Pitaya configuration.
    sampleRates.add (1000);
    sampleRates.add (2000);
    sampleRates.add (5000);
    sampleRates.add (10000);

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

    // 2. Close the TCP connection first. The Red Pitaya server then sees EOF /
    //    send failure and leaves run_stream; we must NOT write STOP (or anything)
    //    on this socket while run() is still consuming the same byte stream as
    //    binary packets — that corrupts framing and leaves stale bytes for the
    //    next START (classic "second start works" failure).
    if (commandSocket != nullptr)
        commandSocket->close();

    // 3. Wait for run() to finish before deleting the socket object.
    stopThread (500);

    if (commandSocket != nullptr)
    {
        delete commandSocket;
        commandSocket = nullptr;
    }

    if (buffer != nullptr)
        buffer->clear();

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

        // Connect (consider swapping to direct IP like "192.168.x.x" if mDNS lags)
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
    snprintf (msg, sizeof (msg), "FREQ:%d\n", newFreq);

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

bool AcqBoardRedPitaya::sendSensorCfgAcc (int sensorIndex, int presetId)
{
    if (commandSocket == nullptr)
        return false;

    char msg[48];
    snprintf (msg, sizeof (msg), "CFG %d ACC %d\n", sensorIndex, presetId);
    return commandSocket->write (msg, (int) strlen (msg)) > 0;
}

bool AcqBoardRedPitaya::sendSensorCfgGyr (int sensorIndex, int presetId)
{
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

void AcqBoardRedPitaya::run()
{
    if (commandSocket == nullptr)
        return;

    int64 sampleNumber = 0;
    const int64 samplesPerBuffer = int64 (settings.boardSampleRate / 1000.0);
    uint64 eventCode = 0;

    const int numAdcChannelsLocal = getNumDataOutputs (ContinuousChannel::ADC);

    if (numAdcChannelsLocal <= 0 || numAdcChannelsLocal > MAX_CHANNELS || samplesPerBuffer <= 0)
        return;

    constexpr int headerSize = 22;
    constexpr int maxPacketSize = 1024;

    uint8_t packet[maxPacketSize];

    // Non-blocking read helper: polls in 100ms slices so threadShouldExit() is
    // checked regularly and stopAcquisition()'s close() is noticed promptly.
    auto socketReadFully = [&] (void* buf, int size) -> bool
    {
        int done = 0;
        char* ptr = static_cast<char*> (buf);
        while (done < size && ! threadShouldExit())
        {
            int ready = commandSocket->waitUntilReady (true, 100);
            if (ready < 0)
                return false; // socket closed / error
            if (ready == 0)
                continue; // 100ms timeout — recheck threadShouldExit
            int n = commandSocket->read (ptr + done, size - done, false);
            if (n <= 0)
                return false;
            done += n;
        }
        return ! threadShouldExit();
    };

    // MSVC: nested lambda must capture constexpr locals explicitly (no implicit capture in []).
    auto parseHeaderBytesPerFrame = [=] (const uint8_t* hdr, int32_t& outBytes) -> bool
    {
        if (hdr[8] != 0x03 || hdr[9] != 0x00)
            return false;

        memcpy (&outBytes, hdr + 4, sizeof (int32_t));

        // int16 samples per frame; keep in sync with Red Pitaya write_stream_header
        if (outBytes < 2 || outBytes > (maxPacketSize - headerSize) || (outBytes & 1) != 0)
            return false;

        return true;
    };

    // Read one 22-byte header + variable payload. If the header is not valid,
    // slide one byte and retry (same framing idea as the original sync loop).
    auto readOneFrame = [&] (int32_t& outPayloadBytes) -> bool
    {
        if (! socketReadFully (packet, headerSize))
            return false;

        constexpr int maxResync = 65536;

        for (int guard = 0; guard < maxResync && ! threadShouldExit(); ++guard)
        {
            if (parseHeaderBytesPerFrame (packet, outPayloadBytes))
                return socketReadFully (packet + headerSize, (int) outPayloadBytes);

            memmove (packet, packet + 1, (size_t) headerSize - 1);

            if (! socketReadFully (packet + headerSize - 1, 1))
                return false;
        }

        return false;
    };

    while (! threadShouldExit())
    {
        for (int sampleIndex = 0; sampleIndex < samplesPerBuffer; ++sampleIndex)
        {
            int32_t payloadBytes = 0;

            if (! readOneFrame (payloadBytes))
                return;

            const int16_t* channels = reinterpret_cast<const int16_t*> (packet + headerSize);
            const int channelsInPacket = payloadBytes / 2;

            for (int adc = 0; adc < numAdcChannelsLocal; ++adc)
            {
                const float v = (adc < channelsInPacket) ? float (channels[adc]) : 0.0f;
                samples[(adc * samplesPerBuffer) + sampleIndex] = v;
            }

            const double timeSeconds = double (sampleNumber) / double (settings.boardSampleRate);
            sampleNumbers[sampleIndex] = sampleNumber;
            timestamps[sampleIndex] = timeSeconds;
            event_codes[sampleIndex] = eventCode;

            ++sampleNumber;
        }

        buffer->addToBuffer (samples,
                             sampleNumbers,
                             timestamps,
                             event_codes,
                             (int) samplesPerBuffer);
    }
}

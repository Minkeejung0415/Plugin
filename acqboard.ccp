/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI

    ------------------------------------------------------------------
*/

#include "AcqBoardRedPitaya.h"
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

    if (commandSocket != nullptr && ! commandSocket->isConnected())
    {
        commandSocket->close();
        delete commandSocket;
        commandSocket = nullptr;
    }

    if (commandSocket == nullptr)
    {
        commandSocket = new StreamingSocket();
        if (! commandSocket->connect ("rp-f0f85a.local", 5000, 1000))
        {
            std::cout << "Red Pitaya ERROR: Could not connect to board." << std::endl;
            delete commandSocket;
            commandSocket = nullptr;
            return false;
        }
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
    // 1. Signal the thread to exit so readFully() starts returning false
    //    on the next 100ms waitUntilReady timeout.
    if (isThreadRunning())
        signalThreadShouldExit();

    // 2. Tell the server to stop streaming.
    if (commandSocket != nullptr && commandSocket->isConnected())
    {
        const char* msg = "STOP\n";
        commandSocket->write (msg, (int) strlen (msg));
        juce::Thread::sleep (50);
    }

    // 3. Close the socket. This causes waitUntilReady() in run() to return -1
    //    immediately (invalid handle), so the thread exits without waiting for
    //    the full 100ms timeout — no blocking recv stuck forever.
    if (commandSocket != nullptr)
        commandSocket->close();

    // 4. Wait for run() to fully exit BEFORE deleting the socket object.
    //    With readFully() using 100ms timeouts the thread exits within ~100ms.
    //    Deleting commandSocket before this point is a use-after-free.
    stopThread (500);

    // 5. Now safe to delete — thread is guaranteed done.
    if (commandSocket != nullptr)
    {
        delete commandSocket;
        commandSocket = nullptr;
    }

    if (buffer != nullptr)
        buffer->clear();

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

    const int headerSize = 22;
    const int payloadSize = numAdcChannelsLocal * 2;
    const int packetSize = headerSize + payloadSize;

    constexpr int MAX_PACKET_SIZE = 1024;
    uint8_t packet[MAX_PACKET_SIZE];

    // Non-blocking read helper: polls in 100ms slices so threadShouldExit() is
    // checked regularly and stopAcquisition()'s close() is noticed promptly.
    auto readFully = [&] (void* buf, int size) -> bool
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

    bool synchronized = false;
    uint8_t syncBuffer[headerSize];
    int bytesSearched = 0;

    while (! synchronized && ! threadShouldExit())
    {
        if (! readFully (syncBuffer + bytesSearched, 1))
            return;
        bytesSearched++;

        if (bytesSearched == headerSize)
        {
            if (syncBuffer[8] == 0x03 && syncBuffer[9] == 0x00)
            {
                memcpy (packet, syncBuffer, headerSize);

                if (! readFully (packet + headerSize, payloadSize))
                    return;
                synchronized = true;
            }
            else
            {
                memmove (syncBuffer, syncBuffer + 1, headerSize - 1);
                bytesSearched--;
            }
        }
    }

    while (! threadShouldExit())
    {
        for (int sampleIndex = 0; sampleIndex < samplesPerBuffer; ++sampleIndex)
        {
            if (sampleIndex > 0 || ! synchronized)
            {
                if (! readFully (packet, packetSize))
                    return;
            }

            synchronized = false;

            const int16_t* channels = reinterpret_cast<const int16_t*> (packet + headerSize);

            int ch = 0;
            for (int adc = 0; adc < numAdcChannelsLocal; ++adc)
            {
                samples[(ch * samplesPerBuffer) + sampleIndex] = float (channels[adc]);
                ++ch;
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

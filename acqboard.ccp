/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI

    ------------------------------------------------------------------
*/

#include "Acqboardredpitaya.h"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace
{
    const char* kRedPitayaHosts[] = { "rp-f0f85a.local", "rp-f0cd35.local" };
    constexpr int kNumRedPitayaHosts = 2;
    // active + editor Node IP + ESP32_NODE_HOST + both rp-*.local (deduped)
    constexpr int kMaxHostsToTry = kNumRedPitayaHosts + 4;

    const char* kOpenSimWorkDir = "C:\\Users\\KIN Student\\Open-Sim--Bio-Mech";

    const char* kEsp32RecordDir = "C:\\Users\\KIN Student\\Documents\\Arduino\\ESP32-S3-1\\results";

    // OpenSim PhysicalOffsetFrame names for the Rajagopal2015 model (distal to proximal, right then left).
    const char* kBodySegmentNames[] = {
        "tibia_r_imu",   "femur_r_imu",  "pelvis_imu",    "torso_imu",
        "calcn_r_imu",   "femur_l_imu",  "tibia_l_imu",   "calcn_l_imu",
        "humerus_r_imu", "radius_r_imu", "humerus_l_imu", "radius_l_imu",
        "head_imu"
    };
    const char* kBodySegmentLabels[] = {
        "Right Tibia",     "Right Femur",   "Pelvis",         "Torso",
        "Right Foot",      "Left Femur",    "Left Tibia",     "Left Foot",
        "Right Upper Arm", "Right Forearm", "Left Upper Arm", "Left Forearm",
        "Head"
    };

    String envEsp32NodeHost()
    {
        if (const char* val = std::getenv ("ESP32_NODE_HOST"))
            return String (val).trim();

        return {};
    }

    int parseEsp32ChannelCountFromHandshake (const String& response, int defaultChannels)
    {
        const int idx = response.indexOf (" channels");

        if (idx > 0)
        {
            const int n = response.substring (0, idx).trim().getIntValue();

            if (n >= 8 && n <= AcqBoardRedPitaya::MAX_CHANNELS)
                return n;
        }

        const int okIdx = response.indexOf ("CHANNELS:");

        if (okIdx >= 0)
        {
            const int n = response.substring (okIdx + 9).trim().getIntValue();

            if (n >= 8 && n <= AcqBoardRedPitaya::MAX_CHANNELS)
                return n;
        }

        return defaultChannels;
    }

    bool isEsp32HandshakeResponse (const String& response)
    {
        return response.containsIgnoreCase ("esp32s3")
               || response.containsIgnoreCase ("node=esp32")
               || (response.contains (" channels") && response.contains ("sample_rate"));
    }

    int rawChannelCountForSensorName (const String& sensorName)
    {
        if (sensorName == "MPU6050")
            return 6;

        return 9;
    }

    int channelsPerSensorSlot (const String& sensorName)
    {
        return rawChannelCountForSensorName (sensorName) + 4;
    }

    int channelOffsetForSensorIndex (const Array<String>& sensorNames, int sensorIndex)
    {
        int offset = 0;

        for (int i = 0; i < sensorIndex && i < sensorNames.size(); ++i)
            offset += channelsPerSensorSlot (sensorNames.getReference (i));

        return offset;
    }

    // Firmware >=1.7.0: header offset = low 32 bits of esp_timer_get_time() (µs since boot).
    // Returns inter-frame seconds, or -1.0 to use wall-clock / nominal fallback.
    double deltaSecFromHwOffsetUs (int32_t offsetUs, uint32_t& lastHwUs32, bool& haveLastHw)
    {
        if (offsetUs == 0)
            return -1.0;

        const uint32_t hwUs = static_cast<uint32_t> (offsetUs);

        if (! haveLastHw)
        {
            haveLastHw = true;
            lastHwUs32 = hwUs;
            return -1.0;
        }

        const uint32_t deltaUs = hwUs - lastHwUs32;
        lastHwUs32 = hwUs;
        return static_cast<double> (deltaUs) * 1.0e-6;
    }

    void quaternionFromScaledQ15 (float qwScaled, float qxScaled, float qyScaled, float qzScaled,
                                  float& qw, float& qx, float& qy, float& qz)
    {
        constexpr float kQ15 = 1.0f / 32767.0f;
        qw = qwScaled * kQ15;
        qx = qxScaled * kQ15;
        qy = qyScaled * kQ15;
        qz = qzScaled * kQ15;

        const float n2 = qw * qw + qx * qx + qy * qy + qz * qz;

        if (n2 > 1.0e-8f)
        {
            const float inv = 1.0f / std::sqrt (n2);
            qw *= inv;
            qx *= inv;
            qy *= inv;
            qz *= inv;
        }
    }

    /** Madgwick 6-DOF (gyro + accel) AHRS update, in place on q = {qw,qx,qy,qz}.
     *
     *  The ESP32-S3 node streams only raw accelerometer and gyroscope (its firmware
     *  does not read the ICM20948 magnetometer), so we fuse orientation here to feed
     *  OpenSim, which needs quaternions. This is self-contained on purpose: it adds no
     *  new compile unit to the plugin build. Without a magnetometer the heading (yaw)
     *  is unobservable and will slowly drift — that is inherent to 6-DOF fusion.
     *
     *  @param gx,gy,gz  angular rate in rad/s
     *  @param ax,ay,az  acceleration (any consistent unit; normalized internally)
     *  @param dt        sample period in seconds
     *  @param beta      filter gain (accel correction strength; ~0.1 is a good default)
     */
    void madgwickUpdate6DOF (float q[4],
                             float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float dt, float beta)
    {
        float qw = q[0], qx = q[1], qy = q[2], qz = q[3];

        // Rate of change of quaternion from the gyroscope.
        float qDot0 = 0.5f * (-qx * gx - qy * gy - qz * gz);
        float qDot1 = 0.5f * (qw * gx + qy * gz - qz * gy);
        float qDot2 = 0.5f * (qw * gy - qx * gz + qz * gx);
        float qDot3 = 0.5f * (qw * gz + qx * gy - qy * gx);

        const float aNorm = std::sqrt (ax * ax + ay * ay + az * az);

        if (aNorm > 1.0e-9f)
        {
            ax /= aNorm;
            ay /= aNorm;
            az /= aNorm;

            // Gradient descent corrective step (objective + Jacobian for gravity).
            const float f1 = 2.0f * (qx * qz - qw * qy) - ax;
            const float f2 = 2.0f * (qw * qx + qy * qz) - ay;
            const float f3 = 2.0f * (0.5f - qx * qx - qy * qy) - az;

            float s0 = -2.0f * qy * f1 + 2.0f * qx * f2;
            float s1 = 2.0f * qz * f1 + 2.0f * qw * f2 - 4.0f * qx * f3;
            float s2 = -2.0f * qw * f1 + 2.0f * qz * f2 - 4.0f * qy * f3;
            float s3 = 2.0f * qx * f1 + 2.0f * qy * f2;

            const float sNorm = std::sqrt (s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);

            if (sNorm > 1.0e-9f)
            {
                s0 /= sNorm;
                s1 /= sNorm;
                s2 /= sNorm;
                s3 /= sNorm;

                qDot0 -= beta * s0;
                qDot1 -= beta * s1;
                qDot2 -= beta * s2;
                qDot3 -= beta * s3;
            }
        }

        qw += qDot0 * dt;
        qx += qDot1 * dt;
        qy += qDot2 * dt;
        qz += qDot3 * dt;

        const float qNorm = std::sqrt (qw * qw + qx * qx + qy * qy + qz * qz);

        if (qNorm > 1.0e-9f)
        {
            q[0] = qw / qNorm;
            q[1] = qx / qNorm;
            q[2] = qy / qNorm;
            q[3] = qz / qNorm;
        }
    }

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
    activeRedPitayaHost = {};

    if (commandSocket != nullptr && ! commandSocket->isConnected())
        resetCommandSocket();

    for (int i = 0; i < kNumRedPitayaHosts; ++i)
    {
        const String host (kRedPitayaHosts[i]);
        resetCommandSocket();

        if (! connectCommandSocketToHost (host))
        {
            std::cout << "Red Pitaya: connect failed: " << host << std::endl;
            continue;
        }

        activeRedPitayaHost = host;

        if (performDetectionHandshake())
            return true;

        resetCommandSocket();
        activeRedPitayaHost = {};
    }

    return false;
} */

void AcqBoardRedPitaya::resetCommandSocket()
{
    if (commandSocket != nullptr)
    {
        commandSocket->close();
        delete commandSocket;
        commandSocket = nullptr;
    }
}

bool AcqBoardRedPitaya::connectCommandSocketToHost (const String& host)
{
    if (commandSocket == nullptr)
        commandSocket = new StreamingSocket();

    if (! commandSocket->connect (host, 5000, 1000))
        return false;

    std::cout << "Red Pitaya: TCP connected to " << host << ":5000" << std::endl;
    return true;
}

bool AcqBoardRedPitaya::performDetectionHandshake()
{
    const char* msg = "REDPITAYA\n";
    commandSocket->write (msg, (int) strlen (msg));

    if (! commandSocket->waitUntilReady (true, 500))
    {
        std::cout << "Red Pitaya: no handshake reply from " << activeRedPitayaHost << std::endl;
        return false;
    }

    String response;
    String line;

    for (int guard = 0; guard < 4; ++guard)
    {
        line.clear();

        if (! commandSocket->waitUntilReady (true, 500))
            break;

        char c = 0;

        while (commandSocket->waitUntilReady (true, 200))
        {
            if (commandSocket->read (&c, 1, false) <= 0 || c == '\n')
                break;

            if ((unsigned char) c < 0x20 || (unsigned char) c > 0x7E)
                return false;

            line += c;
        }

        if (line.isEmpty())
            break;

        if (response.isEmpty())
            response = line;
        else
            response += " " + line;

        if (line.containsIgnoreCase ("CHANNELS") || line.startsWith ("OK"))
            break;
    }

    if (response.isEmpty())
        return false;

    std::cout << "Red Pitaya: handshake response: " << response << std::endl;

    // ESP32-S3 STEP node: "8 channels; sample_rate=100; node=esp32s3_arduino".
    // Check this BEFORE the generic Red Pitaya "OK" branch: the ESP32 firmware and
    // the USB bridge both follow the esp32 marker line with "OK CHANNELS:8", and over
    // a localhost/TCP connection the two writes usually coalesce into a single read.
    // If we tested for "OK" first, that "OK CHANNELS:8" would misclassify the ESP32 as
    // a Red Pitaya, sending run() down the UDP:55001 path while the board streams
    // binary over TCP — i.e. a successful "detect" that never produces any samples.
    if (isEsp32HandshakeResponse (response))
    {
        isEsp32Node = true;
        numAdcChannels = parseEsp32ChannelCountFromHandshake (
            response, AcqBoardRedPitaya::ESP32_DEFAULT_CHANNELS);
        settings.boardSampleRate = 100.0f;

        const int srIdx = response.indexOf ("sample_rate=");

        if (srIdx >= 0)
        {
            String srText = response.substring (srIdx + 12).trim();

            if (srText.contains (";"))
                srText = srText.upToFirstOccurrenceOf (";", false, false).trim();

            const int hz = srText.getIntValue();

            if (hz >= 1)
                settings.boardSampleRate = static_cast<float> (hz);
        }

        deviceFound = true;
        std::cout << "Detected ESP32-S3 node at " << activeRedPitayaHost << " with "
                  << numAdcChannels << " channels @ "
                  << (int) settings.boardSampleRate << " Hz (TCP stream)." << std::endl;
        return true;
    }

    // Red Pitaya firmware: "OK CHANNELS:N" (no esp32 markers)
    if (response.contains ("OK"))
    {
        isEsp32Node = false;

        if (response.contains ("CHANNELS:"))
        {
            const int startIdx = response.indexOf ("CHANNELS:") + 9;
            numAdcChannels = response.substring (startIdx).getIntValue();
        }
        else
            numAdcChannels = 6;

        deviceFound = true;
        std::cout << "Detected Red Pitaya at " << activeRedPitayaHost << " with "
                  << numAdcChannels << " channels." << std::endl;
        return true;
    }

    return false;
}

bool AcqBoardRedPitaya::connectCommandSocketToBoard()
{
    String hostsToTry[kMaxHostsToTry];
    int numTry = 0;

    auto tryPushHost = [&](const String& host)
    {
        if (host.isEmpty() || numTry >= kMaxHostsToTry)
            return;

        for (int j = 0; j < numTry; ++j)
            if (hostsToTry[j] == host)
                return;

        hostsToTry[numTry++] = host;
    };

    tryPushHost (activeRedPitayaHost);

    const String envHost = envEsp32NodeHost();
    const String cfgHost = configurableNodeHost.trim();

    tryPushHost (cfgHost);
    tryPushHost (envHost);

    for (int i = 0; i < kNumRedPitayaHosts; ++i)
        tryPushHost (String (kRedPitayaHosts[i]));

    for (int i = 0; i < numTry; ++i)
    {
        resetCommandSocket();

        if (! connectCommandSocketToHost (hostsToTry[i]))
        {
            std::cout << "Red Pitaya: connect failed: " << hostsToTry[i] << std::endl;
            continue;
        }

        activeRedPitayaHost = hostsToTry[i];
        return true;
    }

    activeRedPitayaHost = {};
    return false;
}


bool AcqBoardRedPitaya::detectBoard()
{
    std::cout << "detectBoard called" << std::endl;

    deviceFound = false;
    activeRedPitayaHost = {};

    if (commandSocket != nullptr && ! commandSocket->isConnected())
        resetCommandSocket();

    for (int i = 0; i < kNumRedPitayaHosts; ++i)
    {
        const String host (kRedPitayaHosts[i]);
        resetCommandSocket();

        if (! connectCommandSocketToHost (host))
        {
            std::cout << "Red Pitaya: connect failed: " << host << std::endl;
            continue;
        }

        activeRedPitayaHost = host;

        if (performDetectionHandshake())
            return true;

        resetCommandSocket();
        activeRedPitayaHost = {};
    }

    // ESP32-S3: configurable host, ESP32_NODE_HOST env, then localhost (after rp-*.local).
    // 127.0.0.1 is the USB serial->TCP bridge (serial_tcp_bridge.py / run_usb_plugin_bridge.ps1),
    // so the bridge is found without manually setting a Node IP. Detection still requires a
    // valid handshake reply, so an unrelated localhost:5000 service won't be mistaken for a board.
    // 192.168.4.1 is the fixed Soft AP IP the firmware starts when STA join times out — no manual
    // Node IP entry needed when the PC joins the STEP_ESP32 hotspot.
    const String esp32Candidates[4] = { configurableNodeHost.trim(), envEsp32NodeHost(), "192.168.4.1", "127.0.0.1" };

    for (int h = 0; h < 4; ++h)
    {
        const String host = esp32Candidates[h];

        if (host.isEmpty())
            continue;

        resetCommandSocket();

        if (! connectCommandSocketToHost (host))
        {
            std::cout << "ESP32 node: connect failed: " << host << std::endl;
            continue;
        }

        activeRedPitayaHost = host;

        if (performDetectionHandshake())
            return true;

        resetCommandSocket();
        activeRedPitayaHost = {};
    }

    return false;
}

bool AcqBoardRedPitaya::retryDetection()
{
    deviceFound = false;
    isEsp32Node = false;
    activeRedPitayaHost = {};
    resetCommandSocket();
    return detectBoard();
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

    if (! connectCommandSocketToBoard())
    {
        std::cout << "Red Pitaya ERROR: Could not connect to board." << std::endl;
        return false;
    }

    if (isEsp32Node)
    {
        int targetHz = (int) settings.boardSampleRate;
        if (targetHz < 1)
            targetHz = 100;

        numAdcChannels = jmax (ESP32_DEFAULT_CHANNELS, numAdcChannels);
        settings.boardSampleRate = static_cast<float> (targetHz);
        lastRecordingPath = {};
        lastRecordingCsvPath = {};

        streamSensorNames.clear();

        // Helper: read one '\n'-terminated ASCII line from the command socket.
        // Returns false if no line arrived (timeout) or the next byte is binary
        // (frame data has started — the leading OeHeader byte is 0x00).
        auto readReplyLine = [this] (String& out, int firstByteTimeoutMs) -> bool
        {
            out.clear();

            if (! commandSocket->waitUntilReady (true, firstByteTimeoutMs))
                return false;

            char first = 0;
            if (commandSocket->read (&first, 1, false) <= 0)
                return false;

            if ((unsigned char) first < 0x20 || (unsigned char) first > 0x7E)
                return false; // binary stream has begun

            out += first;

            while (commandSocket->waitUntilReady (true, 1000))
            {
                char c = 0;
                if (commandSocket->read (&c, 1, false) <= 0 || c == '\n')
                    break;
                out += c;
            }

            return true;
        };

        // This is a fresh TCP session (we always reconnect above), so re-do the
        // handshake before START: the USB serial->TCP bridge (--plugin) requires a
        // REDPITAYA line on every new connection or it rejects START and closes the
        // socket — which looks like "acquisition starts but no data ever arrives".
        //
        // Send REDPITAYA and START as SEPARATE writes, reading the handshake reply in
        // between: the bridge peeks up to 256 bytes on connect, and if REDPITAYA and
        // START arrive together it consumes both in that peek and then waits forever for
        // a START it already swallowed. Reading the reply first forces START into its own
        // read on the bridge. The Wi-Fi firmware handles either ordering fine.
        const char* hello = "REDPITAYA\n";
        commandSocket->write (hello, (int) strlen (hello));

        // Drain the handshake reply ("8 channels…" + "OK CHANNELS:N"); stop after the
        // terminal line so START is sent only once the reply is consumed.
        String replyLine;
        for (int guard = 0; guard < 4; ++guard)
        {
            if (! readReplyLine (replyLine, 2000))
                break;

            if (replyLine.containsIgnoreCase ("CHANNELS") || replyLine.startsWith ("OK"))
                break; // last handshake line before START is expected
        }

        {
            char freqMsg[32];
            snprintf (freqMsg, sizeof (freqMsg), "FREQ:%d\n", targetHz);
            commandSocket->write (freqMsg, (int) strlen (freqMsg));
        }

        const char* startMsg = "START\n";
        commandSocket->write (startMsg, (int) strlen (startMsg));

        // Drain START reply lines ("STARTED …" then "SENSORS:…") up to and including the
        // terminal SENSORS line; binary frames begin immediately after it, so stopping
        // there keeps run()'s parser byte-aligned from the first header.
        for (int guard = 0; guard < 4; ++guard)
        {
            if (! readReplyLine (replyLine, 2000))
                break;

            if (replyLine.startsWith ("SENSORS:"))
            {
                const String body = replyLine.fromFirstOccurrenceOf ("SENSORS:", false, false).trim();
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

                break; // SENSORS is the last line before binary frames
            }
        }

        if (streamSensorNames.isEmpty())
            streamSensorNames.add ("ICM20948");

        // Relay is active after START; re-send FREQ so USB bridge → serial always applies
        // the rate matching settings.boardSampleRate (pre-START FREQ is also forwarded).
        // In WiFi mode the firmware replies "OK FREQ:N\n" / "OK FILTER ON\n"; drain those
        // replies here so run()'s frame parser never sees ASCII bytes at the start of the
        // stream. readReplyLine returns false immediately if a binary frame already arrived
        // (first byte < 0x20), so this is a no-op in that case.
        {
            char freqMsg[32];
            snprintf (freqMsg, sizeof (freqMsg), "FREQ:%d\n", targetHz);
            commandSocket->write (freqMsg, (int) strlen (freqMsg));
            String okFreq;
            readReplyLine (okFreq, 200);
        }

        {
            const char* filterMsg = filterEnabled ? "FILTER ON\n" : "FILTER OFF\n";
            commandSocket->write (filterMsg, (int) strlen (filterMsg));
            String okFilter;
            readReplyLine (okFilter, 200);
        }

        std::cout << "ESP32-S3: Streaming started (TCP binary, "
                  << numAdcChannels << " ch @ " << targetHz << " Hz)." << std::endl;

        startThread();
        return true;
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
    openSimAngleSocket.reset();

    {
        const juce::ScopedLock sl (liveAngleLock);
        liveAnglesValid = false;
    }

    if (openSimLiveProcess != nullptr)
    {
        openSimLiveProcess->kill();
        openSimLiveProcess.reset();
    }

    // Flush and close any in-progress PC-side recording so no data is lost.
    {
        const juce::ScopedLock sl (esp32RecordLock);
        if (esp32RecordStream != nullptr)
        {
            esp32RecordStream->flush();
            esp32RecordStream.reset();
        }
    }

    streamSensorNames.clear();

    return true;
}
bool AcqBoardRedPitaya::sendRecordOnCommand()
{
    if (isEsp32Node)
    {
        // ESP32 WiFi nodes have no board-side storage — record to a CSV on the PC instead.
        const juce::File dir (kEsp32RecordDir);
        dir.createDirectory();

        const String stamp = Time::getCurrentTime().formatted ("%Y%m%d_%H%M%S");
        const juce::File file = dir.getChildFile ("recording_" + stamp + ".csv");

        auto stream = std::make_unique<juce::FileOutputStream> (file);
        if (! stream->openedOk())
        {
            std::cout << "ESP32 record: could not open " << file.getFullPathName() << std::endl;
            return false;
        }

        stream->writeText ("timestamp_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,dio,qw,qx,qy,qz\n",
                           false, false, nullptr);

        lastRecordingPath = file.getFullPathName();
        lastRecordingCsvPath = {};

        const juce::ScopedLock sl (esp32RecordLock);
        esp32RecordStream     = std::move (stream);
        esp32RecordSampleCount = 0;
        std::cout << "ESP32 record: writing to " << lastRecordingPath << std::endl;
        return true;
    }

    // Red Pitaya: tell the board to save locally.
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
    return written > 0;
}

bool AcqBoardRedPitaya::sendRecordOffCommand()
{
    if (isEsp32Node)
    {
        const juce::ScopedLock sl (esp32RecordLock);
        if (esp32RecordStream != nullptr)
        {
            esp32RecordStream->flush();
            esp32RecordStream.reset();
            std::cout << "ESP32 record: closed " << lastRecordingPath << std::endl;
        }
        return true;
    }

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

    if (isEsp32Node)
    {
        if (newFreq < 1)
            return;

        settings.boardSampleRate = static_cast<float> (newFreq);

        if (commandSocket != nullptr && commandSocket->isConnected())
        {
            char msg[32];
            snprintf (msg, sizeof (msg), "FREQ:%d\n", newFreq);
            commandSocket->write (msg, (int) strlen (msg));
            std::cout << "ESP32-S3: Sent command -> " << msg;
        }

        return;
    }

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
        if (! connectCommandSocketToBoard())
        {
            std::cout << "Red Pitaya ERROR: Could not connect to board." << std::endl;
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

const char* AcqBoardRedPitaya::getBodySegmentName (int idx)
{
    if (idx < 0 || idx >= NUM_BODY_SEGMENTS) idx = 0;
    return kBodySegmentNames[idx];
}

const char* AcqBoardRedPitaya::getBodySegmentLabel (int idx)
{
    if (idx < 0 || idx >= NUM_BODY_SEGMENTS) idx = 0;
    return kBodySegmentLabels[idx];
}

void AcqBoardRedPitaya::setSensorBodySegment (int sensorIndex, int segmentIndex)
{
    if (sensorIndex >= 0 && sensorIndex < 6)
        sensorBodySegment[sensorIndex] = jlimit (0, NUM_BODY_SEGMENTS - 1, segmentIndex);
}

int AcqBoardRedPitaya::getSensorBodySegment (int sensorIndex) const
{
    if (sensorIndex < 0 || sensorIndex >= 6) return 0;
    return sensorBodySegment[sensorIndex];
}

bool AcqBoardRedPitaya::writeOpenSimSensorMap() const
{
    const juce::File mapFile (String (kOpenSimWorkDir) + "\\opensim_sensor_map.json");

    juce::FileOutputStream out (mapFile);
    if (! out.openedOk())
    {
        std::cout << "OpenSim sensor map: cannot write " << mapFile.getFullPathName() << std::endl;
        return false;
    }

    out.setPosition (0);
    out.truncate();

    const int n = jmax (1, streamSensorNames.size());
    out.writeText ("{\n"
                   "  \"comment\": \"Written by Open Ephys plugin. sensor_slots[i] = OpenSim body frame for sensor i.\",\n"
                   "  \"sensor_slots\": [\n",
                   false, false, nullptr);

    for (int i = 0; i < n; ++i)
    {
        const int seg  = jlimit (0, NUM_BODY_SEGMENTS - 1, sensorBodySegment[i]);
        const bool last = (i == n - 1);
        out.writeText (String ("    \"") + kBodySegmentNames[seg] + "\"" + (last ? "\n" : ",\n"),
                       false, false, nullptr);
    }

    out.writeText ("  ]\n}\n", false, false, nullptr);

    std::cout << "OpenSim sensor map: wrote " << n << " sensor(s) to " << mapFile.getFullPathName() << std::endl;
    return true;
}

void AcqBoardRedPitaya::setTargetKneeAngleDeg (float degrees)
{
    targetKneeAngleDeg = degrees;
}

void AcqBoardRedPitaya::setTargetHipAngleDeg (float degrees)
{
    targetHipAngleDeg = degrees;
}

bool AcqBoardRedPitaya::getLiveJointAngles (float& kneeDeg, float& hipDeg) const
{
    const juce::ScopedLock sl (liveAngleLock);

    if (! liveAnglesValid)
        return false;

    kneeDeg = liveKneeAngleDeg;
    hipDeg  = liveHipAngleDeg;
    return true;
}

bool AcqBoardRedPitaya::writeOpenSimTargetAngles() const
{
    const juce::File targetFile (String (kOpenSimWorkDir) + "\\opensim_target_angles.json");

    juce::FileOutputStream out (targetFile);
    if (! out.openedOk())
    {
        std::cout << "OpenSim target angles: cannot write " << targetFile.getFullPathName() << std::endl;
        return false;
    }

    out.setPosition (0);
    out.truncate();

    out.writeText ("{\n"
                   "  \"comment\": \"Written by Open Ephys plugin. Target joint angles in degrees.\",\n"
                   "  \"tolerance_deg\": " + String (targetAngleToleranceDeg, 1) + ",\n"
                   "  \"targets\": {\n"
                   "    \"knee_angle_r\": " + String (targetKneeAngleDeg, 2) + ",\n"
                   "    \"hip_flexion_r\": " + String (targetHipAngleDeg, 2) + ",\n"
                   "    \"pelvis_tilt\": 0.0,\n"
                   "    \"pelvis_list\": 0.0,\n"
                   "    \"pelvis_rotation\": 0.0\n"
                   "  }\n"
                   "}\n",
                   false, false, nullptr);

    std::cout << "OpenSim target angles: knee=" << targetKneeAngleDeg
              << " hip=" << targetHipAngleDeg
              << " -> " << targetFile.getFullPathName() << std::endl;
    return true;
}

void AcqBoardRedPitaya::pollOpenSimAngleFeedback()
{
    if (openSimAngleSocket == nullptr)
        return;

    constexpr int kPacketFloats = 7;
    float buffer[kPacketFloats];
    String sender;
    int senderPort = 0;

    while (openSimAngleSocket->read (buffer, (int) sizeof (buffer), false, sender, senderPort) == (int) sizeof (buffer))
    {
        const float version = buffer[1];

        if (version < 2.99f || version > 3.01f)
            continue;

        const juce::ScopedLock sl (liveAngleLock);
        liveHipAngleDeg  = buffer[2];
        liveKneeAngleDeg = buffer[3];
        liveAnglesValid  = true;
    }
}

void AcqBoardRedPitaya::launchOpenSimMotion()
{
    writeOpenSimSensorMap();
    writeOpenSimTargetAngles();
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
    writeOpenSimSensorMap();
    writeOpenSimTargetAngles();
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
    openSimAngleSocket = std::make_unique<DatagramSocket>();

    if (openSimAngleSocket->bindToPort (5001))
        std::cout << "Red Pitaya: OpenSim angle feedback listening on UDP port 5001" << std::endl;
    else
        std::cout << "Red Pitaya: OpenSim angle feedback bind failed on port 5001" << std::endl;

    {
        const juce::ScopedLock sl (liveAngleLock);
        liveAnglesValid = false;
    }

    openSimEnabled = true;
    std::cout << "Red Pitaya: OpenSim UDP forwarding enabled -> 127.0.0.1:5000" << std::endl;
}

void AcqBoardRedPitaya::sendOpenSimQuaternionPacket (float timestamp, const float* quats, int numSensors)
{
    if (! openSimEnabled || openSimSocket == nullptr || quats == nullptr || numSensors < 1)
        return;

    const int totalFloats = 3 + numSensors * 4;
    juce::HeapBlock<float> pkt (totalFloats);
    pkt[0] = timestamp;
    pkt[1] = 2.0f;
    pkt[2] = (float) numSensors;

    for (int i = 0; i < numSensors * 4; ++i)
        pkt[3 + i] = quats[i];

    openSimSocket->write ("127.0.0.1", 5000, pkt, totalFloats * (int) sizeof (float));

    static int openSimUdpSendCount = 0;
    ++openSimUdpSendCount;

    if (openSimUdpSendCount == 1 || (openSimUdpSendCount % 1000) == 0)
    {
        std::cout << "OpenSim UDP v2 sent #" << openSimUdpSendCount
                  << " t=" << timestamp
                  << " n_sensors=" << numSensors
                  << " qw[0]=" << quats[0] << std::endl;
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

    // ESP32-S3: binary samples on the same TCP socket after START (not UDP 55001).
    if (isEsp32Node)
    {
        constexpr int headerSize = 22;
        const float accScale = 1.0f / kAccSensitivity[jlimit (0, 3, sensorAccPreset[0])];
        const float gyrScale = 1.0f / kGyrSensitivity[jlimit (0, 3, sensorGyrPreset[0])];
        Array<uint8> tcpRx;
        double lastFrameWallSec = 0.0;
        uint32_t lastHwUs32 = 0;
        bool haveLastHw = false;

        while (! threadShouldExit())
        {
            DataBuffer* currentBuffer = buffer;

            if (currentBuffer == nullptr)
            {
                Thread::sleep (1);
                continue;
            }

            if (commandSocket->waitUntilReady (true, 100))
            {
                char chunk[4096];
                const int nRead = commandSocket->read (chunk, (int) sizeof (chunk), false);

                if (nRead <= 0)
                    break; // peer closed or socket error — exit acquisition loop

                tcpRx.insertArray (tcpRx.size(), (const uint8*) chunk, nRead);
            }

            while (tcpRx.size() >= (size_t) headerSize && ! threadShouldExit())
            {
                int32_t numBytes = 0;
                int32_t elemType = 0;
                std::memcpy (&numBytes, tcpRx.getRawDataPointer() + 4, 4);
                std::memcpy (&elemType, tcpRx.getRawDataPointer() + 10, 4);

                if (elemType != 2 || numBytes < 2 || numBytes > 65507 - headerSize
                    || (numBytes & 1) != 0)
                {
                    tcpRx.removeRange (0, 1);
                    continue;
                }

                const int totalFrame = headerSize + (int) numBytes;

                if (tcpRx.size() < (size_t) totalFrame)
                    break;

                const int16_t* channels = reinterpret_cast<const int16_t*> (
                    tcpRx.getRawDataPointer() + headerSize);
                const int channelsInPacket = (int) numBytes / 2;
                const int sampleIndex = (int) (sampleNumber % samplesPerBuffer);

                for (int adc = 0; adc < numAdcChannelsLocal; ++adc)
                {
                    float raw = 0.0f;

                    if (adc < channelsInPacket)
                    {
                        if (adc <= 2)
                            raw = float (channels[adc]) * accScale;
                        else if (adc <= 5)
                            raw = float (channels[adc]) * gyrScale;
                        else if (adc == 6)
                            raw = float (channels[adc] & 1); // DIO level bit0
                        else
                            raw = float (channels[adc]); // filter quat Q15 (ch7–10)
                    }

                    samples[(adc * samplesPerBuffer) + sampleIndex] = raw;
                }

                // One physical IMU → one quaternion in UDP v2 (numSensors=1). Do not map
                // gyro raw counts into quat slots (legacy 8-ch layout); that drove multiple
                // OpenSim segments from a single bad orientation.
                if (openSimEnabled && filterEnabled && channelsInPacket >= 6)
                {
                    float qw = 0.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;

                    if (channelsInPacket >= 11)
                    {
                        quaternionFromScaledQ15 (float (channels[7]), float (channels[8]),
                                                 float (channels[9]), float (channels[10]),
                                                 qw, qx, qy, qz);
                    }
                    else
                    {
                        const float dt = 1.0f / jmax (1.0f, settings.boardSampleRate);
                        float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
                        constexpr float kDegToRad = 0.017453292519943295f;
                        const float gx = float (channels[3]) * gyrScale * kDegToRad;
                        const float gy = float (channels[4]) * gyrScale * kDegToRad;
                        const float gz = float (channels[5]) * gyrScale * kDegToRad;
                        const float ax = float (channels[0]) * accScale;
                        const float ay = float (channels[1]) * accScale;
                        const float az = float (channels[2]) * accScale;
                        madgwickUpdate6DOF (q, gx, gy, gz, ax, ay, az, dt, 0.1f);
                        qw = q[0];
                        qx = q[1];
                        qy = q[2];
                        qz = q[3];
                    }

                    const float n2 = qw * qw + qx * qx + qy * qy + qz * qz;

                    if (n2 > 1.0e-6f)
                    {
                        const float quatPkt[4] = { qw, qx, qy, qz };

                        if (sampleNumber == 0)
                        {
                            std::cout << "ESP32-S3: OpenSim UDP v2 n_sensors=1 (filter on), qw="
                                      << qw << std::endl;
                        }

                        sendOpenSimQuaternionPacket ((float) elapsedSeconds, quatPkt, 1);
                    }
                }

                // PC-side CSV recording for ESP32 WiFi mode.
                // Use a counter-based timestamp (sample index / nominal rate) rather than
                // elapsedSeconds: TCP delivers frames in bursts so wall-clock dt is unreliable —
                // consecutive frames in the same burst all arrive at nearly the same instant and
                // get clamped to the 2 ms minimum, producing uneven timestamps. The counter gives
                // perfectly even spacing at the configured sample rate.
                {
                    const juce::ScopedLock sl (esp32RecordLock);
                    if (esp32RecordStream != nullptr)
                    {
                        const float rax  = channelsInPacket > 0 ? float (channels[0]) * accScale : 0.0f;
                        const float ray  = channelsInPacket > 1 ? float (channels[1]) * accScale : 0.0f;
                        const float raz  = channelsInPacket > 2 ? float (channels[2]) * accScale : 0.0f;
                        const float rgx  = channelsInPacket > 3 ? float (channels[3]) * gyrScale : 0.0f;
                        const float rgy  = channelsInPacket > 4 ? float (channels[4]) * gyrScale : 0.0f;
                        const float rgz  = channelsInPacket > 5 ? float (channels[5]) * gyrScale : 0.0f;
                        const float rdio = channelsInPacket > 6 ? float (channels[6] & 1) : 0.0f;

                        float rqw = 0.0f, rqx = 0.0f, rqy = 0.0f, rqz = 0.0f;
                        if (filterEnabled && channelsInPacket >= 11)
                        {
                            constexpr float kQ15inv = 1.0f / 32767.0f;
                            rqw = float (channels[7])  * kQ15inv;
                            rqx = float (channels[8])  * kQ15inv;
                            rqy = float (channels[9])  * kQ15inv;
                            rqz = float (channels[10]) * kQ15inv;
                        }

                        const double nominalRate = jmax (1.0, static_cast<double> (settings.boardSampleRate));
                        const double csvTimestamp = static_cast<double> (esp32RecordSampleCount) / nominalRate;

                        char row[192];
                        const int len = snprintf (row, sizeof (row),
                            "%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.0f,%.6f,%.6f,%.6f,%.6f\n",
                            csvTimestamp, rax, ray, raz, rgx, rgy, rgz, rdio,
                            rqw, rqx, rqy, rqz);

                        if (len > 0 && len < (int) sizeof (row))
                            esp32RecordStream->write (row, (size_t) len);

                        // Flush every 100 samples (~1 s at 100 Hz) so data survives a crash.
                        if (++esp32RecordSampleCount % 100 == 0)
                            esp32RecordStream->flush();
                    }
                }

                const double currentSampleRate = jmax (1.0, static_cast<double> (settings.boardSampleRate));
                const double wallSec = Time::getMillisecondCounterHiRes() * 0.001;
                double dt = 1.0 / currentSampleRate;

                int32_t frameOffsetUs = 0;
                std::memcpy (&frameOffsetUs, tcpRx.getRawDataPointer(), 4);
                const double hwDt = deltaSecFromHwOffsetUs (frameOffsetUs, lastHwUs32, haveLastHw);

                if (hwDt > 0.0)
                    dt = jlimit (1.0 / 5000.0, 0.5, hwDt);
                else if (lastFrameWallSec > 0.0)
                    dt = jlimit (1.0 / 500.0, 0.5, wallSec - lastFrameWallSec);

                lastFrameWallSec = wallSec;

                sampleNumbers[sampleIndex] = sampleNumber;
                timestamps[sampleIndex]    = elapsedSeconds;
                event_codes[sampleIndex]   = eventCode;

                ++sampleNumber;
                elapsedSeconds += dt;

                if (sampleIndex == (int) samplesPerBuffer - 1)
                {
                    if (openSimEnabled)
                        pollOpenSimAngleFeedback();

                    currentBuffer = buffer;
                    if (currentBuffer != nullptr && ! threadShouldExit())
                        currentBuffer->addToBuffer (samples,
                                                    sampleNumbers,
                                                    timestamps,
                                                    event_codes,
                                                    (int) samplesPerBuffer);
                }

                tcpRx.removeRange (0, totalFrame);
            }
        }

        return;
    }

    // Red Pitaya: dedicated UDP socket on port 55001 (unchanged).
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

            if (openSimEnabled)
            {
                const int numSensors = streamSensorNames.size();

                if (numSensors >= 1)
                {
                    float quatPkt[MAX_CHANNELS];
                    int   nQuat = 0;

                    for (int si = 0; si < numSensors && si < 6; ++si)
                    {
                        const String& sname = streamSensorNames.getReference (si);
                        const int     off     = channelOffsetForSensorIndex (streamSensorNames, si);
                        const int     numRaw  = rawChannelCountForSensorName (sname);
                        const int     q0      = off + numRaw;

                        const auto qSample = [&](int qIdx) -> float {
                            const int ch = q0 + qIdx;

                            if (ch >= numAdcChannelsLocal)
                                return 0.0f;

                            return samples[(ch * samplesPerBuffer) + sampleIndex];
                        };

                        float qw, qx, qy, qz;
                        quaternionFromScaledQ15 (qSample (0), qSample (1), qSample (2), qSample (3),
                                                 qw, qx, qy, qz);

                        quatPkt[nQuat++] = qw;
                        quatPkt[nQuat++] = qx;
                        quatPkt[nQuat++] = qy;
                        quatPkt[nQuat++] = qz;
                    }

                    if (sampleNumber == 0)
                    {
                        std::cout << "Red Pitaya: OpenSim UDP v2, " << numSensors
                                  << " sensor(s), qw0=" << quatPkt[0] << std::endl;
                    }

                    sendOpenSimQuaternionPacket ((float) elapsedSeconds, quatPkt, numSensors);
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
                if (openSimEnabled)
                    pollOpenSimAngleFeedback();

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

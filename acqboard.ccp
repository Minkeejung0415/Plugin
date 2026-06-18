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

    const char* kOpenSimWorkDir = "C:\\Users\\justi\\Open-Sim--Bio-Mech";

    const char* kEsp32RecordDir = "C:\\Users\\justi\\Documents\\Arduino\\ESP32-S3-1\\results";

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

    const char* kDisplayJointNames[] = {
        "hip_flexion_r",
        "knee_angle_r",
        "ankle_angle_r",
        "hip_flexion_l",
        "knee_angle_l",
        "ankle_angle_l",
        "pelvis_tilt",
        "pelvis_list",
        "pelvis_rotation",
        "lumbar_extension",
    };
    const char* kDisplayJointLabels[] = {
        "Right Hip Flexion",
        "Right Knee",
        "Right Ankle",
        "Left Hip Flexion",
        "Left Knee",
        "Left Ankle",
        "Pelvis Tilt",
        "Pelvis List",
        "Pelvis Rotation",
        "Lumbar Extension",
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

    // ESP32-S3 STEP node: "14 channels; sample_rate=100; node=esp32s3_arduino".
    // Check this BEFORE the generic Red Pitaya "OK" branch: the ESP32 firmware and
    // the USB bridge both follow the esp32 marker line with "OK CHANNELS:N", and over
    // a localhost/TCP connection the two writes usually coalesce into a single read.
    // If we tested for "OK" first, that "OK CHANNELS:N" would misclassify the ESP32 as
    // a Red Pitaya, sending run() down the UDP:55001 path while the board streams
    // binary over TCP — i.e. a successful "detect" that never produces any samples.
    if (isEsp32HandshakeResponse (response))
    {
        isEsp32Node = true;
        esp32UsesUdpStream = response.containsIgnoreCase ("transport=udp");
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
                  << (int) settings.boardSampleRate << " Hz ("
                  << (esp32UsesUdpStream ? "UDP samples + TCP control" : "TCP stream")
                  << ")." << std::endl;

        // Negotiate rec-v1 SD recording protocol.  sendEsp32RecHello() also checks for
        // any timeout-finalized session retained by firmware (D-10).
        esp32RecV1Supported = sendEsp32RecHello();

        return true;
    }

    // Red Pitaya firmware: "OK CHANNELS:N" (no esp32 markers)
    if (response.contains ("OK"))
    {
        isEsp32Node = false;
        esp32UsesUdpStream = false;

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

        // Drain the handshake reply ("14 channels..." + "OK CHANNELS:N"); stop after the
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

        std::cout << "ESP32-S3: Streaming started ("
                  << (esp32UsesUdpStream ? "UDP samples + TCP control" : "TCP binary")
                  << ", " << numAdcChannels << " ch @ " << targetHz << " Hz)."
                  << std::endl;

        startThread();
        writeJointDisplayConfig();
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
    writeJointDisplayConfig();
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
        return sendEsp32RecStart();
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
        return sendEsp32RecStop();
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

const char* AcqBoardRedPitaya::getDisplayJointName (int idx)
{
    if (idx < 0 || idx >= NUM_DISPLAY_JOINTS) idx = 0;
    return kDisplayJointNames[idx];
}

const char* AcqBoardRedPitaya::getDisplayJointLabel (int idx)
{
    if (idx < 0 || idx >= NUM_DISPLAY_JOINTS) idx = 0;
    return kDisplayJointLabels[idx];
}

void AcqBoardRedPitaya::setDisplayJointIndex (int index)
{
    displayJointIndex = jlimit (0, NUM_DISPLAY_JOINTS - 1, index);
}

bool AcqBoardRedPitaya::getLiveDisplayAngle (float& angleDeg) const
{
    const juce::ScopedLock sl (liveAngleLock);

    if (! liveAnglesValid)
        return false;

    angleDeg = liveDisplayAngleDeg;
    return true;
}

bool AcqBoardRedPitaya::writeOpenSimDisplayJoint() const
{
    const juce::File jointFile (String (kOpenSimWorkDir) + "\\opensim_display_joint.json");
    const int idx = jlimit (0, NUM_DISPLAY_JOINTS - 1, displayJointIndex);

    juce::FileOutputStream out (jointFile);
    if (! out.openedOk())
    {
        std::cout << "OpenSim display joint: cannot write " << jointFile.getFullPathName() << std::endl;
        return false;
    }

    out.setPosition (0);
    out.truncate();

    out.writeText ("{\n"
                   "  \"comment\": \"Written by Open Ephys plugin. Joint shown on the OpenSim screen.\",\n"
                   "  \"joint_index\": " + String (idx) + ",\n"
                   "  \"joint\": \"" + String (kDisplayJointNames[idx]) + "\",\n"
                   "  \"label\": \"" + String (kDisplayJointLabels[idx]) + "\"\n"
                   "}\n",
                   false, false, nullptr);

    std::cout << "OpenSim display joint: " << kDisplayJointLabels[idx]
              << " (" << kDisplayJointNames[idx] << ") -> "
              << jointFile.getFullPathName() << std::endl;
    return true;
}

void AcqBoardRedPitaya::pollOpenSimAngleFeedback()
{
    if (openSimAngleSocket == nullptr)
        return;

    constexpr int kPacketFloatsV31 = 4;
    float buffer[kPacketFloatsV31];
    String sender;
    int senderPort = 0;

    while (openSimAngleSocket->read (buffer, (int) sizeof (buffer), false, sender, senderPort) == (int) sizeof (buffer))
    {
        const float version = buffer[1];

        if (version < 3.09f || version > 3.11f)
            continue;

        const int jointIndex = jlimit (0, NUM_DISPLAY_JOINTS - 1, (int) std::lround (buffer[2]));

        if (jointIndex != displayJointIndex)
            continue;

        const juce::ScopedLock sl (liveAngleLock);
        liveDisplayAngleDeg = buffer[3];
        liveAnglesValid     = true;
    }
}

String AcqBoardRedPitaya::getOpenSimWorkDir()
{
    return kOpenSimWorkDir;
}

const OpenSimJointCatalogEntry& AcqBoardRedPitaya::getJointCatalogEntry (int index) const
{
    jassert (index >= 0 && index < kOpenSimJointCatalogSize);
    return kOpenSimJointCatalog[index];
}

bool AcqBoardRedPitaya::isJointDisplaySelected (int catalogIndex) const
{
    if (catalogIndex < 0 || catalogIndex >= kOpenSimJointCatalogSize)
        return false;

    return jointDisplaySelected[(size_t) catalogIndex];
}

void AcqBoardRedPitaya::setJointDisplaySelected (int catalogIndex, bool selected)
{
    if (catalogIndex < 0 || catalogIndex >= kOpenSimJointCatalogSize)
        return;

    if (selected)
    {
        int count = 0;

        for (bool on : jointDisplaySelected)
            if (on)
                ++count;

        if (! jointDisplaySelected[(size_t) catalogIndex] && count >= kMaxJointDisplaySelection)
            return;
    }

    jointDisplaySelected[(size_t) catalogIndex] = selected;
}

StringArray AcqBoardRedPitaya::getSelectedDisplayJoints() const
{
    StringArray joints;

    for (int i = 0; i < kOpenSimJointCatalogSize; ++i)
    {
        if (jointDisplaySelected[(size_t) i])
            joints.add (kOpenSimJointCatalog[i].coordinate);
    }

    return joints;
}

void AcqBoardRedPitaya::loadJointDisplayFromXml (const XmlElement& parent)
{
    jointDisplaySelected.fill (false);

    if (auto* jointDisplay = parent.getChildByName ("JOINT_DISPLAY"))
    {
        for (auto* joint : jointDisplay->getChildIterator())
        {
            if (! joint->hasTagName ("JOINT"))
                continue;

            const String coordinate = joint->getStringAttribute ("coordinate");
            const bool selected = joint->getBoolAttribute ("selected", false);

            for (int i = 0; i < kOpenSimJointCatalogSize; ++i)
            {
                if (coordinate == kOpenSimJointCatalog[i].coordinate)
                {
                    if (selected)
                        setJointDisplaySelected (i, true);

                    break;
                }
            }
        }
    }

    jointDisplayConfigSeq = parent.getIntAttribute ("JointDisplaySeq", jointDisplayConfigSeq);
}

void AcqBoardRedPitaya::saveJointDisplayToXml (XmlElement& parent) const
{
    parent.setAttribute ("JointDisplaySeq", jointDisplayConfigSeq);

    if (auto* existing = parent.getChildByName ("JOINT_DISPLAY"))
        parent.removeChildElement (existing, true);

    auto* jointDisplay = parent.createNewChildElement ("JOINT_DISPLAY");

    for (int i = 0; i < kOpenSimJointCatalogSize; ++i)
    {
        auto* joint = jointDisplay->createNewChildElement ("JOINT");
        joint->setAttribute ("coordinate", kOpenSimJointCatalog[i].coordinate);
        joint->setAttribute ("selected", jointDisplaySelected[(size_t) i]);
    }
}

bool AcqBoardRedPitaya::writeJointDisplayConfig()
{
    StringArray joints = getSelectedDisplayJoints();

    if (joints.isEmpty())
        joints.add (kDefaultJointDisplayCoordinate);

    const int seq = ++jointDisplayConfigSeq;

    DynamicObject::Ptr root = new DynamicObject();
    Array<var> jointVars;

    for (const auto& name : joints)
        jointVars.add (name);

    root->setProperty ("joints", jointVars);
    root->setProperty ("trigger_ts", Time::getMillisecondCounterHiRes() / 1000.0);
    root->setProperty ("seq", seq);

    const String json = JSON::toString (var (root.get()), true) + "\n";

    const File workDir (kOpenSimWorkDir);
    const File target = workDir.getChildFile ("opensim_joint_display_config.json");
    const File temp = workDir.getChildFile ("opensim_joint_display_config.json.tmp");

    if (! workDir.createDirectory())
    {
        std::cout << "Joint display config: could not create work dir " << workDir.getFullPathName() << std::endl;
        return false;
    }

    if (! temp.replaceWithText (json))
    {
        std::cout << "Joint display config: failed to write temp file" << std::endl;
        return false;
    }

    if (! temp.moveFileTo (target))
    {
        std::cout << "Joint display config: atomic rename failed" << std::endl;
        return false;
    }

    std::cout << "Joint display config written seq=" << seq
              << " joints=" << joints.joinIntoString (", ") << std::endl;
    return true;
}

void AcqBoardRedPitaya::launchOpenSimMotion()
{
    writeOpenSimSensorMap();
    writeOpenSimDisplayJoint();
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
    writeOpenSimDisplayJoint();
    const juce::String workDir = kOpenSimWorkDir;
    const juce::String scriptPath = workDir + "\\opensim_live_realtime.py";

    const File repoRoot = File::getCurrentWorkingDirectory();
    const File catalogSrc = repoRoot.getChildFile ("opensim_joint_catalog.py");
    const File catalogDst = File (workDir).getChildFile ("opensim_joint_catalog.py");

    if (catalogSrc.existsAsFile())
        catalogSrc.copyFileTo (catalogDst);

    const File liveSrc = repoRoot.getChildFile ("opensim_live_realtime.py");
    const File liveDst = File (workDir).getChildFile ("opensim_live_realtime.py");

    if (liveSrc.existsAsFile())
        liveSrc.copyFileTo (liveDst);

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
    writeJointDisplayConfig();
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

void AcqBoardRedPitaya::sendOpenSimImuPacket (float timestamp, const float* imu6, int numSensors)
{
    if (! openSimEnabled || openSimSocket == nullptr || imu6 == nullptr || numSensors < 1)
        return;

    const int totalFloats = 3 + numSensors * 6;
    juce::HeapBlock<float> pkt (totalFloats);
    pkt[0] = timestamp;
    pkt[1] = 3.0f;
    pkt[2] = (float) numSensors;

    for (int i = 0; i < numSensors * 6; ++i)
        pkt[3 + i] = imu6[i];

    openSimSocket->write ("127.0.0.1", 5000, pkt, totalFloats * (int) sizeof (float));

    static int openSimImuSendCount = 0;
    ++openSimImuSendCount;

    if (openSimImuSendCount == 1 || (openSimImuSendCount % 1000) == 0)
    {
        std::cout << "OpenSim UDP v3 sent #" << openSimImuSendCount
                  << " t=" << timestamp
                  << " n_sensors=" << numSensors
                  << " ax[0]=" << imu6[0] << std::endl;
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

    // ESP32-S3: UDP-capable firmware keeps TCP for control; legacy firmware streams samples on TCP.
    if (isEsp32Node)
    {
        constexpr int headerSize = 22;
        const float accScale = 1.0f / kAccSensitivity[jlimit (0, 3, sensorAccPreset[0])];
        const float gyrScale = 1.0f / kGyrSensitivity[jlimit (0, 3, sensorGyrPreset[0])];
        constexpr float magScale = 0.15f;
        Array<uint8> tcpRx;
        double lastFrameWallSec = 0.0;
        uint32_t lastHwUs32 = 0;
        bool haveLastHw = false;
        DatagramSocket esp32UdpSocket;
        uint8_t udpDatagram[65507];

        if (esp32UsesUdpStream && ! esp32UdpSocket.bindToPort (55001))
        {
            std::cout << "ESP32 ERROR: Failed to bind UDP sample port 55001" << std::endl;
            return;
        }

        while (! threadShouldExit())
        {
            DataBuffer* currentBuffer = buffer;

            if (currentBuffer == nullptr)
            {
                Thread::sleep (1);
                continue;
            }

            if (esp32UsesUdpStream)
            {
                if (esp32UdpSocket.waitUntilReady (true, 100))
                {
                    const int nRead = esp32UdpSocket.read (
                        (char*) udpDatagram, (int) sizeof (udpDatagram), false);

                    if (nRead < headerSize)
                        continue;

                    int32_t numBytes = 0;
                    uint16_t bitDepth = 0;
                    int32_t elemType = 0;
                    int32_t channelsInHeader = 0;
                    int32_t samplesInHeader = 0;
                    std::memcpy (&numBytes, udpDatagram + 4, 4);
                    std::memcpy (&bitDepth, udpDatagram + 8, 2);
                    std::memcpy (&elemType, udpDatagram + 10, 4);
                    std::memcpy (&channelsInHeader, udpDatagram + 14, 4);
                    std::memcpy (&samplesInHeader, udpDatagram + 18, 4);

                    const bool valid = elemType == 2
                                       && (bitDepth == 3 || bitDepth == 16)
                                       && samplesInHeader == 1
                                       && channelsInHeader > 0
                                       && channelsInHeader <= MAX_CHANNELS
                                       && numBytes == channelsInHeader * elemType
                                       && nRead == headerSize + numBytes;

                    if (! valid)
                        continue;

                    tcpRx.insertArray (tcpRx.size(), udpDatagram, nRead);
                }
            }
            else if (commandSocket->waitUntilReady (true, 100))
            {
                char chunk[4096];
                const int nRead = commandSocket->read (chunk, (int) sizeof (chunk), false);

                if (nRead <= 0)
                    break;

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
                        else if (adc <= 8)
                            raw = float (channels[adc]) * magScale;
                        else
                            raw = float (channels[adc]); // filter quat Q15 (ch9-12)
                    }

                    samples[(adc * samplesPerBuffer) + sampleIndex] = raw;
                }

                // OpenSim Live: forward raw accel/gyro (UDP v3) so Python can run 3 s baseline
                // calibration before AHRS → IK. One physical IMU → numSensors=1.
                if (openSimEnabled && channelsInPacket >= 6)
                {
                    if (filterEnabled && channelsInPacket >= 13)
                    {
                        constexpr float kQ15inv = 1.0f / 32767.0f;
                        const float quatPkt[4] = {
                            float (channels[9])  * kQ15inv,
                            float (channels[10]) * kQ15inv,
                            float (channels[11]) * kQ15inv,
                            float (channels[12]) * kQ15inv
                        };

                        if (sampleNumber == 0)
                        {
                            std::cout << "ESP32-S3: OpenSim UDP v2 n_sensors=1, qw="
                                      << quatPkt[0] << std::endl;
                        }

                        sendOpenSimQuaternionPacket ((float) elapsedSeconds, quatPkt, 1);
                    }
                    else
                    {
                        const float imuPkt[6] = {
                            float (channels[0]) * accScale,
                            float (channels[1]) * accScale,
                            float (channels[2]) * accScale,
                            float (channels[3]) * gyrScale,
                            float (channels[4]) * gyrScale,
                            float (channels[5]) * gyrScale,
                        };

                        if (sampleNumber == 0)
                        {
                            std::cout << "ESP32-S3: OpenSim UDP v3 n_sensors=1, ax="
                                      << imuPkt[0] << std::endl;
                        }

                        sendOpenSimImuPacket ((float) elapsedSeconds, imuPkt, 1);
                    }
                }

                // PC-side CSV recording for ESP32 WiFi mode (legacy path).
                // Only runs when rec-v1 is NOT supported (old firmware without SD recording).
                // When esp32RecV1Supported == true, SD recording is handled by the
                // sendEsp32RecStart/Stop/retrieval path and this block must be skipped.
                if (! esp32RecV1Supported)
                {
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
                        const float rmx  = channelsInPacket > 6 ? float (channels[6]) * magScale : 0.0f;
                        const float rmy  = channelsInPacket > 7 ? float (channels[7]) * magScale : 0.0f;
                        const float rmz  = channelsInPacket > 8 ? float (channels[8]) * magScale : 0.0f;

                        float rqw = 0.0f, rqx = 0.0f, rqy = 0.0f, rqz = 0.0f;
                        if (filterEnabled && channelsInPacket >= 13)
                        {
                            constexpr float kQ15inv = 1.0f / 32767.0f;
                            rqw = float (channels[9])  * kQ15inv;
                            rqx = float (channels[10]) * kQ15inv;
                            rqy = float (channels[11]) * kQ15inv;
                            rqz = float (channels[12]) * kQ15inv;
                        }

                        const double nominalRate = jmax (1.0, static_cast<double> (settings.boardSampleRate));
                        const double csvTimestamp = static_cast<double> (esp32RecordSampleCount) / nominalRate;

                        char row[224];
                        const int len = snprintf (row, sizeof (row),
                            "%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f\n",
                            csvTimestamp, rax, ray, raz, rgx, rgy, rgz, rmx, rmy, rmz,
                            rqw, rqx, rqy, rqz);

                        if (len > 0 && len < (int) sizeof (row))
                            esp32RecordStream->write (row, (size_t) len);

                        // Flush every 100 samples (~1 s at 100 Hz) so data survives a crash.
                        if (++esp32RecordSampleCount % 100 == 0)
                            esp32RecordStream->flush();
                    }
                }
                } // end !esp32RecV1Supported guard

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
                    float imuPkt[MAX_CHANNELS];
                    int   nImu = 0;

                    for (int si = 0; si < numSensors && si < 6; ++si)
                    {
                        const String& sname = streamSensorNames.getReference (si);
                        const int     off     = channelOffsetForSensorIndex (streamSensorNames, si);
                        const int     gyrOff  = (sname == "BNO055") ? off + 6 : off + 3;

                        const auto sampleCh = [&](int ch) -> float {
                            if (ch >= numAdcChannelsLocal)
                                return 0.0f;

                            return samples[(ch * samplesPerBuffer) + sampleIndex];
                        };

                        imuPkt[nImu++] = sampleCh (off + 0);
                        imuPkt[nImu++] = sampleCh (off + 1);
                        imuPkt[nImu++] = sampleCh (off + 2);
                        imuPkt[nImu++] = sampleCh (gyrOff + 0);
                        imuPkt[nImu++] = sampleCh (gyrOff + 1);
                        imuPkt[nImu++] = sampleCh (gyrOff + 2);
                    }

                    if (sampleNumber == 0)
                    {
                        std::cout << "Red Pitaya: OpenSim UDP v3, " << numSensors
                                  << " sensor(s), ax0=" << imuPkt[0] << std::endl;
                    }

                    sendOpenSimImuPacket ((float) elapsedSeconds, imuPkt, numSensors);
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

// =============================================================================
// rec-v1 SD recording protocol implementation
// =============================================================================

namespace
{
    /** Read a single line (up to '\n') from commandSocket with the given timeout ms.
     *  Returns the line string (without '\n') or an empty string on timeout/error. */
    static String readSocketLine (StreamingSocket* sock, int timeoutMs)
    {
        if (sock == nullptr)
            return {};

        String line;
        const int64 deadline = Time::currentTimeMillis() + timeoutMs;

        while (Time::currentTimeMillis() < deadline)
        {
            if (! sock->waitUntilReady (true, 50))
                continue;

            char c = 0;
            if (sock->read (&c, 1, false) <= 0)
                break;

            if (c == '\n')
                break;

            if (c >= 0x20 || c == '\t')
                line += c;
        }

        return line;
    }

    /** Parse a key=value token from a rec-v1 response string.
     *  E.g. parseRecField("REC HELLO_OK max_chunk=4096 ...", "max_chunk") → "4096" */
    static String parseRecField (const String& line, const String& key)
    {
        const String needle = key + "=";
        const int idx = line.indexOf (needle);
        if (idx < 0)
            return {};
        String val = line.substring (idx + needle.length());
        // Trim at first whitespace
        const int sp = val.indexOfAnyOf (" \t\r\n");
        if (sp >= 0)
            val = val.substring (0, sp);
        return val;
    }

    // Standard (Ethernet / PKZIP) CRC-32 table.
    static const uint32_t kCrc32Table[256] = {
        0x00000000,0x77073096,0xEE0E612C,0x990951BA,0x076DC419,0x706AF48F,0xE963A535,0x9E6495A3,
        0x0EDB8832,0x79DCB8A4,0xE0D5E91B,0x97D2D988,0x09B64C2B,0x7EB17CBF,0xE7B82D09,0x90BF1D95,
        0x1DB71064,0x6AB020F2,0xF3B97148,0x84BE41DE,0x1ADAD47D,0x6DDDE4EB,0xF4D4B551,0x83D385C7,
        0x136C9856,0x646BA8C0,0xFD62F97A,0x8A65C9EC,0x14015C4F,0x63066CD9,0xFA0F3D63,0x8D080DF5,
        0x3B6E20C8,0x4C69105E,0xD56041E4,0xA2677172,0x3C03E4D1,0x4B04D447,0xD20D85FD,0xA50AB56B,
        0x35B5A8FA,0x42B2986C,0xDBBBC9D6,0xACBCF940,0x32D86CE3,0x45DF5C75,0xDCD60DCF,0xABD13D59,
        0x26D930AC,0x51DE003A,0xC8D75180,0xBFD06116,0x21B4F93B,0x56B3C423,0xCFBA9599,0xB8BDA50F,
        0x2802B89E,0x5F058808,0xC60CD9B2,0xB10BE924,0x2F6F7C87,0x58684C11,0xC1611DAB,0xB6662D3D,
        0x76DC4190,0x01DB7106,0x98D220BC,0xEFD5102A,0x71B18589,0x06B6B51F,0x9FBFE4A5,0xE8B8D433,
        0x7807C9A2,0x0F00F934,0x9609A88E,0xE10E9818,0x7F6AD9BB,0x086D3D2D,0x91646C97,0xE6635C01,
        0x6B6B51F4,0x1C6C6162,0x856530D8,0xF262004E,0x6C0695ED,0x1B01A57B,0x8208F4C1,0xF50FC457,
        0x65B0D9C6,0x12B7E950,0x8BBEB8EA,0xFCB9887C,0x62DD1FDF,0x15DA2D49,0x8CD37CF3,0xFBD44C65,
        0x4DB26158,0x3AB551CE,0xA3BC0074,0xD4BB30E2,0x4ADFA541,0x3DD895D7,0xA4D1C46D,0xD3D6F4FB,
        0x4369E96A,0x346ED9FC,0xAD678846,0xDA60B8D0,0x44042D73,0x33031DE5,0xAA0A4C5F,0xDD0D7CC9,
        0x5005713C,0x270241AA,0xBE0B1010,0xC90C2086,0x5768B525,0x206F85B3,0xB966D409,0xCE61E49F,
        0x5EDEF90E,0x29D9C998,0xB0D09822,0xC7D7A8B4,0x59B33D17,0x2EB40D81,0xB7BD5C3B,0xC0BA6CAD,
        0xEDB88320,0x9ABFB3B6,0x03B6E20C,0x74B1D29A,0xEAD54739,0x9DD277AF,0x04DB2615,0x73DC1683,
        0xE3630B12,0x94643B84,0x0D6D6A3E,0x7A6A5AA8,0xE40ECF0B,0x9309FF9D,0x0A00AE27,0x7D079EB1,
        0xF00F9344,0x8708A3D2,0x1E01F268,0x6906C2FE,0xF762575D,0x806567CB,0x196C3671,0x6E6B06E7,
        0xFED41B76,0x89D32BE0,0x10DA7A5A,0x67DD4ACC,0xF9B9DF6F,0x8EBEEFF9,0x17B7BE43,0x60B08ED5,
        0xD6D6A3E8,0xA1D1937E,0x38D8C2C4,0x4FDFF252,0xD1BB67F1,0xA6BC5767,0x3FB506DD,0x48B2364B,
        0xD80D2BDA,0xAF0A1B4C,0x36034AF6,0x41047A60,0xDF60EFC3,0xA8670955,0x316658EF,0x46616879,
        0xB40BBE37,0xC30C8EA1,0x5A05DF1B,0x2D02EF8D
    };

    static uint32_t crc32Ethernet (const void* data, size_t length)
    {
        const uint8_t* p = static_cast<const uint8_t*> (data);
        uint32_t crc = 0xFFFFFFFFu;
        for (size_t i = 0; i < length; ++i)
            crc = kCrc32Table[(crc ^ p[i]) & 0xFF] ^ (crc >> 8);
        return crc ^ 0xFFFFFFFFu;
    }

} // anonymous namespace

// -----------------------------------------------------------------------------
// sendEsp32RecHello — negotiate rec-v1 on connect (Task 6)
// -----------------------------------------------------------------------------
bool AcqBoardRedPitaya::sendEsp32RecHello()
{
    if (commandSocket == nullptr)
        return false;

    // Reset any stale protocol state.
    esp32RecState.store (Esp32RecState::Idle);
    esp32SessionId.clear();
    esp32PendingSessionId.clear();
    esp32PendingFileSize     = 0;
    esp32PendingFileChecksum.clear();
    esp32RetrievalRetryAvailable = false;

    {
        const juce::ScopedLock sl (esp32RecStatusLock);
        esp32RecStatusText = "Negotiating SD recording protocol...";
    }

    const juce::ScopedLock cmdLock (esp32CommandLock);

    const char* hello = "REC HELLO protocol_min=rec-v1 client=plugin mode=direct_tcp\n";
    if (commandSocket->write (hello, (int) strlen (hello)) <= 0)
    {
        std::cout << "ESP32 rec-v1: failed to send REC HELLO" << std::endl;
        return false;
    }

    const String resp = readSocketLine (commandSocket, 2000);
    std::cout << "ESP32 rec-v1 HELLO response: [" << resp << "]" << std::endl;

    if (resp.startsWith ("REC HELLO_OK"))
    {
        // Parse max_chunk if present.
        const String maxChunkStr = parseRecField (resp, "max_chunk");
        if (maxChunkStr.isNotEmpty())
        {
            const int mc = maxChunkStr.getIntValue();
            if (mc > 0)
                esp32MaxChunkBytes = static_cast<uint32_t> (mc);
        }

        esp32RecV1Supported = true;
        {
            const juce::ScopedLock sl (esp32RecStatusLock);
            esp32RecStatusText = "SD recording ready (rec-v1)";
        }
        std::cout << "ESP32 rec-v1: supported. max_chunk=" << esp32MaxChunkBytes << std::endl;

        // Check for a timeout-finalized session retained from a previous connection (D-10).
        checkEsp32ReconnectSession();
        return true;
    }
    else if (resp.startsWith ("REC ERR"))
    {
        esp32RecV1Supported = false;
        esp32RecState.store (Esp32RecState::UnsupportedProtocol);
        {
            const juce::ScopedLock sl (esp32RecStatusLock);
            esp32RecStatusText = "Recording protocol unsupported — update firmware for SD recording";
        }
        std::cout << "ESP32 rec-v1: firmware returned REC ERR — old firmware." << std::endl;
        return false;
    }
    else
    {
        // No response or unexpected reply — assume old firmware.
        esp32RecV1Supported = false;
        {
            const juce::ScopedLock sl (esp32RecStatusLock);
            esp32RecStatusText = "SD recording protocol not detected (old firmware)";
        }
        std::cout << "ESP32 rec-v1: no REC HELLO_OK response — assuming old firmware." << std::endl;
        return false;
    }
}

// -----------------------------------------------------------------------------
// checkEsp32ReconnectSession — surface timeout-finalized session on reconnect (Task 7)
// -----------------------------------------------------------------------------
void AcqBoardRedPitaya::checkEsp32ReconnectSession()
{
    // commandSocket lock already held by caller (sendEsp32RecHello).
    if (commandSocket == nullptr)
        return;

    const char* query = "REC SESSION session_id=latest_finalized\n";
    if (commandSocket->write (query, (int) strlen (query)) <= 0)
        return;

    const String resp = readSocketLine (commandSocket, 2000);
    std::cout << "ESP32 rec-v1 reconnect session check: [" << resp << "]" << std::endl;

    if (! resp.startsWith ("REC SESSION_OK"))
        return; // no retained session or not finalized — that's fine

    const String fileSize = parseRecField (resp, "file_size");
    const String checksum = parseRecField (resp, "file_checksum");
    const String sid      = parseRecField (resp, "session_id");

    if (sid.isEmpty() || fileSize.isEmpty() || fileSize == "0")
        return; // incomplete metadata — ignore

    esp32PendingSessionId    = sid;
    esp32PendingFileSize     = static_cast<uint64_t> (fileSize.getLargeIntValue());
    esp32PendingFileChecksum = checksum;
    esp32RetrievalRetryAvailable = true;
    esp32RecState.store (Esp32RecState::SdFinalized);

    {
        const juce::ScopedLock sl (esp32RecStatusLock);
        esp32RecStatusText = "Timeout-finalized session available (session=" + sid + ") — press Retrieve";
    }
    std::cout << "ESP32 rec-v1: timeout-finalized session " << sid
              << " (" << fileSize << " bytes) available for retrieval." << std::endl;
}

// -----------------------------------------------------------------------------
// sendEsp32RecStart — replaces "RECORD ON" for rec-v1 firmware (Task 2)
// -----------------------------------------------------------------------------
bool AcqBoardRedPitaya::sendEsp32RecStart()
{
    if (! esp32RecV1Supported)
    {
        const juce::ScopedLock sl (esp32RecStatusLock);
        esp32RecStatusText = "Recording protocol unsupported — update firmware for SD recording";
        esp32RecState.store (Esp32RecState::UnsupportedProtocol);
        std::cout << "ESP32 record: rec-v1 not supported; cannot start SD recording." << std::endl;
        return false;
    }

    if (commandSocket == nullptr)
        return false;

    const juce::ScopedLock cmdLock (esp32CommandLock);

    const int hz = jmax (1, (int) settings.boardSampleRate);
    const int ch = numAdcChannels;

    char msg[128];
    snprintf (msg, sizeof (msg),
              "REC START sample_rate_hz=%d channels=%d format=sd-bin sd_required=true\n",
              hz, ch);

    if (commandSocket->write (msg, (int) strlen (msg)) <= 0)
    {
        const juce::ScopedLock sl (esp32RecStatusLock);
        esp32RecStatusText = "ESP32 SD recording: failed to send REC START";
        esp32RecState.store (Esp32RecState::Failed);
        return false;
    }

    esp32RecState.store (Esp32RecState::CommandSent);
    {
        const juce::ScopedLock sl (esp32RecStatusLock);
        esp32RecStatusText = "ESP32 SD recording: command sent — waiting for confirmation";
    }

    const String resp = readSocketLine (commandSocket, 1000);
    std::cout << "ESP32 REC START response: [" << resp << "]" << std::endl;

    if (resp.startsWith ("REC STARTED"))
    {
        esp32SessionId = parseRecField (resp, "session_id");
        if (esp32SessionId.isEmpty())
            esp32SessionId = "session_" + String (Time::currentTimeMillis());

        lastRecordingPath = "esp32://sd/" + esp32SessionId;
        lastRecordingCsvPath = {};

        esp32RecState.store (Esp32RecState::RecordingConfirmed);
        {
            const juce::ScopedLock sl (esp32RecStatusLock);
            esp32RecStatusText = "ESP32 SD recording confirmed (session=" + esp32SessionId + ")";
        }
        std::cout << "ESP32 rec-v1: recording started, session=" << esp32SessionId << std::endl;
        return true;
    }
    else if (resp.startsWith ("REC ERR"))
    {
        esp32RecState.store (Esp32RecState::Failed);
        {
            const juce::ScopedLock sl (esp32RecStatusLock);
            esp32RecStatusText = "ESP32 SD recording failed: " + resp;
        }
        std::cout << "ESP32 rec-v1: REC START returned error: " << resp << std::endl;
        return false;
    }
    else
    {
        // No response or unrecognised reply.
        esp32RecState.store (Esp32RecState::Failed);
        {
            const juce::ScopedLock sl (esp32RecStatusLock);
            esp32RecStatusText = "ESP32 SD recording: no confirmation from firmware";
        }
        std::cout << "ESP32 rec-v1: REC START — unexpected response: " << resp << std::endl;
        return false;
    }
}

// -----------------------------------------------------------------------------
// sendEsp32RecStop — initiate stop + async finalize/retrieve (Task 3)
// -----------------------------------------------------------------------------
bool AcqBoardRedPitaya::sendEsp32RecStop()
{
    if (! esp32RecV1Supported || esp32SessionId.isEmpty())
    {
        // Fallback: send legacy RECORD OFF for backward compat or unexpected state.
        if (commandSocket == nullptr)
            return false;

        const char* legacyMsg = "RECORD OFF\n";
        const int written = commandSocket->write (legacyMsg, (int) strlen (legacyMsg));
        std::cout << "ESP32 record: sent legacy RECORD OFF (no rec-v1 session active)." << std::endl;
        return written > 0;
    }

    if (commandSocket == nullptr)
        return false;

    {
        const juce::ScopedLock cmdLock (esp32CommandLock);

        char msg[128];
        snprintf (msg, sizeof (msg),
                  "REC STOP session_id=%s reason=manual_stop\n",
                  esp32SessionId.toRawUTF8());

        if (commandSocket->write (msg, (int) strlen (msg)) <= 0)
        {
            const juce::ScopedLock sl (esp32RecStatusLock);
            esp32RecStatusText = "ESP32 SD recording: failed to send REC STOP";
            esp32RecState.store (Esp32RecState::Failed);
            return false;
        }
    }

    esp32RecState.store (Esp32RecState::Finalizing);
    {
        const juce::ScopedLock sl (esp32RecStatusLock);
        esp32RecStatusText = "ESP32 SD recording: stop sent — finalizing and retrieving in background";
    }

    // Kick off async finalize/retrieve — does NOT block the audio thread.
    startEsp32RetrievalAsync();
    return true;
}

// -----------------------------------------------------------------------------
// getEsp32RecStatusText — thread-safe status text accessor
// -----------------------------------------------------------------------------
String AcqBoardRedPitaya::getEsp32RecStatusText() const
{
    const juce::ScopedLock sl (esp32RecStatusLock);
    return esp32RecStatusText;
}

// =============================================================================
// Esp32RetrievalThread — background finalize + chunk transfer (Task 4)
// =============================================================================
class Esp32RetrievalThread : public juce::Thread
{
public:
    Esp32RetrievalThread (AcqBoardRedPitaya& board)
        : juce::Thread ("ESP32RetrievalThread"),
          board_ (board)
    {
    }

    void run() override
    {
        using RecState = AcqBoardRedPitaya::Esp32RecState;

        auto setStatus = [this] (const String& text)
        {
            const juce::ScopedLock sl (board_.esp32RecStatusLock);
            board_.esp32RecStatusText = text;
            std::cout << "[ESP32 retrieval] " << text << std::endl;
        };

        auto setState = [this] (RecState s)
        {
            board_.esp32RecState.store (s);
        };

        // -----------------------------------------------------------------------
        // Step 1: Poll REC STATUS until recording_state=finalized (max 30 s)
        // -----------------------------------------------------------------------
        setStatus ("ESP32 SD recording: finalizing...");
        setState (RecState::Finalizing);

        bool finalized = false;
        const int64 finalizeDeadline = Time::currentTimeMillis() + 30000;

        while (! threadShouldExit() && ! board_.esp32RetrievalCancelRequested.load())
        {
            if (Time::currentTimeMillis() > finalizeDeadline)
            {
                setStatus ("ESP32 SD recording: finalization timeout after 30 s");
                setState (RecState::Failed);
                return;
            }

            // Poll STATUS
            String statusResp;
            {
                const juce::ScopedLock cmdLock (board_.esp32CommandLock);
                if (board_.commandSocket == nullptr)
                {
                    setStatus ("ESP32 SD recording: socket closed during finalization");
                    setState (RecState::Failed);
                    return;
                }

                const char* statusMsg = "REC STATUS\n";
                board_.commandSocket->write (statusMsg, (int) strlen (statusMsg));
                statusResp = readSocketLine (board_.commandSocket, 2000);
            }

            std::cout << "[ESP32 retrieval] STATUS: " << statusResp << std::endl;

            const String recState = parseRecField (statusResp, "recording_state");

            if (recState == "finalized")
            {
                finalized = true;
                break;
            }
            else if (recState == "failed" || recState == "error")
            {
                setStatus ("ESP32 SD recording: firmware reported finalization failure");
                setState (RecState::Failed);
                return;
            }

            // Still finalizing — wait 500 ms before next poll.
            Thread::sleep (500);
        }

        if (board_.esp32RetrievalCancelRequested.load())
        {
            sendAbort();
            setStatus ("ESP32 SD recording: retrieval cancelled");
            setState (RecState::Failed);
            return;
        }

        if (! finalized)
        {
            setState (RecState::Failed);
            return;
        }

        // -----------------------------------------------------------------------
        // Step 2: Get session metadata (file_size, file_checksum)
        // -----------------------------------------------------------------------
        String sessionId, fileChecksumHex;
        uint64_t fileSize = 0;

        {
            const juce::ScopedLock cmdLock (board_.esp32CommandLock);
            if (board_.commandSocket == nullptr)
            {
                setStatus ("ESP32 SD recording: socket closed before metadata fetch");
                setState (RecState::Failed);
                return;
            }

            const char* sessionMsg = "REC SESSION session_id=latest_finalized\n";
            board_.commandSocket->write (sessionMsg, (int) strlen (sessionMsg));
            const String resp = readSocketLine (board_.commandSocket, 2000);
            std::cout << "[ESP32 retrieval] SESSION: " << resp << std::endl;

            if (! resp.startsWith ("REC SESSION_OK"))
            {
                setStatus ("ESP32 SD recording: failed to get session metadata — " + resp);
                setState (RecState::Failed);
                return;
            }

            sessionId        = parseRecField (resp, "session_id");
            fileChecksumHex  = parseRecField (resp, "file_checksum");
            const String fs  = parseRecField (resp, "file_size");
            fileSize         = static_cast<uint64_t> (fs.getLargeIntValue());
        }

        if (sessionId.isEmpty())
            sessionId = board_.esp32SessionId;

        // Save pending metadata for retry.
        board_.esp32PendingSessionId    = sessionId;
        board_.esp32PendingFileSize     = fileSize;
        board_.esp32PendingFileChecksum = fileChecksumHex;
        setState (RecState::SdFinalized);
        setStatus ("ESP32 SD finalized (session=" + sessionId + ", " + String (fileSize) + " bytes) — retrieving...");

        // -----------------------------------------------------------------------
        // Step 3: Prepare local output directory
        // -----------------------------------------------------------------------
        const String timestamp = String (Time::currentTimeMillis());
        const String dirName   = sessionId + "_" + timestamp;
        const String resultDir = String (kEsp32RecordDir) + "\\" + dirName;

        File dir (resultDir);
        if (! dir.createDirectory())
        {
            setStatus ("ESP32 retrieval: could not create output directory: " + resultDir);
            setState (RecState::Failed);
            return;
        }

        board_.esp32LocalResultDir = resultDir;

        const String tmpPath  = resultDir + "\\session_data.bin.tmp";
        const String binPath  = resultDir + "\\session_data.bin";
        const String metaPath = resultDir + "\\metadata.json";
        const String logPath  = resultDir + "\\transfer_log.json";

        // -----------------------------------------------------------------------
        // Step 4: Write metadata.json
        // -----------------------------------------------------------------------
        {
            File meta (metaPath);
            juce::FileOutputStream metaOut (meta);
            if (metaOut.openedOk())
            {
                // Populate fields we have; firmware extras (write_errors etc.) come
                // from parseRecField on the SESSION_OK line if firmware provides them.
                metaOut.writeText (
                    "{\n"
                    "  \"session_id\": \"" + sessionId + "\",\n"
                    "  \"protocol\": \"rec-v1\",\n"
                    "  \"file_size\": " + String (fileSize) + ",\n"
                    "  \"file_checksum\": \"" + fileChecksumHex + "\",\n"
                    "  \"checksum_type\": \"crc32\",\n"
                    "  \"timestamp_ms\": " + timestamp + "\n"
                    "}\n",
                    false, false, "\n");
            }
        }

        // -----------------------------------------------------------------------
        // Step 5: Retrieve binary data in chunks via REC GET + SDRF frames
        // -----------------------------------------------------------------------
        setState (RecState::Retrieving);
        setStatus ("ESP32 SD retrieving: 0 / " + String (fileSize) + " bytes...");

        File tmpFile (tmpPath);
        juce::FileOutputStream tmpOut (tmpFile);
        if (! tmpOut.openedOk())
        {
            setStatus ("ESP32 retrieval: could not open temp file: " + tmpPath);
            setState (RecState::Failed);
            return;
        }

        // Build transfer_log JSON incrementally.
        File logFile (logPath);
        juce::FileOutputStream logOut (logFile);
        bool logOk = logOut.openedOk();
        if (logOk)
            logOut.writeText ("[\n", false, false, "\n");

        uint64_t offset          = 0;
        uint32_t chunkIndex      = 0;
        uint32_t localCrc        = 0xFFFFFFFFu; // running CRC32 of received payload bytes
        bool     transferOk      = true;
        bool     gotEof          = false;
        bool     firstLogEntry   = true;

        const uint32_t kSdrfMagic       = 0x46524453u; // 'SDRF' little-endian
        const uint8_t  kFrameTypeData   = 0x01;
        const uint8_t  kFrameTypeEof    = 0x02;
        const int      kSdrfHeaderBytes = 64;

        while (! threadShouldExit() && ! board_.esp32RetrievalCancelRequested.load())
        {
            if (fileSize > 0 && offset >= fileSize)
            {
                gotEof = true;
                break;
            }

            const uint32_t chunkLen = board_.esp32MaxChunkBytes;

            // Send REC GET
            char getMsg[128];
            snprintf (getMsg, sizeof (getMsg),
                      "REC GET session_id=%s offset=%llu length=%u chunk_index=%u\n",
                      sessionId.toRawUTF8(),
                      (unsigned long long) offset,
                      (unsigned) chunkLen,
                      (unsigned) chunkIndex);

            {
                const juce::ScopedLock cmdLock (board_.esp32CommandLock);
                if (board_.commandSocket == nullptr || board_.commandSocket->write (getMsg, (int) strlen (getMsg)) <= 0)
                {
                    setStatus ("ESP32 retrieval: failed to send REC GET at offset " + String (offset));
                    transferOk = false;
                    break;
                }
            }

            // Read 64-byte SDRF header.
            uint8_t header[64] = {};
            int headerBytesRead = 0;

            {
                const juce::ScopedLock cmdLock (board_.esp32CommandLock);
                if (board_.commandSocket == nullptr)
                {
                    transferOk = false;
                    break;
                }

                const int64 chunkDeadline = Time::currentTimeMillis() + 5000;
                while (headerBytesRead < kSdrfHeaderBytes && Time::currentTimeMillis() < chunkDeadline)
                {
                    if (! board_.commandSocket->waitUntilReady (true, 100))
                        continue;

                    const int n = board_.commandSocket->read (
                        header + headerBytesRead,
                        kSdrfHeaderBytes - headerBytesRead,
                        false);

                    if (n > 0)
                        headerBytesRead += n;
                    else if (n < 0)
                        break;
                }
            }

            if (headerBytesRead < kSdrfHeaderBytes)
            {
                setStatus ("ESP32 retrieval: incomplete SDRF header at chunk " + String (chunkIndex));
                transferOk = false;
                break;
            }

            // Validate SDRF magic ('S','D','R','F' = 0x53,0x44,0x52,0x46).
            uint32_t magic = 0;
            std::memcpy (&magic, header, 4);

            if (magic != kSdrfMagic)
            {
                // Check if firmware sent an error line instead of binary.
                String errLine (reinterpret_cast<const char*> (header), kSdrfHeaderBytes);
                setStatus ("ESP32 retrieval: bad SDRF magic at chunk " + String (chunkIndex)
                           + " (got: " + errLine.trim().substring (0, 40) + ")");
                transferOk = false;
                break;
            }

            // Parse header fields (little-endian, per sd_logger spec).
            uint8_t  frame_version    = header[4];
            uint8_t  frame_type       = header[5];
            // header[6..7] = reserved
            uint32_t payload_length   = 0;
            uint64_t byte_offset      = 0;
            uint32_t payload_crc32    = 0;
            uint32_t header_crc32c    = 0;

            std::memcpy (&payload_length, header +  8, 4);
            std::memcpy (&byte_offset,    header + 12, 8);
            std::memcpy (&payload_crc32,  header + 40, 4);
            std::memcpy (&header_crc32c,  header + 48, 4);

            // Validate header CRC32 (zeroing the crc field, compute over 64 bytes).
            uint8_t headerForCrc[64];
            std::memcpy (headerForCrc, header, 64);
            std::memset (headerForCrc + 48, 0, 4);
            const uint32_t computedHeaderCrc = crc32Ethernet (headerForCrc, 64);

            if (computedHeaderCrc != header_crc32c && header_crc32c != 0)
            {
                // Note: if firmware leaves header CRC as 0, we skip validation gracefully.
                setStatus ("ESP32 retrieval: SDRF header CRC mismatch at chunk " + String (chunkIndex));
                transferOk = false;
                break;
            }

            if (frame_type == kFrameTypeEof)
            {
                gotEof = true;
                break;
            }

            if (frame_type != kFrameTypeData)
            {
                setStatus ("ESP32 retrieval: unexpected SDRF frame_type 0x"
                           + String::toHexString ((int) frame_type));
                transferOk = false;
                break;
            }

            if (byte_offset != offset)
            {
                setStatus ("ESP32 retrieval: SDRF byte_offset mismatch at chunk "
                           + String (chunkIndex)
                           + " (expected " + String (offset)
                           + ", got " + String (byte_offset) + ")");
                transferOk = false;
                break;
            }

            // Read payload.
            juce::MemoryBlock payload (payload_length);
            int payloadRead = 0;

            {
                const juce::ScopedLock cmdLock (board_.esp32CommandLock);
                if (board_.commandSocket == nullptr)
                {
                    transferOk = false;
                    break;
                }

                const int64 payDeadline = Time::currentTimeMillis() + 10000;
                while (payloadRead < (int) payload_length && Time::currentTimeMillis() < payDeadline)
                {
                    if (! board_.commandSocket->waitUntilReady (true, 100))
                        continue;

                    const int n = board_.commandSocket->read (
                        static_cast<uint8_t*> (payload.getData()) + payloadRead,
                        (int) payload_length - payloadRead,
                        false);

                    if (n > 0)
                        payloadRead += n;
                    else if (n < 0)
                        break;
                }
            }

            if (payloadRead < (int) payload_length)
            {
                setStatus ("ESP32 retrieval: incomplete payload at chunk " + String (chunkIndex)
                           + " (" + String (payloadRead) + " of " + String (payload_length) + " bytes)");
                transferOk = false;
                break;
            }

            // Validate payload CRC.
            const uint32_t computedPayloadCrc = crc32Ethernet (payload.getData(), payload_length);
            if (computedPayloadCrc != payload_crc32 && payload_crc32 != 0)
            {
                setStatus ("ESP32 retrieval: payload CRC mismatch at chunk " + String (chunkIndex));
                transferOk = false;
                break;
            }

            // Update running whole-file CRC.
            const uint8_t* payBytes = static_cast<const uint8_t*> (payload.getData());
            for (int i = 0; i < (int) payload_length; ++i)
                localCrc = kCrc32Table[(localCrc ^ payBytes[i]) & 0xFF] ^ (localCrc >> 8);

            // Write to temp file.
            tmpOut.write (payload.getData(), payload_length);

            // Append chunk entry to transfer_log.
            if (logOk)
            {
                if (! firstLogEntry)
                    logOut.writeText (",\n", false, false, "\n");
                firstLogEntry = false;

                logOut.writeText (
                    "  {\"chunk_index\":" + String (chunkIndex)
                    + ",\"byte_offset\":" + String (offset)
                    + ",\"payload_length\":" + String (payload_length)
                    + ",\"payload_crc32\":\"" + String::toHexString ((int) computedPayloadCrc) + "\""
                    + "}",
                    false, false, "\n");
            }

            offset += payload_length;
            ++chunkIndex;

            setStatus ("ESP32 SD retrieving: " + String (offset) + " / " + String (fileSize) + " bytes...");
        }

        tmpOut.flush();
        tmpOut.~FileOutputStream(); // close before rename

        if (board_.esp32RetrievalCancelRequested.load())
        {
            sendAbort();
            File (tmpPath).deleteFile();
            setStatus ("ESP32 SD recording: retrieval cancelled");
            setState (RecState::Failed);
            if (logOk) { logOut.writeText ("\n]\n", false, false, "\n"); }
            return;
        }

        if (! transferOk)
        {
            board_.esp32RetrievalRetryAvailable = true;
            setState (RecState::Failed);
            if (logOk) { logOut.writeText ("\n]\n", false, false, "\n"); }
            return;
        }

        // -----------------------------------------------------------------------
        // Step 6: Whole-file CRC32 verification
        // -----------------------------------------------------------------------
        setState (RecState::LocalCopyWritten);
        setStatus ("ESP32 SD: verifying whole-file CRC32...");

        // Finalise running CRC.
        const uint32_t computedFileCrc = localCrc ^ 0xFFFFFFFFu;
        const uint32_t expectedFileCrc = static_cast<uint32_t> (
            fileChecksumHex.getHexValue32());

        const bool crcMatch = (computedFileCrc == expectedFileCrc)
                              || fileChecksumHex.isEmpty(); // no checksum provided by firmware

        if (logOk)
        {
            logOut.writeText (
                "\n],\n"
                "\"whole_file_crc_match\":" + String (crcMatch ? "true" : "false") + ",\n"
                "\"computed_crc32\":\"" + String::toHexString ((int) computedFileCrc) + "\",\n"
                "\"expected_crc32\":\"" + fileChecksumHex + "\"\n"
                "}\n",
                false, false, "\n");
        }

        if (! crcMatch)
        {
            setStatus ("ESP32 SD: transfer CRC mismatch! computed="
                       + String::toHexString ((int) computedFileCrc)
                       + " expected=" + fileChecksumHex
                       + " — press Retry Retrieval");
            board_.esp32RetrievalRetryAvailable = true;
            File (tmpPath).deleteFile();
            setState (RecState::Failed);
            return;
        }

        // Atomically rename .tmp → .bin
        File tmpF (tmpPath);
        File binF (binPath);
        if (! tmpF.moveFileTo (binF))
        {
            setStatus ("ESP32 SD: could not rename temp file to session_data.bin");
            setState (RecState::Failed);
            return;
        }

        setState (RecState::ChecksumPassed);
        setStatus ("ESP32 SD: transfer checksum passed — running analyzer...");

        // Send REC COMPLETE acknowledgement.
        {
            const juce::ScopedLock cmdLock (board_.esp32CommandLock);
            if (board_.commandSocket != nullptr)
            {
                char completeMsg[256];
                snprintf (completeMsg, sizeof (completeMsg),
                          "REC COMPLETE session_id=%s file_size=%llu file_checksum=%s checksum_type=crc32 local_transfer_id=%s\n",
                          sessionId.toRawUTF8(),
                          (unsigned long long) fileSize,
                          fileChecksumHex.toRawUTF8(),
                          String (Time::currentTimeMillis()).toRawUTF8());
                board_.commandSocket->write (completeMsg, (int) strlen (completeMsg));
            }
        }

        // -----------------------------------------------------------------------
        // Step 7: Write analyzer_handoff.json and attempt to run analyzer
        // -----------------------------------------------------------------------
        const String handoffPath  = resultDir + "\\analyzer_handoff.json";
        const String analyzerPath = resultDir + "\\analyzer_result.json";
        const String analyzerCmd  = "python esp32/host/analyze_sample_rate.py --format sd-bin session_data.bin";

        {
            File handoffFile (handoffPath);
            juce::FileOutputStream handoffOut (handoffFile);
            if (handoffOut.openedOk())
            {
                handoffOut.writeText (
                    "{\n"
                    "  \"command\": \"" + analyzerCmd + "\",\n"
                    "  \"session_id\": \"" + sessionId + "\",\n"
                    "  \"required\": true\n"
                    "}\n",
                    false, false, "\n");
            }
        }

        // Try to run the analyzer.
        bool analyzerPassed = false;
        String analyzerResult;

        // Build analyzer script path relative to the result directory.
        // Script is at: <resultDir>/../../host/analyze_sample_rate.py
        //             = C:\Users\justi\Documents\Arduino\ESP32-S3-1\results\..\..\host\...
        // Simplify: always use the path relative to kEsp32RecordDir.
        const String scriptPath = String (kEsp32RecordDir) + "\\..\\host\\analyze_sample_rate.py";
        const String analyzerFullCmd = "python \"" + scriptPath + "\" --format sd-bin \""
                                       + binPath + "\"";

        juce::ChildProcess analyzerProc;
        if (analyzerProc.start (analyzerFullCmd))
        {
            analyzerProc.waitForProcessToFinish (30000);
            analyzerResult = analyzerProc.readAllProcessOutput();
            analyzerPassed = (analyzerProc.getExitCode() == 0);
        }
        else
        {
            analyzerResult = "Analyzer not found or failed to start. Run manually: " + analyzerFullCmd;
        }

        // Write analyzer_result.json.
        {
            File resultFile (analyzerPath);
            juce::FileOutputStream resultOut (resultFile);
            if (resultOut.openedOk())
            {
                const String escaped = analyzerResult.replace ("\"", "\\\"")
                                                     .replace ("\n", "\\n");
                resultOut.writeText (
                    "{\n"
                    "  \"passed\": " + String (analyzerPassed ? "true" : "false") + ",\n"
                    "  \"output\": \"" + escaped + "\"\n"
                    "}\n",
                    false, false, "\n");
            }
        }

        if (analyzerPassed)
        {
            setState (RecState::AnalyzerPassed);
            setStatus ("Recording saved and verified (session=" + sessionId + ")");
        }
        else
        {
            setState (RecState::ChecksumPassed);
            setStatus ("Transfer checksum passed. Analyzer not run / failed — check analyzer_result.json. Session: " + sessionId);
        }

        std::cout << "[ESP32 retrieval] Complete. Session: " << sessionId
                  << " | Dir: " << resultDir << std::endl;
    }

private:
    AcqBoardRedPitaya& board_;

    void sendAbort()
    {
        const juce::ScopedLock cmdLock (board_.esp32CommandLock);
        if (board_.commandSocket != nullptr && ! board_.esp32PendingSessionId.isEmpty())
        {
            char msg[128];
            snprintf (msg, sizeof (msg),
                      "REC ABORT session_id=%s reason=user_cancel\n",
                      board_.esp32PendingSessionId.toRawUTF8());
            board_.commandSocket->write (msg, (int) strlen (msg));
        }
    }

    static String readSocketLine (StreamingSocket* sock, int timeoutMs)
    {
        if (sock == nullptr) return {};
        String line;
        const int64 deadline = Time::currentTimeMillis() + timeoutMs;
        while (Time::currentTimeMillis() < deadline)
        {
            if (! sock->waitUntilReady (true, 50)) continue;
            char c = 0;
            if (sock->read (&c, 1, false) <= 0) break;
            if (c == '\n') break;
            if (c >= 0x20 || c == '\t') line += c;
        }
        return line;
    }

    static String parseRecField (const String& line, const String& key)
    {
        const String needle = key + "=";
        const int idx = line.indexOf (needle);
        if (idx < 0) return {};
        String val = line.substring (idx + needle.length());
        const int sp = val.indexOfAnyOf (" \t\r\n");
        if (sp >= 0) val = val.substring (0, sp);
        return val;
    }
};

// -----------------------------------------------------------------------------
// startEsp32RetrievalAsync — spin up the background retrieval thread (Task 4)
// -----------------------------------------------------------------------------
void AcqBoardRedPitaya::startEsp32RetrievalAsync()
{
    // Stop any existing retrieval thread first.
    if (esp32RetrievalThread != nullptr)
    {
        esp32RetrievalThread->signalThreadShouldExit();
        esp32RetrievalThread->waitForThreadToExit (2000);
        esp32RetrievalThread.reset();
    }

    esp32RetrievalCancelRequested.store (false);
    esp32RetrievalThread = std::make_unique<Esp32RetrievalThread> (*this);
    esp32RetrievalThread->startThread (juce::Thread::Priority::normal);
}

// -----------------------------------------------------------------------------
// retryEsp32Retrieval — operator retry after failed transfer (Task 10)
// -----------------------------------------------------------------------------
bool AcqBoardRedPitaya::retryEsp32Retrieval()
{
    if (! esp32RetrievalRetryAvailable || esp32PendingSessionId.isEmpty())
        return false;

    // Restore session id from pending so the retrieval thread can use it.
    esp32SessionId = esp32PendingSessionId;
    esp32RetrievalRetryAvailable = false;
    esp32RetrievalCancelRequested.store (false);

    {
        const juce::ScopedLock sl (esp32RecStatusLock);
        esp32RecStatusText = "Retrying retrieval for session " + esp32SessionId + "...";
    }
    esp32RecState.store (Esp32RecState::SdFinalized);

    startEsp32RetrievalAsync();
    return true;
}

// -----------------------------------------------------------------------------
// cancelEsp32Retrieval
// -----------------------------------------------------------------------------
void AcqBoardRedPitaya::cancelEsp32Retrieval()
{
    esp32RetrievalCancelRequested.store (true);

    if (esp32RetrievalThread != nullptr)
        esp32RetrievalThread->waitForThreadToExit (5000);
}

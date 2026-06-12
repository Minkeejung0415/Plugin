/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI

    ------------------------------------------------------------------
*/

#ifndef __ACQBOARDREDPITAYA_H_2C4CBD67__
#define __ACQBOARDREDPITAYA_H_2C4CBD67__

#include "../AcquisitionBoard.h"
#include "../../opensim_joint_catalog.h"

#include <array>

/*
    ============================================================
    AcqBoardRedPitaya — the fully filled-in board for Red Pitaya / ESP32-S3
    ============================================================

    This is the concrete (fully implemented) version of AcquisitionBoard
    that knows how to talk to a Red Pitaya Linux computer or an ESP32-S3
    STEP wireless node over TCP.

    Connection flow:
      detectBoard()     — opens a TCP socket on port 5000 and sends "REDPITAYA\n".
                          The board replies with "OK CHANNELS:N\n". We parse N.
      initializeBoard() — sets sample rate and clears any stale state.
      startAcquisition()— sends "FREQ:<hz>\n" then "START\n". Board replies
                          "STARTED BIN:... CSV:...\n" then streams binary frames.
      run()             — the background thread that continuously reads frames,
                          unpacks int16 payload, scales to floats, and writes to
                          the shared DataBuffer.
      stopAcquisition() — sends "STOP\n". Board flushes its recording and replies
                          "STOPPED\n".

    ASCII commands sent to board during acquisition:
      FILTER ON/OFF        — enables/disables VQF sensor fusion on the firmware side
      RECORD ON/OFF        — starts/stops local SD-card or ESP32 CSV recording
      FREQ:<hz>            — changes hardware tick rate on the fly
      CFG <si> ACC <0-3>   — sets accelerometer full-scale range for sensor index si
      CFG <si> GYR <0-3>   — sets gyroscope full-scale range for sensor index si
      CFG <si> SRATE <hz>  — sets per-sensor effective sample rate (firmware decimates)
      AIN_GAIN:<f>         — sets analog input gain
      AOUT:<v>             — sets analog output DAC voltage

    The board also sends back "SENSORS:0,MPU9250;1,ICM20948\n" immediately after
    STARTED so we know which physical sensors are active.
*/

class AcqBoardRedPitaya : public AcquisitionBoard
{
public:
    /** Constructor */
    AcqBoardRedPitaya();

    /** Destructor */
    virtual ~AcqBoardRedPitaya();

    /** Detects whether a board is present */
    bool detectBoard() override;

    /** Initializes board after successful detection */
    bool initializeBoard() override;

    /** Returns true if the device is connected */
    bool foundInputSource() const override;

    /** Returns an array of connected headstages for this board (none for Red Pitaya) */
    Array<const Headstage*> getHeadstages() override;

    /** Returns available sample rates */
    Array<int> getAvailableSampleRates() override;

    /** Set sample rate (in Hz) */
    void setSampleRate (int sampleRateHz) override;

    /** Gets the current sample rate */
    float getSampleRate() const override;

    /** Checks for connected headstages (no-op for Red Pitaya) */
    void scanPorts() override;

    /** Enables AUX channel out (not used) */
    void enableAuxChannels (bool enabled) override;

    /** Checks whether AUX channels are enabled */
    bool areAuxChannelsEnabled() const override;

    /** Enables ADC channel out */
    void enableAdcChannels (bool enabled) override;

    /** Checks whether ADC channels are enabled */
    bool areAdcChannelsEnabled() const override;

    /** Returns bitVolts scaling value for each channel type */
    float getBitVolts (ContinuousChannel::Type) const override;

    /** Measures impedance of each channel (not supported) */
    void measureImpedances() override;

    /** Called when impedance measurement is complete */
    void impedanceMeasurementFinished() override;

    /** Save impedance measurements to XML (not supported) */
    void saveImpedances (File& file) override;

    /** Sets the method for determining channel names (no-op for now) */
    void setNamingScheme (ChannelNamingScheme scheme) override;

    /** Gets the method for determining channel names */
    ChannelNamingScheme getNamingScheme() override;

    bool isReady() override;

    /** Initializes data transfer */
    bool startAcquisition() override;

    /** Stops data transfer */
    bool stopAcquisition() override;

    /** Sets analog filter upper limit; returns actual value */
    double setUpperBandwidth (double upperBandwidth) override;

    /** Sets analog filter lower limit; returns actual value */
    double setLowerBandwidth (double lowerBandwidth) override;

    /** Sets DSP cutoff frequency; returns actual value */
    double setDspCutoffFreq (double freq) override;

    /** Gets the current DSP cutoff frequency */
    double getDspCutoffFreq() const override;

    /** Sets whether DSP offset is enabled */
    void setDspOffset (bool enabled) override;

    /** Sets whether TTL output mode is enabled */
    void setTTLOutputMode (bool enabled) override;

    /** Sets whether DAC highpass filter is enabled, and set the cutoff freq */
    void setDAChpf (float cutoff, bool enabled) override;

    /** Sets whether fast TTL settle is enabled, and set the trigger channel */
    void setFastTTLSettle (bool state, int channel) override;

    /** Sets level of noise slicer on DAC channels */
    int setNoiseSlicerLevel (int level) override;

    /** Turns LEDs on or off */
    void enableBoardLeds (bool enabled) override;

    /** Sets divider on clock output */
    int setClockDivider (int divide_ratio) override;

    /** Connects a headstage channel to a DAC (not used) */
    void connectHeadstageChannelToDAC (int headstageChannelIndex, int dacChannelIndex) override;

    /** Sets trigger threshold for DAC channel (if TTL output mode is enabled) */
    void setDACTriggerThreshold (int dacChannelIndex, float threshold) override;

    /** Returns true if a headstage is enabled (always false; no headstages) */
    bool isHeadstageEnabled (int hsNum) const override;

    /** Returns the active number of channels in a headstage (always 0) */
    int getActiveChannelsInHeadstage (int hsNum) const override;

    /** Returns the total number of channels in a headstage (always 0) */
    int getChannelsInHeadstage (int hsNum) const override;

    /** Returns total number of outputs per channel type */
    int getNumDataOutputs (ContinuousChannel::Type) override;

    /** Sets the number of channels to use in a headstage (no-op) */
    void setNumHeadstageChannels (int headstageIndex, int channelCount) override;

    bool sendRecordOnCommand() override;

    bool sendRecordOffCommand() override;

    String getLastRecordingPath() const override { return lastRecordingPath; }

    String getLastRecordingCsvPath() const override { return lastRecordingCsvPath; }

    void updateSampleFrequency (int newFreq) override;

    /** Enables or disables hardware filter */
    void setFilterEnabled (bool enabled) override;

    /** Sets analog input gain factor */
    void setAnalogInGain (float gain) override;

    /** Sets analog output voltage */
    void setAnalogOutVoltage (float voltage) override;

    /** Sensors active at last successful stream start (from SENSORS: line). */
    int getStreamSensorCount() const { return streamSensorNames.size(); }

    String getStreamSensorName (int index) const;

    /** Send CFG lines (during acquisition when socket is open). */
    bool sendSensorCfgAcc (int sensorIndex, int presetId);
    bool sendSensorCfgGyr (int sensorIndex, int presetId);
    bool sendSensorCfgSrate (int sensorIndex, int targetHz);

    void launchOpenSimMotion();
    void launchOpenSimLive();

    static String getOpenSimWorkDir();

    int getJointCatalogSize() const { return kOpenSimJointCatalogSize; }

    const OpenSimJointCatalogEntry& getJointCatalogEntry (int index) const;

    bool isJointDisplaySelected (int catalogIndex) const;

    void setJointDisplaySelected (int catalogIndex, bool selected);

    StringArray getSelectedDisplayJoints() const;

    void loadJointDisplayFromXml (const XmlElement& parent);

    void saveJointDisplayToXml (XmlElement& parent) const;

    /*
        writeJointDisplayConfig() — atomically writes opensim_joint_display_config.json.

        Python (opensim_live_realtime.py) polls this file every ~50 ms. Whenever the
        operator toggles a joint checkbox OR a trigger fires (via handleBroadcastMessage),
        we write a fresh JSON snapshot of which joints the HUD should show.

        Writing atomically (write to tmp file, then rename) prevents Python from
        reading a half-written file mid-write.
    */
    bool writeJointDisplayConfig();

    /** Fills data buffer — runs on the background acquisition thread */
    void run();

    /*
        MAX_SAMPLES_PER_BUFFER — how many samples we collect before calling
        buffer->addToBuffer(). Larger = less overhead per call; smaller = lower
        latency. 128 samples at 100 Hz = 1.28 seconds of buffering capacity.
    */
    static constexpr int MAX_SAMPLES_PER_BUFFER = 128;

    /*
        MAX_CHANNELS — maximum number of channels we ever expect from a Red Pitaya.
        Sized generously to include IMU raw channels + 4 VQF quaternion channels per
        sensor + 2 analog waveform channels. Actual channel count depends on
        how many sensors the board reports.
    */
    static constexpr int MAX_CHANNELS = 64;

    /*
        ANALOG_WAVEFORM_CHANNELS — the Red Pitaya's two oscilloscope-style analog
        inputs (CH1 and CH2) are always appended at the end of each frame.
        These measure real voltages (e.g. from an EMG amplifier) rather than IMU data.
    */
    static constexpr int ANALOG_WAVEFORM_CHANNELS = 2;

    /*
        Ring buffers the run() thread fills each iteration.
        samples[]       — interleaved float channel data for all channels × MAX_SAMPLES_PER_BUFFER
        sampleNumbers[] — monotonically increasing sample index (used as the "timestamp" Open Ephys stores)
        timestamps[]    — wall-clock time in seconds (unused for Red Pitaya; Open Ephys derives from sampleNumbers)
        event_codes[]   — packed TTL event bits per sample (8 bits = 8 digital input lines)
    */
    float samples[MAX_CHANNELS * MAX_SAMPLES_PER_BUFFER];
    int64 sampleNumbers[MAX_SAMPLES_PER_BUFFER];
    double timestamps[MAX_SAMPLES_PER_BUFFER];
    uint64 event_codes[MAX_SAMPLES_PER_BUFFER];

    /** Number of ADC channels we expose through this board */
    int numAdcChannels = 2;

    /** Whether we report that ADC channels are enabled */
    bool acquireAdc = true;

    /** Whether AUX channels are enabled (unused) */
    bool acquireAux = false;

    /** True if logical device is considered present */
    bool deviceFound = false;

    /** Latest DSP cutoff frequency */
    double dspCutoffFreqHz = 0.5;

    /** Analog filter band limits */
    double lowerBandwidthHz = 1.0;
    double upperBandwidthHz = 7500.0;

    /** Whether TTL output mode is enabled */
    bool ttlOutputMode = false;

    /** Desired clock divide ratio */
    int clockDivideRatio = 1;

    /** BitVolts scaling for ADC channels */
    float adcBitVolts = 1.0f;

    bool filterEnabled = false;
    float analogInGain = 1.0f;
    float analogOutVoltage = 0.0f;

    /*
        commandSocket — the persistent TCP connection to the Red Pitaya / ESP32.
        We keep this open throughout the session so we can send ASCII commands
        at any time (not just during streaming). nullptr when disconnected.
    */
    StreamingSocket* commandSocket = nullptr;

    /*
        activeRedPitayaHost — the hostname or IP that successfully answered our
        REDPITAYA handshake. E.g. "rp-f0f85a.local" or "192.168.4.1".
        Stored so we can reconnect after a disconnect without re-scanning.
    */
    String activeRedPitayaHost;

    /** Optional ESP32-S3 node host (editor or ESP32_NODE_HOST env). Tried after rp-*.local. */
    String configurableNodeHost;

    /*
        isEsp32Node — true when the device that answered the handshake is an
        ESP32-S3 STEP node running the STEP firmware, not a Red Pitaya.
        The ESP32 has a different default channel layout (11 channels) and
        stores recordings on the host PC rather than on-device SD card.
    */
    bool isEsp32Node = false;

    /** Set/clear user-configured node IP or hostname (e.g. 192.168.4.1). */
    void setNodeHost (const String& host) { configurableNodeHost = host.trim(); }

    String getNodeHost() const { return configurableNodeHost; }

    bool getIsEsp32Node() const { return isEsp32Node; }

    /*
        ESP32_DEFAULT_CHANNELS = 11 — the ESP32-S3 STEP node always sends exactly
        11 channels per frame in this order:
          0=ax (accel X, g)   1=ay   2=az
          3=gx (gyro X, dps)  4=gy   5=gz
          6=dio (digital I/O bitmask)
          7=qw  8=qx  9=qy  10=qz  (VQF quaternion, scaled Q15)
    */
    static constexpr int ESP32_DEFAULT_CHANNELS = 11;

    void resetCommandSocket();
    bool connectCommandSocketToHost (const String& host);
    bool connectCommandSocketToBoard();
    bool performDetectionHandshake();

    /** Re-run detection (e.g. after user sets Node IP in editor). */
    bool retryDetection();

    String lastRecordingPath;
    String lastRecordingCsvPath;

    /** Populated after STARTED + SENSORS: snapshot from board. */
    Array<String> streamSensorNames;

    /*
        sensorAccPreset[] / sensorGyrPreset[] — remember the current full-scale range
        setting for each sensor so run() can scale raw int16 counts to physical units.
        Index matches streamSensorNames (sensor 0, 1, 2, ...).
        ACC presets: 0=±2g, 1=±4g, 2=±8g, 3=±16g
        GYR presets: 0=±250°/s, 1=±500, 2=±1000, 3=±2000
    */
    int sensorAccPreset[6] = {};   /* 0=±2g, 1=±4g, 2=±8g,  3=±16g       */
    int sensorGyrPreset[6] = {};   /* 0=±250°/s, 1=±500, 2=±1000, 3=±2000 */

    /** Per-sensor OpenSim body segment assignment — index into getBodySegmentName().
     *  Written to opensim_sensor_map.json when changed or just before launching the bridge. */
    int sensorBodySegment[6] = {};  /* default 0 = tibia_r_imu for every sensor */

    int displayJointIndex = 1;  /* default: knee_angle_r */

    mutable CriticalSection liveAngleLock;
    float liveDisplayAngleDeg = 0.0f;
    bool liveAnglesValid = false;

    static constexpr int NUM_BODY_SEGMENTS = 13;
    static constexpr int NUM_DISPLAY_JOINTS = 10;
    static const char* getBodySegmentName (int idx);   // e.g. "tibia_r_imu"
    static const char* getBodySegmentLabel (int idx);  // e.g. "Right Tibia"

    void setSensorBodySegment (int sensorIndex, int segmentIndex);
    int  getSensorBodySegment (int sensorIndex) const;

    /** Writes opensim_sensor_map.json so the Python bridge picks up the current mapping. */
    bool writeOpenSimSensorMap() const;

    /** Writes opensim_display_joint.json for the live Simbody angle HUD. */
    bool writeOpenSimDisplayJoint() const;

    void setDisplayJointIndex (int index);
    int  getDisplayJointIndex() const { return displayJointIndex; }

    static const char* getDisplayJointName (int idx);
    static const char* getDisplayJointLabel (int idx);

    /** Latest displayed-joint angle from opensim_live_realtime.py (UDP v3 feedback). */
    bool getLiveDisplayAngle (float& angleDeg) const;
    bool hasLiveJointAngles() const { return liveAnglesValid; }

    /** Non-blocking read of angle feedback on UDP port 5001. */
    void pollOpenSimAngleFeedback();

    /*
        sendOpenSimQuaternionPacket() — sends a UDP v2 packet to opensim_live_realtime.py.

        Packet format (all floats, little-endian):
          [timestamp, 2.0 (version tag), numSensors,
           qw0, qx0, qy0, qz0,   <- sensor 0 quaternion (unit length)
           qw1, qx1, qy1, qz1,   <- sensor 1 quaternion
           ...]
        Python reassembles the quaternions into body-segment orientations and feeds
        them to the OpenSim InverseKinematicsSolver every frame.
    */
    void sendOpenSimQuaternionPacket (float timestamp, const float* quats, int numSensors);

    /** UDP v3: imu6 is numSensors×6 floats (ax,ay,az,gx,gy,gz in g and deg/s per sensor). */
    void sendOpenSimImuPacket (float timestamp, const float* imu6, int numSensors);

    /*
        openSimSocket — UDP socket used to send quaternion packets to Python on port 5000.
        Created in startAcquisition() when OpenSim Live is enabled, closed in stopAcquisition().
    */
    std::unique_ptr<DatagramSocket> openSimSocket;

    /** UDP socket for receiving angle feedback from Python (port 5001). */
    std::unique_ptr<DatagramSocket> openSimAngleSocket;

    /*
        openSimEnabled — true once the operator clicks "OpenSim Live" and Python starts.
        When true, run() calls sendOpenSimQuaternionPacket() every frame.
    */
    bool openSimEnabled { false };

    /*
        openSimProcess — ChildProcess handle for the offline IK batch ("Gen Motion") job.
        openSimLiveProcess — ChildProcess handle for the live Python visualizer.
        Both are started with juce::ChildProcess::start() and run independently.
    */
    std::unique_ptr<juce::ChildProcess> openSimProcess;
    std::unique_ptr<juce::ChildProcess> openSimLiveProcess;

    // PC-side CSV recording for ESP32 WiFi mode (written by run(), opened/closed by UI thread).
    std::unique_ptr<juce::FileOutputStream> esp32RecordStream;
    juce::CriticalSection esp32RecordLock;
    int esp32RecordSampleCount = 0;

    /*
        jointDisplaySelected[] — one boolean per entry in kOpenSimJointCatalog (7 joints).
        true = the operator wants this joint shown on the OpenSim HUD.
        Persisted to XML on session save/load. Reflected in the joint toggle checkboxes.
        Maximum 6 can be true at once (enforced in setJointDisplaySelected()).
    */
    std::array<bool, kOpenSimJointCatalogSize> jointDisplaySelected {};

    /*
        jointDisplayConfigSeq — incrementing sequence number written into the JSON file.
        Python can detect that the file was updated by checking if the number changed,
        avoiding a full file parse every poll cycle if it has not changed.
    */
    int jointDisplayConfigSeq = 0;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (AcqBoardRedPitaya);
};

#endif

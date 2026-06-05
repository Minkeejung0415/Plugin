/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI

    ------------------------------------------------------------------
*/

#ifndef __ACQBOARDREDPITAYA_H_2C4CBD67__
#define __ACQBOARDREDPITAYA_H_2C4CBD67__

#include "../AcquisitionBoard.h"

/**
    Interface for a network-based Red Pitaya device acting as an
    Open Ephys Acquisition Board compatible source.

    This implementation currently provides a basic skeleton:
    - It exposes only ADC channels (no headstages).
    - Streaming logic is implemented in run(), which can be adapted
      to receive samples over UDP or another transport.

    @see DataThread, SourceNode
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

    /** Fills data buffer */
    void run();

    /** Maximum samples per buffer for Red Pitaya ADC */
    static constexpr int MAX_SAMPLES_PER_BUFFER = 128;

    /** Maximum Red Pitaya channels, including reserved filtered/fusion outputs */
    static constexpr int MAX_CHANNELS = 64;

    /** Red Pitaya analog voltage waveform channels appended to the stream */
    static constexpr int ANALOG_WAVEFORM_CHANNELS = 2;

    /** Data buffers */
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

    StreamingSocket* commandSocket = nullptr;

    /** Host that answered the last successful REDPITAYA handshake (e.g. rp-f0f85a.local). */
    String activeRedPitayaHost;

    /** Optional ESP32-S3 node host (editor or ESP32_NODE_HOST env). Tried after rp-*.local. */
    String configurableNodeHost;

    /** True when handshake matched ESP32 STEP firmware (TCP stream, 11 ch default @ 100 Hz). */
    bool isEsp32Node = false;

    void setNodeHost (const String& host) { configurableNodeHost = host.trim(); }

    String getNodeHost() const { return configurableNodeHost; }

    bool getIsEsp32Node() const { return isEsp32Node; }

    static constexpr int ESP32_DEFAULT_CHANNELS = 11;

    void resetCommandSocket();
    bool connectCommandSocketToHost (const String& host);
    bool connectCommandSocketToBoard();
    bool performDetectionHandshake();

    bool retryDetection();

    String lastRecordingPath;
    String lastRecordingCsvPath;

    /** Populated after STARTED + SENSORS: snapshot from board. */
    Array<String> streamSensorNames;

    /** Per-sensor range presets, updated when CFG commands are sent.
     *  Index matches streamSensorNames. Used by run() to scale raw counts. */
    int sensorAccPreset[6] = {};   /* 0=±2g, 1=±4g, 2=±8g,  3=±16g       */
    int sensorGyrPreset[6] = {};   /* 0=±250°/s, 1=±500, 2=±1000, 3=±2000 */

    /** Per-sensor OpenSim body segment assignment — index into getBodySegmentName().
     *  Written to opensim_sensor_map.json when changed or just before launching the bridge. */
    int sensorBodySegment[6] = {};  /* default 0 = tibia_r_imu for every sensor */

    static constexpr int NUM_BODY_SEGMENTS = 13;
    static const char* getBodySegmentName (int idx);   // e.g. "tibia_r_imu"
    static const char* getBodySegmentLabel (int idx);  // e.g. "Right Tibia"

    void setSensorBodySegment (int sensorIndex, int segmentIndex);
    int  getSensorBodySegment (int sensorIndex) const;

    /** Writes opensim_sensor_map.json so the Python bridge picks up the current mapping. */
    bool writeOpenSimSensorMap() const;

    /** UDP v2: quats is numSensors×4 floats (qw,qx,qy,qz per sensor, unit quaternion). */
    void sendOpenSimQuaternionPacket (float timestamp, const float* quats, int numSensors);

    std::unique_ptr<DatagramSocket> openSimSocket;
    bool openSimEnabled { false };
    std::unique_ptr<juce::ChildProcess> openSimProcess;
    std::unique_ptr<juce::ChildProcess> openSimLiveProcess;

    // PC-side CSV recording for ESP32 WiFi mode (written by run(), opened/closed by UI thread).
    std::unique_ptr<juce::FileOutputStream> esp32RecordStream;
    juce::CriticalSection esp32RecordLock;
    int esp32RecordSampleCount = 0;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (AcqBoardRedPitaya);
};

#endif

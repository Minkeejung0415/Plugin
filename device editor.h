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

#ifndef __DEVICEEDITOR_H_2AD3C591__
#define __DEVICEEDITOR_H_2AD3C591__

#include "UI/MemoryMonitorUsage.h"
#include "opensim_joint_catalog.h"
#include <VisualizerEditorHeaders.h>

#include <array>

class HeadstageOptionsInterface;
class SampleRateInterface;
class BandwidthInterface;
class DSPInterface;
class AudioInterface;
class ClockDivideInterface;
class DeviceThread;
class ChannelCanvas;

struct ImpedanceData;

/*
    ============================================================
    DeviceEditor — the control panel the user sees in Open Ephys
    ============================================================

    This is the JUCE GUI panel attached to the acquisition board plugin.
    When a Red Pitaya (or ESP32-S3 STEP node) is connected, the panel
    expands to 800 px wide and shows 6 vertical columns of controls:

      Col 1 (x=6):   Sample rate (editable Hz label), OpenSim Live button,
                      RECORD button
      Col 2 (x=135): Hardware FILTER toggle, Analog In gain, Analog Out voltage
      Col 3 (x=275): Sensor config — Accel range, Gyro range, Sensor Hz (decimation)
      Col 4 (x=420): Sensor select (dropdown populated after acquisition starts),
                      Body segment assignment, Gen Motion button
      Col 5 (x=555): Joint HUD checkboxes — 7 joints from kOpenSimJointCatalog,
                      laid out in a 2-column grid (max 6 can be active at once)
      Col 6 (x=690): Node IP label (editable), Display Joint dropdown

    For non-Red-Pitaya boards (OpalKelly, ONI, Simulated) the panel is 340 px
    wide and shows only the standard headstage/bandwidth/DSP controls.

    DeviceEditor listens to button clicks (buttonClicked), combo box changes
    (comboBoxChanged), and editable label changes (labelTextChanged) — all via
    JUCE listener callbacks.
*/

class DeviceEditor : public VisualizerEditor,
                     public ComboBox::Listener,
                     public Button::Listener,
                     public PopupChannelSelector::Listener,
                     public Label::Listener

{
public:
    /** Constructor */
    DeviceEditor (GenericProcessor* parentNode, class AcquisitionBoard* board);

    /** Destructor*/
    ~DeviceEditor() {}

    /** Respond to combo box changes (e.g. sample rate)*/
    void comboBoxChanged (ComboBox* comboBox) override;

    /** Respond to button clicks*/
    void buttonClicked (Button* button) override;

    /** Disable UI during acquisition*/
    void startAcquisition() override;

    /** Enable UI after acquisition is finished*/
    void stopAcquisition() override;

    /** Runs impedance test*/
    void measureImpedances();

    /** Callback when impedance measurement is finished */
    void impedanceMeasurementFinished();

    /** Saves impedance data to a file */
    void saveImpedances (File& file);

    /** Updates channel canvas*/
    void updateSettings() override;

    /** Saves custom parameters */
    void saveVisualizerEditorParameters (XmlElement* xml) override;

    /** Loads custom parameters*/
    void loadVisualizerEditorParameters (XmlElement* xml) override;

    /** Creates an interface with additional channel settings*/
    Visualizer* createNewCanvas (void) override;

    /** Called by PopupChannelSelector */
    void channelStateChanged (Array<int> newChannels) override;

    /** Called by PopupChannelSelector */
    int getChannelCount() override;

    virtual Array<int> getSelectedChannels() override { return Array<int>(); }

    void setPercentMemoryUsed (float memoryUsed)
    {
        if (memoryUsage != nullptr)
            memoryUsage->setPercentMemoryUsed (memoryUsed);
    }

    void labelTextChanged (Label* labelThatHasChanged) override;

    void paint (Graphics& g) override;

    /** Refreshes sensor-select and sensor-rate combos after a stream starts and
        the board reports which sensors are active (SENSORS: line from firmware). */
    void refreshRedPitayaSensorCombosFromBoard();

    /** Turns joint toggle labels orange when an active sensor covers that joint's
        body segment — gives the operator a visual hint about which joints are "live". */
    void refreshJointDisplayHighlights();

    /** Reads the board's jointDisplaySelected[] array and applies it to the 7 toggle
        checkboxes in the UI, so saved state is reflected when loading a session. */
    void syncJointDisplayTogglesFromBoard();

    /** Rebuilds the Sensor Hz combo with 7 decimation choices relative to hwHz.
        Called when the hardware rate label changes or a new stream starts. */
    void repopulateSensorRateComboForHwHz (int hwHz);

    int getSelectedStreamSensorIndex() const;

    /** Keeps AcquisitionBoard::settings.boardSampleRate in sync with the Red Pitaya
        hardware Hz label so Open Ephys stream metadata and buffer sizing match the TCP stream. */
    void syncRedPitayaBoardSampleRateFromLabel();

    /** Updates the editable HW rate label from boardSampleRate (e.g. after ESP32 detect). */
    void syncRedPitayaSampleRateLabelFromBoard();

    /** Disables the board-side RECORD button for ESP32 nodes (no device storage) and
        re-enables it for Red Pitaya. Call after any detection that sets isEsp32Node. */
    void syncRecordButtonForBoardType();

    /** Pushes the display-joint combo selection into the board before OpenSim launch. */
    void syncOpenSimDisplayJointToBoard();

private:
    /** Pointer to acquisition board device */
    class AcquisitionBoard* board;

    /** Pointer to visualizer canvas */
    ChannelCanvas* canvas;

    /** XmlElement to hold previously saved parameters if no device is found */
    std::unique_ptr<XmlElement> previousSettings;

    std::unique_ptr<MemoryMonitorUsage> memoryUsage = nullptr;

    // --- Headstage controls (for OpalKelly/ONI boards with Intan headstages) ---
    OwnedArray<HeadstageOptionsInterface> headstageOptionsInterfaces;
    OwnedArray<ElectrodeButton> electrodeButtons;

    // --- Legacy bandwidth / DSP / sample-rate interfaces (not shown for Red Pitaya) ---
    std::unique_ptr<SampleRateInterface> sampleRateInterface;
    std::unique_ptr<BandwidthInterface> bandwidthInterface;
    std::unique_ptr<DSPInterface> dspInterface;

    // --- Legacy audio and clock-divide interfaces (not shown for Red Pitaya) ---
    std::unique_ptr<AudioInterface> audioInterface;
    std::unique_ptr<ClockDivideInterface> clockInterface;

    // --- Col 1 controls: record, sample rate, OpenSim Live ---
    std::unique_ptr<UtilityButton> recordButton;   // sends RECORD ON / RECORD OFF to firmware
    std::unique_ptr<UtilityButton> rescanButton, dacTTLButton;
    std::unique_ptr<UtilityButton> auxButton;
    std::unique_ptr<UtilityButton> adcButton;
    std::unique_ptr<UtilityButton> ledButton;

    std::unique_ptr<UtilityButton> dspoffsetButton;
    std::unique_ptr<ComboBox> ttlSettleCombo, dacHPFcombo;
    std::unique_ptr<Label> audioLabel, ttlSettleLabel, dacHPFlabel;
    std::unique_ptr<Label> noBoardsDetectedLabel;  // shown when board == nullptr

    // --- Col 1: sample rate label (user edits this to change Hz; sends FREQ: command) ---
    std::unique_ptr<Label> sampleRateTitle;
    std::unique_ptr<Label> sampleRateLabel;  // editable; changing it sends FREQ: to firmware

    // --- Col 6: node IP / hostname (user types ESP32 IP here to re-detect) ---
    std::unique_ptr<Label> nodeHostTitle;
    std::unique_ptr<Label> nodeHostLabel;   // user types in the ESP32 IP here; triggers re-detection when changed

    // --- Col 2: filter and analog controls ---
    std::unique_ptr<UtilityButton> filterButton;  // sends FILTER ON / FILTER OFF to firmware
    std::unique_ptr<Label> filterTitle;

    std::unique_ptr<Label> analogInTitle;
    std::unique_ptr<Label> analogInLabel;   // editable ADC gain (0.1–100); sends AIN_GAIN: command

    std::unique_ptr<Label> analogOutTitle;
    std::unique_ptr<Label> analogOutLabel;  // editable DAC voltage (0–1.8 V); sends AOUT: command

    // --- Col 3 & 4: Red Pitaya sensor configuration (only built when isRedPitaya) ---
    bool redPitayaSensorUiBuilt = false;  // guards against rebuilding the sensor UI twice

    // Col 3: Accel range dropdown (±2g / ±4g / ±8g / ±16g) — sends CFG <si> ACC <preset>
    std::unique_ptr<Label> sensorCfgAccelTitle;
    std::unique_ptr<ComboBox> sensorCfgAccelCombo;

    // Col 3: Gyro range dropdown (±250 / ±500 / ±1000 / ±2000 °/s) — sends CFG <si> GYR <preset>
    std::unique_ptr<Label> sensorCfgGyroTitle;
    std::unique_ptr<ComboBox> sensorCfgGyroCombo;

    // Col 3: Per-sensor effective sample rate (decimated from hw Hz) — sends CFG <si> SRATE <hz>
    std::unique_ptr<Label> sensorCfgRateTitle;
    std::unique_ptr<ComboBox> sensorCfgRateCombo;

    // Col 4: Which sensor is currently being configured (populated after acquisition starts)
    std::unique_ptr<Label> sensorSelectTitle;
    std::unique_ptr<ComboBox> sensorSelectCombo;

    // Col 4: OpenSim body segment this sensor is mounted on (written to opensim_sensor_map.json)
    std::unique_ptr<Label>    sensorBodySegmentTitle;
    std::unique_ptr<ComboBox> sensorBodySegmentCombo;

    // Col 4: Buttons to launch OpenSim workflows
    std::unique_ptr<UtilityButton> openSimMotionButton;  // runs offline IK batch job
    std::unique_ptr<UtilityButton> openSimLiveButton;    // starts live Python visualizer

    // --- Col 5: Joint HUD toggles ---
    std::unique_ptr<Label> jointDisplayTitle;

    /*
        jointDisplayToggles — 7 checkboxes, one per entry in kOpenSimJointCatalog:
          pelvis_tilt, hip_flexion_r, hip_flexion_l,
          knee_angle_r, knee_angle_l, ankle_angle_r, ankle_angle_l.
        The operator checks the joints they want to see on the OpenSim viewer.
        Maximum 6 can be active at once (enforced in buttonClicked()).
        When toggled, the board's jointDisplaySelected[] is updated and
        writeJointDisplayConfig() is called to write the JSON sidecar file.
    */
    std::array<std::unique_ptr<ToggleButton>, kOpenSimJointCatalogSize> jointDisplayToggles;

    // --- Col 6: Display joint dropdown (single joint shown by Python legacy path) ---
    std::unique_ptr<Label>    displayJointTitle;
    std::unique_ptr<ComboBox> displayJointCombo;

    /*
        AudioChannel — which DAC output the audio monitor is routed to.
        LEFT and RIGHT correspond to the two DAC channels (indices 0 and 1).
        activeAudioChannel tracks which one a button press will configure.
    */
    enum AudioChannel
    {
        LEFT = 0,
        RIGHT = 1
    };

    AudioChannel activeAudioChannel;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (DeviceEditor);
};

/*
    HeadstageOptionsInterface — shows the per-port headstage buttons.
    For a 32-channel headstage it lets the user toggle between 16-ch and 32-ch mode.
    One of these is created per physical port (up to 4 ports).
*/
class HeadstageOptionsInterface : public Component,
                                  public Button::Listener
{
public:
    /** Constructor*/
    HeadstageOptionsInterface (class AcquisitionBoard*, DeviceEditor*, int hsNum);

    /** Destructor */
    ~HeadstageOptionsInterface();

    /** Draw the options interface background */
    void paint (Graphics& g) override;

    /** Toggle between 16 and 32 ch */
    void buttonClicked (Button* button) override;

    /** Refresh button state*/
    void checkEnabledState();

    /** Set enabled (e.g. during acquisition) */
    void setEnabled (bool state);

    /** Checks whether headstage is in 32- or 16-channel mode*/
    bool is32Channel (int hsIndex);

    /** Sets HS in 32- or 16-ch mode */
    void set32Channel (int hsIndex, bool is32Channel);

private:
    int hsNumber1, hsNumber2;
    int channelsOnHs1, channelsOnHs2;
    String name;

    bool isEnabled;

    class AcquisitionBoard* board;
    DeviceEditor* editor;

    std::unique_ptr<UtilityButton> hsButton1;
    std::unique_ptr<UtilityButton> hsButton2;
};

/*
    BandwidthInterface — two editable labels for the RHD chip's analog bandpass filter.
    Upper cutoff (default 7500 Hz) and lower cutoff (default 1 Hz) set the recording
    bandwidth for all electrode channels. Calls board->setUpperBandwidth / setLowerBandwidth.
*/
class BandwidthInterface : public Component,
                           public Label::Listener
{
public:
    /** Constructor */
    BandwidthInterface (class AcquisitionBoard*, DeviceEditor*);

    /** Destructor */
    ~BandwidthInterface();

    /** Draw interface labels */
    void paint (Graphics& g) override;

    /** Called when settings are changed */
    void labelTextChanged (Label* te) override;

    /** Sets lower bandwidth value */
    void setLowerBandwidth (double value);

    /** Sets upper bandwidth value */
    void setUpperBandwidth (double value);

    /** Returns actual lower bandwidth value */
    double getLowerBandwidth();

    /** Returns actual upper bandwidth value */
    double getUpperBandwidth();

private:
    String name;

    String lastLowCutString, lastHighCutString;

    class AcquisitionBoard* board;
    DeviceEditor* editor;

    std::unique_ptr<Label> upperBandwidthSelection;
    std::unique_ptr<Label> lowerBandwidthSelection;

    double actualUpperBandwidth;
    double actualLowerBandwidth;
};

/*
    DSPInterface — controls the Intan chip's on-chip digital high-pass filter.
    The DSP filter removes slow electrode drift (DC offset) without touching the
    analog hardware. cutoffFreq defaults to 0.5 Hz. Calls board->setDspCutoffFreq.
*/
class DSPInterface : public Component,
                     public Label::Listener
{
public:
    /** Constructor */
    DSPInterface (class AcquisitionBoard*, DeviceEditor*);

    /** Destructor */
    ~DSPInterface();

    /** Draw interface labels */
    void paint (Graphics& g) override;

    /** Called when settings are changed */
    void labelTextChanged (Label* te) override;

    /** Sets DSP cutoff frequency */
    void setDspCutoffFreq (double value);

    /** Returns actual DSP cutoff frequency */
    double getDspCutoffFreq();

private:
    String name;

    class AcquisitionBoard* board;
    DeviceEditor* editor;

    std::unique_ptr<Label> dspOffsetSelection;

    double actualDspCutoffFreq = 0.5;
};

/*
    SampleRateInterface — dropdown for choosing the global sample rate.
    Used by OpalKelly and ONI boards (which have fixed rate choices like
    1, 1.25, 1.5 kHz … 30 kHz). Red Pitaya uses the free-form sampleRateLabel
    instead, so this interface is not shown for it.
*/
class SampleRateInterface : public Component,
                            public ComboBox::Listener
{
public:
    /** Constructor */
    SampleRateInterface (class AcquisitionBoard*, DeviceEditor*);

    /** Destructor */
    ~SampleRateInterface();

    /** Returns index of selected sample rate */
    int getSelectedId();

    /** Sets sample rate by index */
    void setSelectedId (int);

    /** Returns sample rate string */
    String getText();

    /** Draw interface labels */
    void paint (Graphics& g) override;

    /** Called when settings are changed */
    void comboBoxChanged (ComboBox* cb) override;

private:
    int sampleRate;
    String name;

    class AcquisitionBoard* board;
    DeviceEditor* editor;

    std::unique_ptr<ComboBox> rateSelection;
    StringArray sampleRateOptions;
};

/*
    AudioInterface — controls the board's audio monitor output.
    The noise slicer threshold gate is set here; signals below the threshold
    are silenced so the operator hears only genuine spikes.
*/
class AudioInterface : public Component,
                       public Label::Listener
{
public:
    /** Constructor */
    AudioInterface (class AcquisitionBoard*, DeviceEditor*);

    /** Destructor */
    ~AudioInterface();

    /** Draw interface labels */
    void paint (Graphics& g) override;

    /** Called when settings are changed */
    void labelTextChanged (Label* te) override;

    /** Sets noise slicer level (used to reduce background noise) */
    void setNoiseSlicerLevel (int value);

    /** Returns actual noise slicer level */
    int getNoiseSlicerLevel();

private:
    String name;

    String lastNoiseSlicerString;
    String lastGainString;

    class AcquisitionBoard* board;
    DeviceEditor* editor;

    std::unique_ptr<Label> noiseSlicerLevelSelection;

    int actualNoiseSlicerLevel;
};

/*
    ClockDivideInterface — sets the divide ratio for the clock output on the sync BNC.
    Example: if sample rate = 30 kHz and ratio = 10, the BNC outputs a 3 kHz square wave.
    This is useful for synchronising external equipment with the recording.
*/
class ClockDivideInterface : public Component,
                             public Label::Listener
{
public:
    /** Constructor */
    ClockDivideInterface (class AcquisitionBoard*, DeviceEditor*);

    /** Draws the interface labels */
    void paint (Graphics& g) override;

    /** Called when settings are changed */
    void labelTextChanged (Label* te) override;

    /** Sets clock divide ratio */
    void setClockDivideRatio (int value);

    /** Returns actual clock divide ratio */
    int getClockDivideRatio() const { return actualDivideRatio; };

private:
    String name;
    String lastDivideRatioString;

    class AcquisitionBoard* board;
    DeviceEditor* editor;

    std::unique_ptr<Label> divideRatioSelection;
    int actualDivideRatio;
};

#endif // __DEVICEEDITOR_H_2AD3C591__

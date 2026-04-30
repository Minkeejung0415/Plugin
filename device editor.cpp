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

#include "DeviceEditor.h"

#include "devices/AcquisitionBoard.h"
#include "devices/redpitaya/AcqBoardRedPitaya.h"
#include "devices/oni/AcqBoardONI.h"

#include "UI/ChannelCanvas.h"

#include <cmath>

#ifdef WIN32
#if (_MSC_VER < 1800) //round doesn't exist on MSVC prior to 2013 version
inline double round (double x)
{
    return floor (x + 0.5);
}
#endif
#endif

DeviceEditor::DeviceEditor (GenericProcessor* parentNode,
                            AcquisitionBoard* board_)
    : VisualizerEditor (parentNode,
                        "Acq Board",
                        (board_ != nullptr && board_->getBoardType() == AcquisitionBoard::BoardType::RedPitaya) ? 560 : 340),
      board (board_),
      activeAudioChannel (LEFT)
{
    canvas = nullptr;
    noBoardsDetectedLabel = nullptr;

    if (board == nullptr)
    {
        noBoardsDetectedLabel = std::make_unique<Label> ("NoBoardsDetected", "No Boards Detected.");
        noBoardsDetectedLabel->setBounds (0, 15, 340, 125);
        noBoardsDetectedLabel->setAlwaysOnTop (true);
        noBoardsDetectedLabel->toFront (false);
        noBoardsDetectedLabel->setJustificationType (Justification::centred);
        noBoardsDetectedLabel->setColour (Label::textColourId, Colours::black);
        addAndMakeVisible (noBoardsDetectedLabel.get());

        return;
    }

    board->editor = this;

    int xOffset = 0;

    if (board->getBoardType() == AcquisitionBoard::BoardType::ONI)
    {
        if (((AcqBoardONI*) board)->getMemoryMonitorSupport())
        {
            desiredWidth += 22;

            memoryUsage = std::make_unique<MemoryMonitorUsage> (parentNode);
            memoryUsage->setBounds (8, 30, 15, 95);
            memoryUsage->setTooltip ("Monitors the percent of the hardware memory buffer used.");
            addAndMakeVisible (memoryUsage.get());

            xOffset = memoryUsage->getRight();
        }
    }

    const bool isRedPitaya = (board->getBoardType() == AcquisitionBoard::BoardType::RedPitaya);
    const int leftCol  = xOffset + 6;
    const int col2     = xOffset + 145;
    const int midCol   = isRedPitaya ? (xOffset + 285) : (xOffset + 155);
    const int rightCol = isRedPitaya ? (xOffset + 420) : (xOffset + 155);

    // add headstage-specific controls (currently just a toggle button)
    /*
    for (int i = 0; i < 4; i++)
    {
        HeadstageOptionsInterface* hsOptions = new HeadstageOptionsInterface (board, this, i);
        headstageOptionsInterfaces.add (hsOptions);
        addAndMakeVisible (hsOptions);
        hsOptions->setBounds (xOffset + 3, 28 + i * 20, 70, 18);
    }
    */
    //add record button
    recordButton = std::make_unique<UtilityButton> ("RECORD");
    recordButton->setRadius (3.0f);
    recordButton->setBounds (isRedPitaya ? leftCol : (xOffset + 6), isRedPitaya ? 100 : 108, isRedPitaya ? 120 : 65, 18);
    recordButton->addListener (this);
    recordButton->setClickingTogglesState (true);
    recordButton->setTooltip ("Record streaming data");
    addAndMakeVisible (recordButton.get());

    sampleRateTitle = std::make_unique<Label> ("sampleRateTitle", "Sample Rate (Hz)");
    sampleRateTitle->setFont (FontOptions ("Inter", "Regular", 10.0f));
    sampleRateTitle->setBounds (isRedPitaya ? leftCol : (xOffset + 80), isRedPitaya ? 18 : 22,
                                isRedPitaya ? 90 : 100, isRedPitaya ? 12 : 15);
    addAndMakeVisible (sampleRateTitle.get());

    sampleRateLabel = std::make_unique<Label> ("sampleRateLabel", "1000");
    sampleRateLabel->setEditable (true);
    sampleRateLabel->setColour (Label::backgroundColourId, Colours::black);
    sampleRateLabel->setColour (Label::textColourId, Colours::white);
    sampleRateLabel->setBounds (isRedPitaya ? leftCol : (xOffset + 80), isRedPitaya ? 30 : 38, 60, 18);
    sampleRateLabel->addListener (this);
    sampleRateLabel->setTooltip (isRedPitaya ? "Hardware tick rate 1–2000 Hz (per-sensor SRATE decimates from this)"
                                             : "Set streaming frequency (100 - 2000 Hz)");
    addAndMakeVisible (sampleRateLabel.get());

    filterTitle = std::make_unique<Label> ("filterTitle", "Filter");
    filterTitle->setFont (FontOptions ("Inter", "Regular", 10.0f));
    filterTitle->setBounds (isRedPitaya ? col2 : (xOffset + 155), isRedPitaya ? 18 : 22, 65, 12);
    addAndMakeVisible (filterTitle.get());

    filterButton = std::make_unique<UtilityButton> ("FILTER");
    filterButton->setRadius (3.0f);
    filterButton->setBounds (isRedPitaya ? col2 : (xOffset + 155), isRedPitaya ? 30 : 38, isRedPitaya ? 120 : 65, 18);
    filterButton->addListener (this);
    filterButton->setClickingTogglesState (true);
    filterButton->setToggleState (false, dontSendNotification);
    filterButton->setEnabledState (true);
    filterButton->setTooltip ("Toggle hardware filter on/off");
    addAndMakeVisible (filterButton.get());

    analogInTitle = std::make_unique<Label> ("analogInTitle", isRedPitaya ? "Ain" : "Analog In Gain");
    analogInTitle->setFont (FontOptions ("Inter", "Regular", 10.0f));
    analogInTitle->setBounds (isRedPitaya ? col2 : (xOffset + 155), isRedPitaya ? 58 : 62, 80, 12);
    addAndMakeVisible (analogInTitle.get());

    analogInLabel = std::make_unique<Label> ("analogInLabel", "1.00");
    analogInLabel->setEditable (true);
    analogInLabel->setEnabled (true);
    analogInLabel->setColour (Label::backgroundColourId, Colours::black);
    analogInLabel->setColour (Label::textColourId, Colours::white);
    analogInLabel->setBounds (isRedPitaya ? col2 : (xOffset + 155), isRedPitaya ? 72 : 74, 52, 18);
    analogInLabel->addListener (this);
    analogInLabel->setTooltip ("Set ADC input gain (0.1 - 100.0)");
    addAndMakeVisible (analogInLabel.get());

    analogOutTitle = std::make_unique<Label> ("analogOutTitle", isRedPitaya ? "Aout" : "Analog Out (V)");
    analogOutTitle->setFont (FontOptions ("Inter", "Regular", 10.0f));
    analogOutTitle->setBounds (isRedPitaya ? col2 : (xOffset + 155), isRedPitaya ? 94 : 94, 80, 12);
    addAndMakeVisible (analogOutTitle.get());

    analogOutLabel = std::make_unique<Label> ("analogOutLabel", "0.00");
    analogOutLabel->setEditable (true);
    analogOutLabel->setEnabled (true);
    analogOutLabel->setColour (Label::backgroundColourId, Colours::black);
    analogOutLabel->setColour (Label::textColourId, Colours::white);
    analogOutLabel->setBounds (isRedPitaya ? col2 : (xOffset + 155), isRedPitaya ? 108 : 108, 52, 18);
    analogOutLabel->addListener (this);
    analogOutLabel->setTooltip ("Set DAC output voltage (0.0 - 1.8 V)");
    addAndMakeVisible (analogOutLabel.get());

    if (isRedPitaya)
    {
        sensorCfgAccelTitle = std::make_unique<Label> ("sensorCfgAccelTitle", "Accel");
        sensorCfgAccelTitle->setFont (FontOptions ("Inter", "Regular", 9.0f));
        sensorCfgAccelTitle->setBounds (midCol, 18, 100, 12);
        addAndMakeVisible (sensorCfgAccelTitle.get());

        sensorCfgAccelCombo = std::make_unique<ComboBox> ("sensorCfgAccel");
        sensorCfgAccelCombo->setBounds (midCol, 30, 120, 20);
        sensorCfgAccelCombo->addListener (this);
        sensorCfgAccelCombo->addItem ("±2 g",  1);
        sensorCfgAccelCombo->addItem ("±4 g",  2);
        sensorCfgAccelCombo->addItem ("±8 g",  3);
        sensorCfgAccelCombo->addItem ("±16 g", 4);
        sensorCfgAccelCombo->setSelectedId (1, dontSendNotification);
        addAndMakeVisible (sensorCfgAccelCombo.get());

        sensorCfgGyroTitle = std::make_unique<Label> ("sensorCfgGyroTitle", "Gyro");
        sensorCfgGyroTitle->setFont (FontOptions ("Inter", "Regular", 9.0f));
        sensorCfgGyroTitle->setBounds (midCol, 54, 100, 12);
        addAndMakeVisible (sensorCfgGyroTitle.get());

        sensorCfgGyroCombo = std::make_unique<ComboBox> ("sensorCfgGyro");
        sensorCfgGyroCombo->setBounds (midCol, 66, 120, 20);
        sensorCfgGyroCombo->addListener (this);
        sensorCfgGyroCombo->addItem ("±250 °/s",  1);
        sensorCfgGyroCombo->addItem ("±500 °/s",  2);
        sensorCfgGyroCombo->addItem ("±1000 °/s", 3);
        sensorCfgGyroCombo->addItem ("±2000 °/s", 4);
        sensorCfgGyroCombo->setSelectedId (1, dontSendNotification);
        addAndMakeVisible (sensorCfgGyroCombo.get());

        sensorCfgRateTitle = std::make_unique<Label> ("sensorCfgRateTitle", "Sensor Hz");
        sensorCfgRateTitle->setFont (FontOptions ("Inter", "Regular", 9.0f));
        sensorCfgRateTitle->setBounds (midCol, 90, 100, 12);
        addAndMakeVisible (sensorCfgRateTitle.get());

        sensorCfgRateCombo = std::make_unique<ComboBox> ("sensorCfgRate");
        sensorCfgRateCombo->setBounds (midCol, 102, 120, 20);
        sensorCfgRateCombo->addListener (this);
        addAndMakeVisible (sensorCfgRateCombo.get());

        sensorSelectTitle = std::make_unique<Label> ("sensorSelectTitle", "Sensor");
        sensorSelectTitle->setFont (FontOptions ("Inter", "Regular", 9.0f));
        sensorSelectTitle->setBounds (rightCol, 18, 110, 12);
        addAndMakeVisible (sensorSelectTitle.get());

        sensorSelectCombo = std::make_unique<ComboBox> ("sensorSelect");
        sensorSelectCombo->setBounds (rightCol, 30, 120, 20);
        sensorSelectCombo->addListener (this);
        sensorSelectCombo->addItem ("(start acquisition)", 1);
        sensorSelectCombo->setSelectedId (1, dontSendNotification);
        addAndMakeVisible (sensorSelectCombo.get());

        redPitayaSensorUiBuilt = true;
    }

    // add rescan button
    /*
    rescanButton = std::make_unique<UtilityButton> ("RESCAN");
    rescanButton->setRadius (3.0f);
    rescanButton->setBounds (xOffset + 6, 108, 65, 18);
    rescanButton->addListener (this);
    rescanButton->setTooltip ("Check for connected headstages");
    addAndMakeVisible (rescanButton.get());
    */

    // add sample rate selection
    /*
    sampleRateInterface = std::make_unique<SampleRateInterface> (board, this);
    addAndMakeVisible (sampleRateInterface.get());
    sampleRateInterface->setBounds (xOffset + 80, 22, 80, 50);
    */

    // add Bandwidth selection
    /*
    bandwidthInterface = std::make_unique<BandwidthInterface> (board, this);
    addAndMakeVisible (bandwidthInterface.get());
    bandwidthInterface->setBounds (xOffset + 80, 60, 80, 45);
    */

    // add AUX channel enable/disable button
    /*
    auxButton = std::make_unique<UtilityButton> ("AUX");
    auxButton->setRadius (3.0f);
    auxButton->setBounds (xOffset + 80, 108, 32, 18);
    auxButton->addListener (this);
    auxButton->setClickingTogglesState (true);
    auxButton->setTooltip ("Toggle AUX channels (3 per headstage)");
    addAndMakeVisible (auxButton.get());
    */

    // add ADC channel enable/disable button
    /*
    adcButton = std::make_unique<UtilityButton> ("ADC");
    adcButton->setRadius (3.0f);
    adcButton->setBounds (xOffset + 80 + 32 + 1, 108, 32, 18);
    adcButton->addListener (this);
    adcButton->setClickingTogglesState (true);
    adcButton->setTooltip ("Toggle 8 external HDMI ADC channels");
    addAndMakeVisible (adcButton.get());
    */

    // add audio output config interface
    /*
    audioLabel = std::make_unique<Label> ("audio label", "Audio out");
    audioLabel->setBounds (xOffset + 170, 22, 75, 15);
    audioLabel->setFont (FontOptions ("Inter", "Regular", 10.0f));
    addAndMakeVisible (audioLabel.get());
    */

    /*
    for (int i = 0; i < 2; i++)
    {
        ElectrodeButton* button = new ElectrodeButton (-1);
        electrodeButtons.add (button);

        button->setBounds (xOffset + 174 + i * 30, 35, 30, 15);
        button->setChannelNum (-1);
        button->setClickingTogglesState (false);
        button->setToggleState (false, dontSendNotification);

        addAndMakeVisible (button);
        button->addListener (this);

        if (i == 0)
        {
            button->setTooltip ("Audio monitor left channel");
        }
        else
        {
            button->setTooltip ("Audio monitor right channel");
        }
    }
    */

    // add HW audio parameter selection
    /*
    audioInterface = std::make_unique<AudioInterface> (board, this);
    addAndMakeVisible (audioInterface.get());
    audioInterface->setBounds (xOffset + 174, 55, 70, 50);

    clockInterface = std::make_unique<ClockDivideInterface> (board, this);
    addAndMakeVisible (clockInterface.get());
    clockInterface->setBounds (xOffset + 174, 80, 70, 50);
    */

    // add DSP Offset Button
    /*
    dspoffsetButton = std::make_unique<UtilityButton> ("DSP:");
    dspoffsetButton->setRadius (3.0f); // sets the radius of the button's corners
    dspoffsetButton->setBounds (xOffset + 174, 108, 32, 18); // sets the x position, y position, width, and height of the button
    dspoffsetButton->addListener (this);
    dspoffsetButton->setClickingTogglesState (true); // makes the button toggle its state when clicked
    dspoffsetButton->setTooltip ("Toggle DSP offset removal");
    addAndMakeVisible (dspoffsetButton.get()); // makes the button a child component of the editor and makes it visible
    dspoffsetButton->setToggleState (true, dontSendNotification);
    */

    // add DSP Frequency Selection field
    /*
    dspInterface = std::make_unique<DSPInterface> (board, this);
    addAndMakeVisible (dspInterface.get());
    dspInterface->setBounds (xOffset + 174 + 32, 108, 40, 50);
    */

    /*
    dacTTLButton = std::make_unique<UtilityButton> ("DAC TTL");
    dacTTLButton->setRadius (3.0f);
    dacTTLButton->setBounds (xOffset + 260, 25, 60, 18);
    dacTTLButton->addListener (this);
    dacTTLButton->setClickingTogglesState (true);
    dacTTLButton->setTooltip ("Toggle DAC Threshold TTL Output");
    addAndMakeVisible (dacTTLButton.get());

    dacHPFlabel = std::make_unique<Label> ("DAC HPF", "DAC HPF");
    dacHPFlabel->setFont (FontOptions ("Inter", "Regular", 10.0f));
    dacHPFlabel->setBounds (xOffset + 255, 40, 60, 20);
    addAndMakeVisible (dacHPFlabel.get());

    dacHPFcombo = std::make_unique<ComboBox> ("dacHPFCombo");
    dacHPFcombo->setBounds (xOffset + 260, 55, 60, 18);
    dacHPFcombo->addListener (this);
    dacHPFcombo->addItem ("OFF", 1);
    int HPFvalues[10] = { 50, 100, 200, 300, 400, 500, 600, 700, 800, 900 };
    for (int k = 0; k < 10; k++)
    {
        dacHPFcombo->addItem (String (HPFvalues[k]) + " Hz", 2 + k);
    }
    dacHPFcombo->setSelectedId (1, sendNotification);
    addAndMakeVisible (dacHPFcombo.get());

    ttlSettleLabel = std::make_unique<Label> ("TTL Settle", "TTL Settle");
    ttlSettleLabel->setFont (FontOptions ("Inter", "Regular", 10.0f));
    ttlSettleLabel->setBounds (xOffset + 255, 70, 70, 20);
    addAndMakeVisible (ttlSettleLabel.get());

    ttlSettleCombo = std::make_unique<ComboBox> ("FastSettleComboBox");
    ttlSettleCombo->setBounds (xOffset + 260, 85, 60, 18);
    ttlSettleCombo->addListener (this);
    ttlSettleCombo->addItem ("-", 1);
    for (int k = 0; k < 8; k++)
    {
        ttlSettleCombo->addItem ("TTL" + String (1 + k), 2 + k);
    }
    ttlSettleCombo->setSelectedId (1, sendNotification);
    addAndMakeVisible (ttlSettleCombo.get());
    */

    /*
    ledButton = std::make_unique<UtilityButton> ("LED");
    ledButton->setRadius (3.0f);
    ledButton->setBounds (xOffset + 288, 108, 32, 18);
    ledButton->addListener (this);
    ledButton->setClickingTogglesState (true);
    ledButton->setTooltip ("Toggle board LEDs");
    ledButton->setToggleState (true, dontSendNotification);
    addAndMakeVisible (ledButton.get());
    */
}

void DeviceEditor::measureImpedances()
{
    if (! acquisitionIsActive)
    {
        board->measureImpedances();
    }
}

void DeviceEditor::impedanceMeasurementFinished()
{
    if (canvas != nullptr)
    {
        canvas->updateAsync();
    }
}

void DeviceEditor::saveImpedances (File& file)
{
    LOGD ("Saving impedances to ", file.getFullPathName());

    board->saveImpedances (file);
}

void DeviceEditor::updateSettings()
{
    if (canvas != nullptr)
    {
        canvas->update();
    }
}

void DeviceEditor::paint (Graphics& g)
{
    VisualizerEditor::paint (g);

    if (board != nullptr && board->getBoardType() == AcquisitionBoard::BoardType::RedPitaya)
    {
        g.setColour (findColour (ThemeColours::defaultText).withAlpha (0.25f));
        const int divTop = 12;
        const int divBottom = getHeight() - 8;
        for (int divX : { 135, 275, 410 })
            g.fillRect (divX, divTop, 1, divBottom - divTop);
    }
}

int DeviceEditor::getSelectedStreamSensorIndex() const
{
    if (sensorSelectCombo == nullptr || sensorSelectCombo->getNumItems() <= 0)
        return 0;

    const int id = sensorSelectCombo->getSelectedId();
    return jmax (0, id - 1);
}

void DeviceEditor::repopulateSensorRateComboForHwHz (int hwHz)
{
    if (sensorCfgRateCombo == nullptr)
        return;

    const int prevChoiceId = sensorCfgRateCombo->getSelectedId();

    sensorCfgRateCombo->clear (dontSendNotification);

    struct RateDiv { const char* label; int hz; };
    const RateDiv choices[] = {
        { "Same as HW", hwHz },
        { "HW / 2", jmax (1, hwHz / 2) },
        { "HW / 5", jmax (1, hwHz / 5) },
        { "HW / 10", jmax (1, hwHz / 10) },
        { "1 Hz", 1 },
        { "10 Hz", 10 },
        { "25 Hz", 25 },
    };

    for (int i = 0; i < (int) (sizeof (choices) / sizeof (choices[0])); ++i)
    {
        String item = String (choices[i].label) + " (" + String (choices[i].hz) + " Hz)";
        sensorCfgRateCombo->addItem (item, i + 1);
    }

    const int toSelect = jlimit (1, sensorCfgRateCombo->getNumItems(), prevChoiceId > 0 ? prevChoiceId : 1);
    sensorCfgRateCombo->setSelectedId (toSelect, dontSendNotification);
}

void DeviceEditor::refreshRedPitayaSensorCombosFromBoard()
{
    if (! redPitayaSensorUiBuilt || board == nullptr
        || board->getBoardType() != AcquisitionBoard::BoardType::RedPitaya)
        return;

    auto* rp = static_cast<AcqBoardRedPitaya*> (board);

    sensorSelectCombo->clear (dontSendNotification);

    const int n = rp->getStreamSensorCount();

    if (n <= 0)
    {
        sensorSelectCombo->addItem ("(no sensors)", 1);
        sensorSelectCombo->setSelectedId (1, dontSendNotification);
    }
    else
    {
        for (int i = 0; i < n; ++i)
        {
            const String nm = rp->getStreamSensorName (i);
            sensorSelectCombo->addItem ("[" + String (i) + "] " + (nm.isNotEmpty() ? nm : "sensor"), i + 1);
        }

        sensorSelectCombo->setSelectedId (1, dontSendNotification);
    }

    const int hz = sampleRateLabel != nullptr ? sampleRateLabel->getText().getIntValue() : 0;
    repopulateSensorRateComboForHwHz (hz > 0 ? hz : 100);

    sensorCfgAccelCombo->setSelectedId (1, dontSendNotification);
    sensorCfgGyroCombo->setSelectedId (1, dontSendNotification);
}

void DeviceEditor::comboBoxChanged (ComboBox* comboBox)
{
    if (redPitayaSensorUiBuilt && board != nullptr
        && board->getBoardType() == AcquisitionBoard::BoardType::RedPitaya
        && acquisitionIsActive)
    {
        auto* rp = static_cast<AcqBoardRedPitaya*> (board);
        const int si = getSelectedStreamSensorIndex();

        if (comboBox == sensorSelectCombo.get())
        {
            const int hz = sampleRateLabel != nullptr ? sampleRateLabel->getText().getIntValue() : 100;
            repopulateSensorRateComboForHwHz (hz > 0 ? hz : 100);

            sensorCfgAccelCombo->setSelectedId (1, dontSendNotification);
            sensorCfgGyroCombo->setSelectedId (1, dontSendNotification);

            if (acquisitionIsActive && board != nullptr
                && board->getBoardType() == AcquisitionBoard::BoardType::RedPitaya)
            {
                auto* rp = static_cast<AcqBoardRedPitaya*> (board);
                const int si = getSelectedStreamSensorIndex();
                rp->sendSensorCfgAcc (si, 0);
                rp->sendSensorCfgGyr (si, 0);

                const int hwHz = sampleRateLabel != nullptr ? sampleRateLabel->getText().getIntValue() : 100;
                const int hw = hwHz > 0 ? hwHz : 100;
                struct RateDiv { int hz; };
                const RateDiv choices[] = {
                    { hw },
                    { jmax (1, hw / 2) },
                    { jmax (1, hw / 5) },
                    { jmax (1, hw / 10) },
                    { 1 },
                    { 10 },
                    { 25 },
                };
                const int pick = sensorCfgRateCombo->getSelectedId() - 1;

                if (pick >= 0 && pick < (int) (sizeof (choices) / sizeof (choices[0])))
                    rp->sendSensorCfgSrate (si, choices[pick].hz);
            }

            return;
        }

        if (comboBox == sensorCfgAccelCombo.get())
        {
            rp->sendSensorCfgAcc (si, sensorCfgAccelCombo->getSelectedId() - 1);
            return;
        }

        if (comboBox == sensorCfgGyroCombo.get())
        {
            rp->sendSensorCfgGyr (si, sensorCfgGyroCombo->getSelectedId() - 1);
            return;
        }

        if (comboBox == sensorCfgRateCombo.get())
        {
            const int hwHz = sampleRateLabel != nullptr ? sampleRateLabel->getText().getIntValue() : 100;
            const int hw = hwHz > 0 ? hwHz : 100;

            struct RateDiv { int hz; };
            const RateDiv choices[] = {
                { hw },
                { jmax (1, hw / 2) },
                { jmax (1, hw / 5) },
                { jmax (1, hw / 10) },
                { 1 },
                { 10 },
                { 25 },
            };

            const int pick = sensorCfgRateCombo->getSelectedId() - 1;

            if (pick >= 0 && pick < (int) (sizeof (choices) / sizeof (choices[0])))
                rp->sendSensorCfgSrate (si, choices[pick].hz);

            return;
        }
    }

    /*
    if (comboBox == ttlSettleCombo.get())
    {
        int selectedChannel = ttlSettleCombo->getSelectedId();
        if (selectedChannel == 1)
        {
            board->setFastTTLSettle (false, 0);
        }
        else
        {
            board->setFastTTLSettle (true, selectedChannel - 2);
        }
    }
    else if (comboBox == dacHPFcombo.get())
    {
        int selection = dacHPFcombo->getSelectedId();
        if (selection == 1)
        {
            board->setDAChpf (100, false);
        }
        else
        {
            int HPFvalues[10] = { 50, 100, 200, 300, 400, 500, 600, 700, 800, 900 };
            board->setDAChpf (HPFvalues[selection - 2], true);
        }
    }
    */
}

void DeviceEditor::channelStateChanged (Array<int> newChannels)
{
    if (board == nullptr || int (activeAudioChannel) < 0 || int (activeAudioChannel) >= electrodeButtons.size())
    {
        return;
    }

    int selectedChannel = -1;

    if (newChannels.size() > 0)
    {
        selectedChannel = newChannels[0];
    }

    board->connectHeadstageChannelToDAC (selectedChannel, int (activeAudioChannel));

    if (selectedChannel > -1)
    {
        electrodeButtons[int (activeAudioChannel)]->setToggleState (true, dontSendNotification);
        electrodeButtons[int (activeAudioChannel)]->setChannelNum (selectedChannel + 1);
    }
    else
    {
        electrodeButtons[int (activeAudioChannel)]->setChannelNum (selectedChannel);
        electrodeButtons[int (activeAudioChannel)]->setToggleState (false, dontSendNotification);
    }
}

int DeviceEditor::getChannelCount()
{
    return board->getNumChannels();
}

void DeviceEditor::buttonClicked (Button* button)
{
    /*
    if (button == rescanButton.get() && ! acquisitionIsActive)
    {
        board->scanPorts();

        for (int i = 0; i < 4; i++)
        {
            headstageOptionsInterfaces[i]->checkEnabledState();
        }
        CoreServices::updateSignalChain (this);
    }
    else if (button == electrodeButtons[0] || button == electrodeButtons[1])
    {
        std::vector<bool> channelStates;

        if (button == electrodeButtons[0])
            activeAudioChannel = LEFT;
        else
            activeAudioChannel = RIGHT;

        for (int i = 0; i < board->getNumDataOutputs (ContinuousChannel::ELECTRODE); i++)
        {
            if (electrodeButtons[int (activeAudioChannel)]->getChannelNum() - 1 == i)
                channelStates.push_back (true);
            else
                channelStates.push_back (false);
        }

        auto* channelSelector = new PopupChannelSelector (this, this, channelStates);

        channelSelector->setChannelButtonColour (Colour (0, 174, 239));
        channelSelector->setMaximumSelectableChannels (1);

        CallOutBox& myBox = CallOutBox::launchAsynchronously (std::unique_ptr<Component> (channelSelector),
                                                              button->getScreenBounds(),
                                                              nullptr);
    }
    */
    if (button == recordButton.get() && acquisitionIsActive)
    {
        if (recordButton->getToggleState())
        {
            if (board->sendRecordOnCommand())
            {
                const String binPath = board->getLastRecordingPath();
                const String csvPath = board->getLastRecordingCsvPath();

                if (binPath.isNotEmpty() && csvPath.isNotEmpty())
                    CoreServices::sendStatusMessage ("Recording to BIN: " + binPath + " CSV: " + csvPath);
                else if (binPath.isNotEmpty())
                    CoreServices::sendStatusMessage ("Recording to " + binPath);
                else
                    CoreServices::sendStatusMessage ("Recording started.");
            }
        }
        else
        {
            if (board->sendRecordOffCommand())
            {
                const String binPath = board->getLastRecordingPath();
                const String csvPath = board->getLastRecordingCsvPath();

                if (binPath.isNotEmpty() && csvPath.isNotEmpty())
                    CoreServices::sendStatusMessage ("Recording saved. BIN: " + binPath + " CSV: " + csvPath);
                else if (binPath.isNotEmpty())
                    CoreServices::sendStatusMessage ("Recording saved to " + binPath);
                else
                    CoreServices::sendStatusMessage ("Recording stopped and saved.");
            }
        }
    }
    else if (button == filterButton.get())
    {
        if (board != nullptr)
        {
            board->setFilterEnabled (filterButton->getToggleState());
            CoreServices::sendStatusMessage (filterButton->getToggleState()
                                                 ? "Filter enabled for next data buffer."
                                                 : "Filter disabled for next data buffer.");
        }
    }
    /*
    else if (button == auxButton.get() && ! acquisitionIsActive)
    {
        board->enableAuxChannels (button->getToggleState());
        LOGD ("AUX Button toggled");
        CoreServices::updateSignalChain (this);
    }
    else if (button == adcButton.get() && ! acquisitionIsActive)
    {
        board->enableAdcChannels (button->getToggleState());
        LOGD ("ADC Button toggled");
        CoreServices::updateSignalChain (this);
    }
    else if (button == dacTTLButton.get())
    {
        board->setTTLOutputMode (dacTTLButton->getToggleState());
    }
    else if (button == dspoffsetButton.get() && ! acquisitionIsActive)
    {
        LOGD ("DSP offset ", button->getToggleState());
        board->setDspOffset (button->getToggleState());
    }
    else if (button == ledButton.get())
    {
        board->enableBoardLeds (button->getToggleState());
    }
    */
}

void DeviceEditor::startAcquisition()
{
    //rescanButton->setEnabledState (false);
    //auxButton->setEnabledState (false);
    //adcButton->setEnabledState (false);
    //dspoffsetButton->setEnabledState (false);

    // Allow filter to remain enabled during acquisition
    // if (filterButton != nullptr)
    //     filterButton->setEnabledState (false);

    if (filterButton != nullptr)
        filterButton->setEnabledState (true);

    if (analogInLabel != nullptr)
        analogInLabel->setEnabled (true);

    if (analogOutLabel != nullptr)
        analogOutLabel->setEnabled (true);

    for (auto headstageOptions : headstageOptionsInterfaces)
    {
        headstageOptions->setEnabled (false);
    }

    if (canvas != nullptr)
    {
        canvas->beginAnimation();
        // Right-column controls sit under the canvas z-order; bring strip to front for hit-testing.
        if (sampleRateTitle != nullptr)
            sampleRateTitle->toFront (false);
        if (sampleRateLabel != nullptr)
            sampleRateLabel->toFront (false);
        if (filterTitle != nullptr)
            filterTitle->toFront (false);
        if (filterButton != nullptr)
            filterButton->toFront (false);
        if (analogInTitle != nullptr)
            analogInTitle->toFront (false);
        if (analogInLabel != nullptr)
            analogInLabel->toFront (false);
        if (analogOutTitle != nullptr)
            analogOutTitle->toFront (false);
        if (analogOutLabel != nullptr)
            analogOutLabel->toFront (false);
        if (recordButton != nullptr)
            recordButton->toFront (false);
        if (sensorCfgAccelTitle != nullptr)
            sensorCfgAccelTitle->toFront (false);
        if (sensorCfgAccelCombo != nullptr)
            sensorCfgAccelCombo->toFront (false);
        if (sensorCfgGyroTitle != nullptr)
            sensorCfgGyroTitle->toFront (false);
        if (sensorCfgGyroCombo != nullptr)
            sensorCfgGyroCombo->toFront (false);
        if (sensorCfgRateTitle != nullptr)
            sensorCfgRateTitle->toFront (false);
        if (sensorCfgRateCombo != nullptr)
            sensorCfgRateCombo->toFront (false);
        if (sensorSelectTitle != nullptr)
            sensorSelectTitle->toFront (false);
        if (sensorSelectCombo != nullptr)
            sensorSelectCombo->toFront (false);
    }

    if (memoryUsage != nullptr)
        memoryUsage->startAcquisition();

    acquisitionIsActive = true;

    refreshRedPitayaSensorCombosFromBoard();
}

void DeviceEditor::stopAcquisition()
{
    //rescanButton->setEnabledState (true);
    //auxButton->setEnabledState (true);
    //adcButton->setEnabledState (true);
    //dspoffsetButton->setEnabledState (true);

    if (filterButton != nullptr)
        filterButton->setEnabledState (true);

    if (analogInLabel != nullptr)
        analogInLabel->setEnabled (true);

    if (analogOutLabel != nullptr)
        analogOutLabel->setEnabled (true);

    for (auto headstageOptions : headstageOptionsInterfaces)
    {
        headstageOptions->setEnabled (true);
    }

    if (canvas != nullptr)
    {
        canvas->endAnimation();
    }

    if (memoryUsage != nullptr)
        memoryUsage->stopAcquisition();

    acquisitionIsActive = false;
}

// --- ADD THIS METHOD TO HANDLE YOUR NEW TEXT BOX ---
void DeviceEditor::labelTextChanged (Label* labelThatHasChanged)
{
    if (labelThatHasChanged == sampleRateLabel.get())
    {
        int newFreq = sampleRateLabel->getText().getIntValue();
        const int hwHz = jlimit (1, 2000, newFreq > 0 ? newFreq : 1000);

        if (board != nullptr)
        {
            if (board->getBoardType() == AcquisitionBoard::BoardType::RedPitaya)
            {
                board->setSampleRate (hwHz);
                CoreServices::updateSignalChain (this);
            }

            std::cout << "DeviceEditor: Board found. Dispatching updateSampleFrequency..." << std::endl;
            board->updateSampleFrequency (hwHz);
        }

        if (redPitayaSensorUiBuilt && acquisitionIsActive)
            repopulateSensorRateComboForHwHz (hwHz);
    }
    else if (labelThatHasChanged == analogInLabel.get())
    {
        float newGain = analogInLabel->getText().getFloatValue();

        if (newGain < 0.1f || newGain > 100.0f)
        {
            CoreServices::sendStatusMessage ("Analog In gain out of range (0.1 - 100.0).");
            analogInLabel->setText ("1.00", dontSendNotification);
            return;
        }

        if (board != nullptr)
            board->setAnalogInGain (newGain);
    }
    else if (labelThatHasChanged == analogOutLabel.get())
    {
        float newVoltage = analogOutLabel->getText().getFloatValue();

        if (newVoltage < 0.0f || newVoltage > 1.8f)
        {
            CoreServices::sendStatusMessage ("Analog Out voltage out of range (0.0 - 1.8 V).");
            analogOutLabel->setText ("0.00", dontSendNotification);
            return;
        }

        if (board != nullptr)
            board->setAnalogOutVoltage (newVoltage);
    }
}

void DeviceEditor::saveVisualizerEditorParameters (XmlElement* xml)
{
    if (board == nullptr)
    {
        if (previousSettings == nullptr)
        {
            return;
        }

        xml->setAttribute ("SampleRate", previousSettings->getIntAttribute ("SampleRate"));
        xml->setAttribute ("LowCut", previousSettings->getDoubleAttribute ("LowCut"));
        xml->setAttribute ("HighCut", previousSettings->getDoubleAttribute ("HighCut"));

        xml->setAttribute ("AUXsOn", previousSettings->getBoolAttribute ("AUXsOn"));
        xml->setAttribute ("ADCsOn", previousSettings->getBoolAttribute ("ADCsOn"));
        xml->setAttribute ("NoiseSlicer", previousSettings->getIntAttribute ("NoiseSlicer"));
        xml->setAttribute ("TTLFastSettle", previousSettings->getIntAttribute ("TTLFastSettle"));
        xml->setAttribute ("DAC_TTL", previousSettings->getBoolAttribute ("DAC_TTL"));
        xml->setAttribute ("DAC_HPF", previousSettings->getIntAttribute ("DAC_HPF"));
        xml->setAttribute ("DSPOffset", previousSettings->getBoolAttribute ("DSPOffset"));
        xml->setAttribute ("DSPCutoffFreq", previousSettings->getDoubleAttribute ("DSPCutoffFreq"));
        xml->setAttribute ("LEDs", previousSettings->getBoolAttribute ("LEDs", true));
        xml->setAttribute ("ClockDivideRatio", previousSettings->getIntAttribute ("ClockDivideRatio"));
        xml->setAttribute ("Channel_Naming_Scheme", previousSettings->getIntAttribute ("Channel_Naming_Scheme", 0));
        xml->setAttribute ("AudioOutputL", previousSettings->getIntAttribute ("AudioOutputL"));
        xml->setAttribute ("AudioOutputR", previousSettings->getIntAttribute ("AudioOutputR"));
        xml->setAttribute ("FilterEnabled", previousSettings->getBoolAttribute ("FilterEnabled", false));
        xml->setAttribute ("AnalogInGain", previousSettings->getDoubleAttribute ("AnalogInGain", 1.0));
        xml->setAttribute ("AnalogOutVoltage", previousSettings->getDoubleAttribute ("AnalogOutVoltage", 0.0));

        forEachXmlChildElementWithTagName (*previousSettings, hsOptions, "HSOPTIONS")
        {
            XmlElement* newHsOptions = xml->createNewChildElement ("HSOPTIONS");
            int index = hsOptions->getIntAttribute ("index", -1);

            newHsOptions->setAttribute ("index", index);
            newHsOptions->setAttribute ("hs1_full_channels", hsOptions->getBoolAttribute ("hs1_full_channels", true));
            newHsOptions->setAttribute ("hs2_full_channels", hsOptions->getBoolAttribute ("hs2_full_channels", true));
        }

        return;
    }

    /*
    xml->setAttribute ("SampleRate", sampleRateInterface->getSelectedId());
    xml->setAttribute ("LowCut", bandwidthInterface->getLowerBandwidth());
    xml->setAttribute ("HighCut", bandwidthInterface->getUpperBandwidth());
    xml->setAttribute ("AUXsOn", auxButton->getToggleState());
    xml->setAttribute ("ADCsOn", adcButton->getToggleState());
    */
    //xml->setAttribute ("AudioOutputL", electrodeButtons[0]->getChannelNum());
    //xml->setAttribute ("AudioOutputR", electrodeButtons[1]->getChannelNum());
    /*
    xml->setAttribute ("NoiseSlicer", audioInterface->getNoiseSlicerLevel());
    xml->setAttribute ("TTLFastSettle", ttlSettleCombo->getSelectedId());
    xml->setAttribute ("DAC_TTL", dacTTLButton->getToggleState());
    xml->setAttribute ("DAC_HPF", dacHPFcombo->getSelectedId());
    xml->setAttribute ("DSPOffset", dspoffsetButton->getToggleState());
    xml->setAttribute ("DSPCutoffFreq", dspInterface->getDspCutoffFreq());
    xml->setAttribute ("LEDs", ledButton->getToggleState());
    xml->setAttribute ("ClockDivideRatio", clockInterface->getClockDivideRatio());
    */

    // loop through all headstage options interfaces and save their parameters
    for (int i = 0; i < 4; i++)
    {
        XmlElement* hsOptions = xml->createNewChildElement ("HSOPTIONS");
        hsOptions->setAttribute ("index", i);
        // Note: Make sure headstageOptionsInterfaces vector was properly populated if you intend to save this.
        // If you commented out their creation, this might also cause an issue or just save nothing.
        if (headstageOptionsInterfaces.size() > i && headstageOptionsInterfaces[i] != nullptr)
        {
            hsOptions->setAttribute ("hs1_full_channels", headstageOptionsInterfaces[i]->is32Channel (0));
            hsOptions->setAttribute ("hs2_full_channels", headstageOptionsInterfaces[i]->is32Channel (1));
        }
    }

    // save channel naming scheme
    xml->setAttribute ("Channel_Naming_Scheme", board->getNamingScheme());

    xml->setAttribute ("FilterEnabled", filterButton->getToggleState());
    xml->setAttribute ("AnalogInGain", analogInLabel->getText().getFloatValue());
    xml->setAttribute ("AnalogOutVoltage", analogOutLabel->getText().getFloatValue());
}

void DeviceEditor::loadVisualizerEditorParameters (XmlElement* xml)
{
    if (board == nullptr)
    {
        previousSettings = std::make_unique<XmlElement> ("DeviceEditorSettings");
        previousSettings->setAttribute ("SampleRate", xml->getIntAttribute ("SampleRate"));
        previousSettings->setAttribute ("LowCut", xml->getDoubleAttribute ("LowCut"));
        previousSettings->setAttribute ("HighCut", xml->getDoubleAttribute ("HighCut"));
        previousSettings->setAttribute ("AUXsOn", xml->getBoolAttribute ("AUXsOn"));
        previousSettings->setAttribute ("ADCsOn", xml->getBoolAttribute ("ADCsOn"));
        previousSettings->setAttribute ("NoiseSlicer", xml->getIntAttribute ("NoiseSlicer"));
        previousSettings->setAttribute ("TTLFastSettle", xml->getIntAttribute ("TTLFastSettle"));
        previousSettings->setAttribute ("DAC_TTL", xml->getBoolAttribute ("DAC_TTL"));
        previousSettings->setAttribute ("DAC_HPF", xml->getIntAttribute ("DAC_HPF"));
        previousSettings->setAttribute ("DSPOffset", xml->getBoolAttribute ("DSPOffset"));
        previousSettings->setAttribute ("DSPCutoffFreq", xml->getDoubleAttribute ("DSPCutoffFreq"));
        previousSettings->setAttribute ("LEDs", xml->getBoolAttribute ("LEDs", true));
        previousSettings->setAttribute ("ClockDivideRatio", xml->getIntAttribute ("ClockDivideRatio"));
        previousSettings->setAttribute ("Channel_Naming_Scheme", xml->getIntAttribute ("Channel_Naming_Scheme", 0));
        previousSettings->setAttribute ("AudioOutputL", xml->getIntAttribute ("AudioOutputL"));
        previousSettings->setAttribute ("AudioOutputR", xml->getIntAttribute ("AudioOutputR"));
        previousSettings->setAttribute ("FilterEnabled", xml->getBoolAttribute ("FilterEnabled", false));
        previousSettings->setAttribute ("AnalogInGain", xml->getDoubleAttribute ("AnalogInGain", 1.0));
        previousSettings->setAttribute ("AnalogOutVoltage", xml->getDoubleAttribute ("AnalogOutVoltage", 0.0));

        forEachXmlChildElementWithTagName (*xml, hsOptions, "HSOPTIONS")
        {
            XmlElement* newHsOptions = previousSettings->createNewChildElement ("HSOPTIONS");
            int index = hsOptions->getIntAttribute ("index", -1);

            newHsOptions->setAttribute ("index", index);
            newHsOptions->setAttribute ("hs1_full_channels", hsOptions->getBoolAttribute ("hs1_full_channels", true));
            newHsOptions->setAttribute ("hs2_full_channels", hsOptions->getBoolAttribute ("hs2_full_channels", true));
        }

        return;
    }

    /*
    int sampleRateId = xml->getIntAttribute ("SampleRate");
    Array<int> sampleRates = board->getAvailableSampleRates();
    if (! sampleRates.contains (sampleRateId))
    {
        sampleRateId = sampleRates.getLast(); // if the requested sample rate is not available, use the last one in the list
    }
    sampleRateInterface->setSelectedId (sampleRateId);
    bandwidthInterface->setLowerBandwidth (xml->getDoubleAttribute ("LowCut"));
    bandwidthInterface->setUpperBandwidth (xml->getDoubleAttribute ("HighCut"));
    auxButton->setToggleState (xml->getBoolAttribute ("AUXsOn"), sendNotification);
    adcButton->setToggleState (xml->getBoolAttribute ("ADCsOn"), sendNotification);

    audioInterface->setNoiseSlicerLevel (xml->getIntAttribute ("NoiseSlicer"));
    ttlSettleCombo->setSelectedId (xml->getIntAttribute ("TTLFastSettle"));
    dacTTLButton->setToggleState (xml->getBoolAttribute ("DAC_TTL"), sendNotification);
    dacHPFcombo->setSelectedId (xml->getIntAttribute ("DAC_HPF"));
    dspoffsetButton->setToggleState (xml->getBoolAttribute ("DSPOffset"), sendNotification);
    dspInterface->setDspCutoffFreq (xml->getDoubleAttribute ("DSPCutoffFreq"));
    ledButton->setToggleState (xml->getBoolAttribute ("LEDs", true), sendNotification);
    clockInterface->setClockDivideRatio (xml->getIntAttribute ("ClockDivideRatio"));
    */

    forEachXmlChildElementWithTagName (*xml, hsOptions, "HSOPTIONS")
    {
        int index = hsOptions->getIntAttribute ("index", -1);

        if (index > -1 && headstageOptionsInterfaces.size() > index && headstageOptionsInterfaces[index] != nullptr)
        {
            headstageOptionsInterfaces[index]->set32Channel (0, hsOptions->getBoolAttribute ("hs1_full_channels", true));
            headstageOptionsInterfaces[index]->set32Channel (1, hsOptions->getBoolAttribute ("hs2_full_channels", true));
        }
    }

    bool filterOn = xml->getBoolAttribute ("FilterEnabled", false);
    filterButton->setToggleState (filterOn, dontSendNotification);
    board->setFilterEnabled (filterOn);

    float ainGain = jlimit (0.1f, 100.0f, (float) xml->getDoubleAttribute ("AnalogInGain", 1.0));
    analogInLabel->setText (String (ainGain, 2), dontSendNotification);
    board->setAnalogInGain (ainGain);

    float aoutVoltage = jlimit (0.0f, 1.8f, (float) xml->getDoubleAttribute ("AnalogOutVoltage", 0.0));
    analogOutLabel->setText (String (aoutVoltage, 2), dontSendNotification);
    board->setAnalogOutVoltage (aoutVoltage);

    // load channel naming scheme
    board->setNamingScheme ((ChannelNamingScheme) xml->getIntAttribute ("Channel_Naming_Scheme", 0));
}

Visualizer* DeviceEditor::createNewCanvas()
{
    GenericProcessor* processor = (GenericProcessor*) getProcessor();

    if (board == nullptr)
    {
        return nullptr;
    }

    canvas = new ChannelCanvas (board, this);

    return canvas;
}

// Bandwidth Options --------------------------------------------------------------------

BandwidthInterface::BandwidthInterface (AcquisitionBoard* board_,
                                        DeviceEditor* editor_) : board (board_),
                                                                 editor (editor_)
{
    name = "Bandwidth";

    lastHighCutString = "7500";
    lastLowCutString = "1";

    actualUpperBandwidth = 7500.0f;
    actualLowerBandwidth = 1.0f;

    upperBandwidthSelection = std::make_unique<Label> ("UpperBandwidth", lastHighCutString); // this is currently set in DeviceThread, the cleaner way would be to set it here again
    upperBandwidthSelection->setEditable (true, false, false);
    upperBandwidthSelection->addListener (this);
    upperBandwidthSelection->setBounds (25, 25, 50, 20);
    addAndMakeVisible (upperBandwidthSelection.get());

    lowerBandwidthSelection = std::make_unique<Label> ("LowerBandwidth", lastLowCutString);
    lowerBandwidthSelection->setEditable (true, false, false);
    lowerBandwidthSelection->addListener (this);
    lowerBandwidthSelection->setBounds (25, 10, 50, 20);
    addAndMakeVisible (lowerBandwidthSelection.get());
}

BandwidthInterface::~BandwidthInterface()
{
}

void BandwidthInterface::labelTextChanged (Label* label)
{
    if (! (editor->acquisitionIsActive) && board->foundInputSource())
    {
        if (label == upperBandwidthSelection.get())
        {
            Value val = label->getTextValue();
            double requestedValue = double (val.getValue());

            if (requestedValue < 100.0 || requestedValue > 20000.0 || requestedValue < lastLowCutString.getFloatValue())
            {
                CoreServices::sendStatusMessage ("Value out of range.");

                label->setText (lastHighCutString, dontSendNotification);

                return;
            }

            actualUpperBandwidth = board->setUpperBandwidth (requestedValue);

            LOGD ("Setting Upper Bandwidth to ", requestedValue);
            LOGD ("Actual Upper Bandwidth:  ", actualUpperBandwidth);
            label->setText (String (round (actualUpperBandwidth * 10.f) / 10.f), dontSendNotification);
        }
        else
        {
            Value val = label->getTextValue();
            double requestedValue = double (val.getValue());

            if (requestedValue < 0.1 || requestedValue > 500.0 || requestedValue > lastHighCutString.getFloatValue())
            {
                CoreServices::sendStatusMessage ("Value out of range.");

                label->setText (lastLowCutString, dontSendNotification);

                return;
            }

            actualLowerBandwidth = board->setLowerBandwidth (requestedValue);

            LOGD ("Setting Lower Bandwidth to ", requestedValue);
            LOGD ("Actual Lower Bandwidth:  ", actualLowerBandwidth);

            label->setText (String (round (actualLowerBandwidth * 10.f) / 10.f), dontSendNotification);
        }
    }
    else if (editor->acquisitionIsActive)
    {
        CoreServices::sendStatusMessage ("Can't change bandwidth while acquisition is active!");
        if (label == upperBandwidthSelection.get())
            label->setText (lastHighCutString, dontSendNotification);
        else
            label->setText (lastLowCutString, dontSendNotification);
        return;
    }
}

void BandwidthInterface::setLowerBandwidth (double value)
{
    actualLowerBandwidth = board->setLowerBandwidth (value);
    lowerBandwidthSelection->setText (String (round (actualLowerBandwidth * 10.f) / 10.f), dontSendNotification);
}

void BandwidthInterface::setUpperBandwidth (double value)
{
    actualUpperBandwidth = board->setUpperBandwidth (value);
    upperBandwidthSelection->setText (String (round (actualUpperBandwidth * 10.f) / 10.f), dontSendNotification);
}

double BandwidthInterface::getLowerBandwidth()
{
    return actualLowerBandwidth;
}

double BandwidthInterface::getUpperBandwidth()
{
    return actualUpperBandwidth;
}

void BandwidthInterface::paint (Graphics& g)
{
    g.setColour (findColour (ThemeColours::defaultText));

    g.setFont (FontOptions ("Inter", "Regular", 10.0f));

    g.drawText (name, 0, 0, 200, 11, Justification::left, false);

    g.drawText ("Low:", 0, 11, 200, 15, Justification::left, false);

    g.drawText ("High:", 0, 26, 200, 15, Justification::left, false);
}

// Sample rate Options --------------------------------------------------------------------

SampleRateInterface::SampleRateInterface (AcquisitionBoard* board_,
                                          DeviceEditor* editor_) : board (board_),
                                                                   editor (editor_)
{
    name = "Sample Rate";

    Array<int> sampleRates = board->getAvailableSampleRates();

    rateSelection = std::make_unique<ComboBox> ("Sample Rate");
    rateSelection->addItemList (sampleRateOptions, 1);
    for (auto sampleRate : sampleRates)
    {
        int numDecimals = sampleRate < 10000 ? 2 : 1;

        rateSelection->addItem (String (float (sampleRate) / 1000.0f, numDecimals) + " kS/s", sampleRate);
    }

    rateSelection->setSelectedId (sampleRates.getLast(), dontSendNotification);
    rateSelection->addListener (this);
    rateSelection->setBounds (0, 14, 80, 20);
    addAndMakeVisible (rateSelection.get());
}

SampleRateInterface::~SampleRateInterface()
{
}

void SampleRateInterface::comboBoxChanged (ComboBox* cb)
{
    if (! (editor->acquisitionIsActive) && board->foundInputSource())
    {
        if (cb == rateSelection.get())
        {
            board->setSampleRate (cb->getSelectedId());

            LOGD ("Setting sample rate to ", cb->getSelectedId());

            CoreServices::updateSignalChain (editor);
        }
    }
}

int SampleRateInterface::getSelectedId()
{
    return rateSelection->getSelectedId();
}

void SampleRateInterface::setSelectedId (int id)
{
    rateSelection->setSelectedId (id);
}

String SampleRateInterface::getText()
{
    return rateSelection->getText();
}

void SampleRateInterface::paint (Graphics& g)
{
    g.setColour (findColour (ThemeColours::defaultText));

    g.setFont (FontOptions ("Inter", "Regular", 10.0f));

    g.drawText (name, 0, 0, 80, 15, Justification::left, false);
}

// Headstage Options --------------------------------------------------------------------

HeadstageOptionsInterface::HeadstageOptionsInterface (AcquisitionBoard* board_,
                                                      DeviceEditor* editor_,
                                                      int hsNum) : isEnabled (false),
                                                                   board (board_),
                                                                   editor (editor_)
{
    switch (hsNum)
    {
        case 0:
            name = "A";
            break;
        case 1:
            name = "B";
            break;
        case 2:
            name = "C";
            break;
        case 3:
            name = "D";
            break;
        default:
            name = "X";
    }

    hsNumber1 = hsNum * 2; // data stream 1
    hsNumber2 = hsNumber1 + 1; // data stream 2

    channelsOnHs1 = 0;
    channelsOnHs2 = 0;

    hsButton1 = std::make_unique<UtilityButton> (" ");
    hsButton1->setRadius (3.0f);
    hsButton1->setBounds (23, 1, 20, 17);
    hsButton1->setEnabledState (false);
    hsButton1->setCorners (true, false, true, false);
    hsButton1->addListener (this);
    addAndMakeVisible (hsButton1.get());

    hsButton2 = std::make_unique<UtilityButton> (" ");
    hsButton2->setRadius (3.0f);
    hsButton2->setBounds (43, 1, 20, 17);
    hsButton2->setEnabledState (false);
    hsButton2->setCorners (false, true, false, true);
    hsButton2->addListener (this);
    addAndMakeVisible (hsButton2.get());

    checkEnabledState();
}

HeadstageOptionsInterface::~HeadstageOptionsInterface()
{
}

void HeadstageOptionsInterface::setEnabled (bool state)
{
    hsButton1->setEnabledState (state);
    hsButton2->setEnabledState (state);
}

void HeadstageOptionsInterface::checkEnabledState()
{
    LOGD ("Checking enabled state of HS ", hsNumber1, " and HS ", hsNumber2);

    isEnabled = (board->isHeadstageEnabled (hsNumber1) || board->isHeadstageEnabled (hsNumber2));

    LOGD ("Is enabled: ", isEnabled);

    if (board->isHeadstageEnabled (hsNumber1))
    {
        channelsOnHs1 = board->getActiveChannelsInHeadstage (hsNumber1);
        String label = channelsOnHs1 == 0 ? "IMU" : String (channelsOnHs1); // NB: No active channels indicates a BNO is connected instead
        hsButton1->setLabel (label);
        hsButton1->setEnabledState (true);
        hsButton1->setToggleState (true, dontSendNotification);
    }
    else
    {
        channelsOnHs1 = 0;
        hsButton1->setLabel (" ");
        hsButton1->setEnabledState (false);
        hsButton1->setToggleState (false, dontSendNotification);
    }

    LOGD ("Channels on HS1: ", channelsOnHs1);

    if (board->isHeadstageEnabled (hsNumber2))
    {
        channelsOnHs2 = board->getActiveChannelsInHeadstage (hsNumber2);
        String label = channelsOnHs2 == 0 ? "IMU" : String (channelsOnHs2); // NB: No active channels indicates a BNO is connected instead
        hsButton2->setLabel (label);
        hsButton2->setEnabledState (true);
        hsButton2->setToggleState (true, dontSendNotification);
    }
    else
    {
        channelsOnHs2 = 0;
        hsButton2->setLabel (" ");
        hsButton2->setEnabledState (false);
        hsButton2->setToggleState (false, dontSendNotification);
    }

    LOGD ("Channels on HS2: ", channelsOnHs1);

    repaint();
}

void HeadstageOptionsInterface::buttonClicked (Button* button)
{
    if (! (editor->acquisitionIsActive) && board->foundInputSource())
    {
        if ((button == hsButton1.get()) && (board->getChannelsInHeadstage (hsNumber1) == 32))
        {
            if (channelsOnHs1 == 32)
                channelsOnHs1 = 16;
            else
                channelsOnHs1 = 32;

            hsButton1->setLabel (String (channelsOnHs1));
            board->setNumHeadstageChannels (hsNumber1, channelsOnHs1);
            CoreServices::updateSignalChain (editor);
        }
        else if ((button == hsButton2.get()) && (board->getChannelsInHeadstage (hsNumber2) == 32))
        {
            if (channelsOnHs2 == 32)
                channelsOnHs2 = 16;
            else
                channelsOnHs2 = 32;

            hsButton2->setLabel (String (channelsOnHs2));
            board->setNumHeadstageChannels (hsNumber2, channelsOnHs2);
            CoreServices::updateSignalChain (editor);
        }
    }
}

bool HeadstageOptionsInterface::is32Channel (int hsIndex)
{
    if (hsIndex == 0)
        return channelsOnHs1 == 32;

    else if (hsIndex == 1)
        return channelsOnHs2 == 32;

    return false;
}

void HeadstageOptionsInterface::set32Channel (int hsIndex, bool is32Channel)
{
    if (hsIndex == 0 && (board->getChannelsInHeadstage (hsNumber1) == 32))
    {
        if (is32Channel)
            channelsOnHs1 = 32;
        else
            channelsOnHs1 = 16;

        hsButton1->setLabel (String (channelsOnHs1));
        board->setNumHeadstageChannels (hsNumber1, channelsOnHs1);
    }

    else if (hsIndex == 1 && (board->getChannelsInHeadstage (hsNumber2) == 32))
    {
        if (is32Channel)
            channelsOnHs2 = 32;
        else
            channelsOnHs2 = 16;

        hsButton2->setLabel (String (channelsOnHs2));
        board->setNumHeadstageChannels (hsNumber1, channelsOnHs1);
    }
}

void HeadstageOptionsInterface::paint (Graphics& g)
{
    g.setColour (findColour (ThemeColours::componentBackground).darker (0.2f));

    g.fillRoundedRectangle (5, 0, getWidth() - 10, getHeight(), 4.0f);

    g.setColour (findColour (ThemeColours::defaultText));

    g.setFont (FontOptions ("Inter", "Regular", 15.0f));

    g.drawText (name, 10, 2, 200, 15, Justification::left, false);
}

// (Direct OpalKelly) Audio Options --------------------------------------------------------------------

AudioInterface::AudioInterface (AcquisitionBoard* board_,
                                DeviceEditor* editor_) : board (board_),
                                                         editor (editor_)
{
    name = "Noise";

    lastNoiseSlicerString = "0";

    actualNoiseSlicerLevel = 0.0f;

    noiseSlicerLevelSelection = std::make_unique<Label> ("Noise Slicer", lastNoiseSlicerString); // this is currently set in DeviceThread, the cleaner would be to set it here again
    noiseSlicerLevelSelection->setEditable (true, false, false);
    noiseSlicerLevelSelection->addListener (this);
    noiseSlicerLevelSelection->setBounds (35, 0, 35, 20);
    noiseSlicerLevelSelection->setJustificationType (Justification::bottomLeft);
    addAndMakeVisible (noiseSlicerLevelSelection.get());
}

AudioInterface::~AudioInterface()
{
}

void AudioInterface::labelTextChanged (Label* label)
{
    if (board->foundInputSource())
    {
        if (label == noiseSlicerLevelSelection.get())
        {
            Value val = label->getTextValue();
            int requestedValue = int (val.getValue()); // Note that it might be nice to translate to actual uV levels (16*value)

            if (requestedValue < 0 || requestedValue > 127)
            {
                CoreServices::sendStatusMessage ("Value out of range.");

                label->setText (lastNoiseSlicerString, dontSendNotification);

                return;
            }

            actualNoiseSlicerLevel = board->setNoiseSlicerLevel (requestedValue);

            LOGD ("Setting Noise Slicer Level to ", requestedValue);
            label->setText (String ((roundToInt) (actualNoiseSlicerLevel)), dontSendNotification);
        }
    }
    else
    {
        Value val = label->getTextValue();
        int requestedValue = int (val.getValue()); // Note that it might be nice to translate to actual uV levels (16*value)
        if (requestedValue < 0 || requestedValue > 127)
        {
            CoreServices::sendStatusMessage ("Value out of range.");
            label->setText (lastNoiseSlicerString, dontSendNotification);
            return;
        }
    }
}

void AudioInterface::setNoiseSlicerLevel (int value)
{
    actualNoiseSlicerLevel = board->setNoiseSlicerLevel (value);
    noiseSlicerLevelSelection->setText (String (roundToInt (actualNoiseSlicerLevel)), dontSendNotification);
}

int AudioInterface::getNoiseSlicerLevel()
{
    return actualNoiseSlicerLevel;
}

void AudioInterface::paint (Graphics& g)
{
    g.setColour (findColour (ThemeColours::defaultText));
    g.setFont (FontOptions ("Inter", "Regular", 10.0f));
    g.drawText (name, 0, 0, 35, 10, Justification::left, false);
    g.drawText ("Slicer:", 0, 10, 35, 10, Justification::left, false);
}

// Clock Divider options
ClockDivideInterface::ClockDivideInterface (AcquisitionBoard* board_,
                                            DeviceEditor* editor_) : name ("Clock"),
                                                                     lastDivideRatioString ("1"),
                                                                     board (board_),
                                                                     editor (editor_),
                                                                     actualDivideRatio (1)

{
    divideRatioSelection = std::make_unique<Label> ("Clock Divider", lastDivideRatioString);
    divideRatioSelection->setEditable (true, false, false);
    divideRatioSelection->addListener (this);
    divideRatioSelection->setBounds (35, 0, 35, 20);
    addAndMakeVisible (divideRatioSelection.get());
}

void ClockDivideInterface::labelTextChanged (Label* label)
{
    if (board->foundInputSource())
    {
        if (label == divideRatioSelection.get())
        {
            Value val = label->getTextValue();
            int requestedValue = int (val.getValue());

            if (requestedValue < 1 || requestedValue > 65534)
            {
                CoreServices::sendStatusMessage ("Value must be between 1 and 65534.");
                label->setText (lastDivideRatioString, dontSendNotification);
                return;
            }

            actualDivideRatio = board->setClockDivider (requestedValue);
            lastDivideRatioString = String (actualDivideRatio);

            LOGD ("Setting clock divide ratio to ", actualDivideRatio);
            label->setText (lastDivideRatioString, dontSendNotification);
        }
    }
}

void ClockDivideInterface::setClockDivideRatio (int value)
{
    actualDivideRatio = board->setClockDivider (value);
    divideRatioSelection->setText (String (actualDivideRatio), dontSendNotification);
}

void ClockDivideInterface::paint (Graphics& g)
{
    g.setColour (findColour (ThemeColours::defaultText));
    g.setFont (FontOptions ("Inter", "Regular", 10.0f));
    g.drawText (name, 0, 0, 35, 10, Justification::left, false);
    g.drawText ("Divider: ", 0, 10, 35, 10, Justification::left, false);
}

// DSP Options --------------------------------------------------------------------

DSPInterface::DSPInterface (AcquisitionBoard* board_,
                            DeviceEditor* editor_) : board (board_),
                                                     editor (editor_)
{
    name = "DSP";

    dspOffsetSelection = std::make_unique<Label> ("DspOffsetSelection",
                                                  String (round (board->getDspCutoffFreq() * 10.f) / 10.f));
    dspOffsetSelection->setEditable (true, false, false);
    dspOffsetSelection->addListener (this);
    dspOffsetSelection->setBounds (0, 0, 35, 20);
    addAndMakeVisible (dspOffsetSelection.get());
}

DSPInterface::~DSPInterface()
{
}

void DSPInterface::labelTextChanged (Label* label)
{
    if (! (editor->acquisitionIsActive) && board->foundInputSource())
    {
        if (label == dspOffsetSelection.get())
        {
            Value val = label->getTextValue();
            double requestedValue = double (val.getValue());

            actualDspCutoffFreq = board->setDspCutoffFreq (requestedValue);

            LOGD ("Setting DSP Cutoff Freq to ", requestedValue);
            LOGD ("Actual DSP Cutoff Freq:  ", actualDspCutoffFreq);
            label->setText (String (round (actualDspCutoffFreq * 10.f) / 10.f), dontSendNotification);
        }
    }
    else if (editor->acquisitionIsActive)
    {
        CoreServices::sendStatusMessage ("Can't change DSP cutoff while acquisition is active!");
    }
}

void DSPInterface::setDspCutoffFreq (double value)
{
    actualDspCutoffFreq = board->setDspCutoffFreq (value);
    dspOffsetSelection->setText (String (round (actualDspCutoffFreq * 10.f) / 10.f), dontSendNotification);
}

double DSPInterface::getDspCutoffFreq()
{
    return actualDspCutoffFreq;
}

void DSPInterface::paint (Graphics& g)
{
    g.setColour (findColour (ThemeColours::defaultText));
    g.setFont (FontOptions ("Inter", "Regular", 10.0f));
}

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

#ifndef __ACQUISITIONBOARD_H_2C4CBD67__
#define __ACQUISITIONBOARD_H_2C4CBD67__

#include <DataThreadHeaders.h>

#include "../DeviceEditor.h"
#include "Headstage.h"
#include "ImpedanceMeter.h"

/*
    A simple container holding one digital-output request.
    Think of it like a sticky note: "please turn TTL line #N on" or "please turn it off."
    These get queued up and processed by the run() thread so the UI doesn't have to wait.
*/
struct DigitalOutputCommand
{
    int ttlLine;  // which TTL output line (0-7) to change
    bool state;   // true = turn ON (rising edge), false = turn OFF (falling edge)
};

/*
    ============================================================
    AcquisitionBoard — the universal "job description" for all boards
    ============================================================

    This is a blueprint / contract that ALL boards must follow.
    Think of it like a job posting: it says WHAT any acquisition board
    must be able to do, but says nothing about HOW. Each concrete board
    type (OpalKelly, ONI, RedPitaya, Simulated) fills in the HOW by
    inheriting from this class and implementing every pure-virtual (= 0)
    method.

    Why bother? Because the rest of Open Ephys (DeviceThread, DeviceEditor,
    etc.) can talk to *any* board through this single interface without
    knowing the hardware details. Swap the board → only the subclass changes.

    The "= 0" after a method means: "I don't know how to do this — my
    subclass MUST provide an implementation or it won't compile."

    AcquisitionBoard also inherits from JUCE's Thread so it can run its
    data-collection loop (run()) on a background thread without blocking
    the UI.
*/

class AcquisitionBoard : public Thread
{
public:
    /** Constructor */
    AcquisitionBoard () : Thread ("Acquisition Board")
    {
        buffer = nullptr;
    }

    /** Destructor */
    virtual ~AcquisitionBoard() {}

    /** Detects whether a board is present */
    virtual bool detectBoard() = 0;

    /** Initializes board after successful detection */
    virtual bool initializeBoard() = 0;

    /** Returns true if the device is connected */
    virtual bool foundInputSource() const = 0;

    /** Returns an array of connected headstages for this board */
    virtual Array<const Headstage*> getHeadstages() = 0;

    /** Returns available sample rates */
    virtual Array<int> getAvailableSampleRates() = 0;

    /** Set sample rate */
    virtual void setSampleRate (int sampleRateHz) = 0;

    /** Get current sample rate */
    virtual float getSampleRate() const = 0;

    /** Checks for connected headstages */
    virtual void scanPorts() = 0;

    /** Enables AUX channel out */
    virtual void enableAuxChannels (bool enabled) = 0;

    /** Checks whether AUX channels are enabled */
    virtual bool areAuxChannelsEnabled() const = 0;

    /** Enables ADC channel out */
    virtual void enableAdcChannels (bool enabled) = 0;

    /** Checks whether ADC channels are enabled */
    virtual bool areAdcChannelsEnabled() const = 0;

    /** Returns bitVolts scaling value for each channel type */
    virtual float getBitVolts (ContinuousChannel::Type) const = 0;

    /** Measures impedance of each channel */
    virtual void measureImpedances() = 0;

    /** Called when impedance measurement is complete */
    virtual void impedanceMeasurementFinished() = 0;

    /** Save impedance measurements to XML*/
    virtual void saveImpedances (File& file) = 0;

    /** Sets the method for determining channel names*/
    virtual void setNamingScheme (ChannelNamingScheme scheme) = 0;

    /** Gets the method for determining channel names*/
    virtual ChannelNamingScheme getNamingScheme() = 0;

    virtual bool isReady() = 0;

    /** Initiates data transfer */
    virtual bool startAcquisition() = 0;

    /** Stops data transfer */
    virtual bool stopAcquisition() = 0;

    /** Sets analog filter upper limit; returns actual value */
    virtual double setUpperBandwidth (double upperBandwidth) = 0;

    /** Sets analog filter lower limit; returns actual value */
    virtual double setLowerBandwidth (double lowerBandwidth) = 0;

    /** Sets DSP cutoff frequency; returns actual value */
    virtual double setDspCutoffFreq (double freq) = 0;

    /** Gets the current DSP cutoff frequency */
    virtual double getDspCutoffFreq() const = 0;

    /** Sets whether DSP offset is enabled */
    virtual void setDspOffset (bool enabled) = 0;

    /** Sets whether TTL output mode is enabled */
    virtual void setTTLOutputMode (bool enabled) = 0;

    /** Sets whether DAC highpass filter is enabled, and set the cutoff freq */
    virtual void setDAChpf (float cutoff, bool enabled) = 0;

    /** Sets whether fast TTL settle is enabled, and set the trigger channel */
    virtual void setFastTTLSettle (bool state, int channel) = 0;

    /** Sets level of noise slicer on DAC channels */
    virtual int setNoiseSlicerLevel (int level) = 0;

    /** Turns LEDs on or off */
    virtual void enableBoardLeds (bool enabled) = 0;

    /** Sets divider on clock output; returns the actual divide ratio */
    virtual int setClockDivider (int divide_ratio) = 0;

    /** Connects a headstage channel to a DAC */
    virtual void connectHeadstageChannelToDAC (int headstageChannelIndex, int dacChannelIndex) = 0;

    /** Sets trigger threshold for DAC channel (if TTL output mode is enabled) */
    virtual void setDACTriggerThreshold (int dacChannelIndex, float threshold) = 0;

    /** Returns true if a headstage is enabled */
    virtual bool isHeadstageEnabled (int hsNum) const = 0;

    /** Sets the number of channels to use in a headstage */
    virtual void setNumHeadstageChannels (int headstageIndex, int channelCount) = 0;

    /** Returns the active number of channels in a headstage */
    virtual int getActiveChannelsInHeadstage (int hsNum) const = 0;

    /** Returns the total number of channels in a headstage */
    virtual int getChannelsInHeadstage (int hsNum) const = 0;

    /** Returns the total number of channels of a given type */
    virtual int getNumDataOutputs (ContinuousChannel::Type) = 0;

    virtual bool sendRecordOnCommand() { return false; }

    virtual bool sendRecordOffCommand() { return false; }

    virtual String getLastRecordingPath() const { return {}; }

    virtual String getLastRecordingCsvPath() const { return {}; }

    virtual void updateSampleFrequency (int newFreq) {}

    /** Enables or disables the hardware filter */
    virtual void setFilterEnabled (bool enabled) {}

    /** Sets the analog input gain/scaling factor */
    virtual void setAnalogInGain (float gain) {}

    /** Sets the analog output voltage */
    virtual void setAnalogOutVoltage (float voltage) {}

    /*
        Returns the total number of continuous channels the board produces.
        This adds up three channel types:
          - ELECTRODE: the neural recording channels (from headstages)
          - AUX:       auxiliary accelerometer channels (3 per headstage)
          - ADC:       external analog-to-digital converter channels
        The result tells Open Ephys how big to make the data buffer.
    */
    int getNumChannels()
    {
        return getNumDataOutputs (ContinuousChannel::ELECTRODE)
               + getNumDataOutputs (ContinuousChannel::AUX)
               + getNumDataOutputs (ContinuousChannel::ADC);
    }

    /*
        triggerDigitalOutput — fires a TTL pulse for a set duration.

        Two-step flow:
          Step 1 — RIGHT NOW: push a turn-ON command into the queue so the
                   run() thread sends the rising edge on the next iteration.
          Step 2 — LATER: create a DigitalOutputTimer that will fire after
                   eventDurationMs milliseconds and push a turn-OFF command,
                   creating the falling edge automatically.

        The timer is owned by digitalOutputTimers (an OwnedArray) which will
        delete it once it has fired. This avoids us having to track it manually.
    */
    void triggerDigitalOutput (int ttlLine, int eventDurationMs)
    {
        DigitalOutputCommand command;
        command.ttlLine = ttlLine;
        command.state = true;

        digitalOutputCommands.push (command); // Step 1: queue the ON edge now

        DigitalOutputTimer* timer = new DigitalOutputTimer (this, ttlLine, eventDurationMs); // Step 2: schedule OFF edge

        digitalOutputTimers.add (timer);
    }

    /** Pointer to device editor*/
    DeviceEditor* editor = nullptr;
    /** Creates buffers for custom streams if the acquisition board type has them */
    virtual void createCustomStreams (OwnedArray<DataBuffer>& otherBuffers) {};

    /** Create stream and channel structures is the acquisition board type has custom streams and updates the buffers */
    virtual void updateCustomStreams (OwnedArray<DataStream>& otherStreams, OwnedArray<ContinuousChannel>& otherChannels) {};

    DataBuffer* getBuffer()
    {
        buffer = new DataBuffer (getNumChannels(), 60000);
        return buffer;
    }

    /*
        BoardType — an enum that names each supported hardware variant.
        DeviceThread uses this to decide which concrete class to create
        and which UI columns to show in the DeviceEditor.

        None      = no board detected yet (default at startup)
        Simulated = fake board used for testing without real hardware
        OpalKelly = classic USB FPGA board (Intan RHD2000 era)
        ONI       = Open Neuro Interface — newer USB3 platform
        RedPitaya = our custom Red Pitaya / ESP32-S3 wireless IMU board
    */
    enum class BoardType
    {
        None = 0,
        Simulated = 1,   // software-only fake board for testing
        OpalKelly = 2,   // USB FPGA (legacy Intan boards)
        ONI = 3,         // Open Neuro Interface USB3 platform
        RedPitaya = 4    // Red Pitaya or ESP32-S3 STEP node (our wireless IMU board)
    };

    BoardType getBoardType() const { return boardType; }

protected:
    /*
        DigitalOutputTimer — a one-shot JUCE Timer that pushes the turn-OFF
        command after the caller-specified delay.

        Why use a timer instead of sleeping? Because sleeping the run() thread
        would freeze the data stream for eventDurationMs. Instead we let the
        JUCE message thread fire the callback asynchronously while run() keeps
        going. When the timer fires, it calls addDigitalOutputCommand() to
        queue the falling edge and then removes itself from digitalOutputTimers.
    */
    class DigitalOutputTimer : public Timer
    {
    public:
        /** Constructor — starts the one-shot countdown immediately */
        DigitalOutputTimer (AcquisitionBoard* board_, int tllLine, int eventDurationMs) : board (board_),
                                                                                          tllOutputLine (tllLine)
        {
            startTimer (eventDurationMs); // fires once after eventDurationMs ms
        }

        /** Destructor*/
        ~DigitalOutputTimer() {}

        /** Sends signal to turn off event channel*/
        void timerCallback()
        {
            stopTimer(); // prevent re-firing

            board->addDigitalOutputCommand (this, tllOutputLine, false); // queue the OFF edge
        }

    private:
        AcquisitionBoard* board;

        const int tllOutputLine;
    };

    void addDigitalOutputCommand (DigitalOutputTimer* timerToDelete, int ttlLine, bool state)
    {
        DigitalOutputCommand command;
        command.ttlLine = ttlLine;
        command.state = state;

        digitalOutputCommands.push (command);

        digitalOutputTimers.removeObject (timerToDelete); // this deletes the timer object
    }

    /*
        buffer — the shared memory ring buffer between the run() thread and Open Ephys.
        The run() thread continuously writes new samples into this buffer.
        Open Ephys reads from it on its own schedule to send data downstream.
        Sized at 60000 samples deep so a brief processing hiccup doesn't cause data loss.
    */
    DataBuffer* buffer;

    /*
        OptimumDelay — stores the automatically measured cable-delay calibration
        for each of the 8 headstage ports. Units are in 1/2-sample increments.
        Default -1 means "not yet measured."
    */
    struct OptimumDelay
    {
        float portA = -1;
        float portB = -1;
        float portC = -1;
        float portD = -1;
        float portE = -1;
        float portF = -1;
        float portG = -1;
        float portH = -1;
    };

    /*
        CableLength — the physical cable length (in metres) for each port.
        This feeds into the Intan chip's delay calculation so signals are
        sampled at exactly the right phase despite cable propagation delay.
        Default 0.914 m (about 3 feet) matches the standard Open Ephys cable.
    */
    struct CableLength
    {
        float portA = 0.914f;
        float portB = 0.914f;
        float portC = 0.914f;
        float portD = 0.914f;
        float portE = 0.914f;
        float portF = 0.914f;
        float portG = 0.914f;
        float portH = 0.914f;
    };

    /*
        Dsp — on-chip digital high-pass filter (offset removal).
        enabled=true removes slow DC drift from electrodes.
        cutoffFreq is in Hz; 0.5 Hz is a common choice for LFP recording.
    */
    struct Dsp
    {
        bool enabled = true;
        double cutoffFreq = 0.5;
    };

    /*
        AnalogFilter — the RHD chip's on-chip analog bandpass filter.
        upperBandwidth: low-pass cutoff in Hz (default 7500 Hz, limits high-freq noise)
        lowerBandwidth: high-pass cutoff in Hz (default 1 Hz, blocks slow drift)
        Together they define the "recording bandwidth" of each electrode channel.
    */
    struct AnalogFilter
    {
        double upperBandwidth = 7500.0f;
        double lowerBandwidth = 1.0f;
    };

    /*
        Settings — a single struct that bundles all board-wide configuration.
        Subclasses read from and write to `settings` to persist UI choices.

        Key fields:
          acquireAux / acquireAdc  — enable/disable AUX (accel) and ADC channel groups
          fastSettleEnabled        — rapidly bleed electrode charge after a large TTL pulse;
                                     prevents amplifier saturation artifacts
          fastTTLSettleEnabled     — same settle logic but triggered by a specific TTL input
          ttlOutputMode            — when true, DAC channels output TTL pulses (not audio)
          dsp                      — on-chip DSP high-pass filter settings (see Dsp above)
          analogFilter             — on-chip analog bandpass settings (see AnalogFilter above)
          boardSampleRate          — current sample rate in Hz (e.g. 30000.0 for 30 kHz)
          cableLength              — physical cable lengths per port (affects timing calibration)
          optimumDelay             — auto-calibrated port delays
          audioOutputL/R           — headstage channel index routed to left/right audio monitor
          ledsEnabled              — whether the board's indicator LEDs are on
          clockDivideFactor        — divide ratio for the clock output BNC connector
    */
    struct Settings
    {
        bool acquireAux = false;
        bool acquireAdc = false;

        bool fastSettleEnabled = false;
        bool fastTTLSettleEnabled = false;
        int fastSettleTTLChannel = -1;
        bool ttlOutputMode = false;

        Dsp dsp;
        AnalogFilter analogFilter;

        int noiseSlicerLevel;

        bool desiredDAChpfState;
        double desiredDAChpf;
        float boardSampleRate = 30000.f;

        CableLength cableLength;
        OptimumDelay optimumDelay;

        int audioOutputL = -1;
        int audioOutputR = -1;
        bool ledsEnabled = true;
        bool newScan = true;
        int numberingScheme = 1;
        uint16 clockDivideFactor;

    } settings;

    /** Impedance meter */
    std::unique_ptr<ImpedanceMeter> impedanceMeter;

    /** Impedance data*/
    Impedances impedances;

    /** Determines how channel names are created */
    ChannelNamingScheme channelNamingScheme = GLOBAL_INDEX;

    /** Queue of commands for setting digital output state */
    std::queue<DigitalOutputCommand> digitalOutputCommands;

    /** Array of timers for setting digital output state */
    OwnedArray<DigitalOutputTimer> digitalOutputTimers;

    /** True if change in settings is needed during acquisition*/
    bool updateSettingsDuringAcquisition = false;

    BoardType boardType = BoardType::None;
};

#endif

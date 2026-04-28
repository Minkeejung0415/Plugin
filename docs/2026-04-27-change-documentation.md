
# If red pitaya does not boot

1. check if you can access the [rp-f0f85a.local](http://rp-f0f85a.local/) page
2. check if you can ssh into the red pitaya
3. check if blue and green lights are on and red light is blinking
4. If the led lights are doing fine but you can't ssh into the red pitaya, check if the red pitaya is correctly wired to the router which is connected to the PC
5. if those are fine but you can't access the red pitaya, go to terminal and use "arp -a" command and check if you can see f0f85a or f0cd35 as one of the addresses
6. If you can find them but can't access both of them check if the router is connected to the power on the wall
7. If that is also good, just unplug everything and let is rest for a bit

 
## Changes in the last week of April

### `RedPitaya_justin.c`
new changes: save recordings to .bin and .csv

# Open Ephys Plugin Changes

- Added fixed channel layout support for raw sensor data, VQF/filter channels, and analog waveform inputs. ex) sensor = 1 : 9 (raw) + 4 (filter) + 2 (analog input)
- Added filter/VQF toggling during acquisition. 
- Added two analog waveform input channels.
- Added UI controls for recording, filtering, sample rate, analog input gain, and analog output voltage.

![[Screenshot 2026-04-27 142836 1.png]]


## VQF / Filter Feature

*The `FILTER` button controls whether VQF/filter values are written into the reserved VQF channels.*

*When filter is on:*

- *VQF channels are filled with calculated values.*

*When filter is off:*

- *VQF channels are still present;*
- *VQF channels are filled with zeros.*

*The filter can be changed while data is being collected. The Red Pitaya server checks for filter commands before building the next frame, so the next frame uses the latest filter state.*

## *Analog Output Voltage Control*

*The UI has an editable `Analog Out (V)` field.*

*Valid range:*

```
0.0 to 1.8 V
```

*When changed, Open Ephys sends:*

```
AOUT:<value>
```

*The Red Pitaya server currently receives and logs this command.*

*Analog output is a command, not a measured waveform channel. If the physical output voltage needs to be displayed, it must be measured back through an input channel.*

# What more needs to be done

- test analog input and output with ADC 
- fix the Makefile issue where when the new code file uses the vqf.c or sensor_fusion.c it has to be manually implemented in the makefile and not automatically dealt with
- the double start issue for opne ephys it requires me to start and stop and restart the acquisition for it to start the acquisition properly

# DeWaltBatteryCutoff
This circuit is purely experimental and there has not been very much testing on it - at all.

PCB using the RP2040 Pico microcontroller to monitor a 20V DeWalt battery and give an alarm and eventually shutoff the battery load to avoid deep discharging.

KiCAD PCB Design
I used the pico microcontroller KiCAD model from https://github.com/ncarandini/KiCad-RP-Pico, which is really nicely done!

![Pico_DeWaltBattMonitor](https://user-images.githubusercontent.com/5246863/206541472-f4480458-15e5-416d-8e03-21f5f08cb92e.jpg)
The above screen capture was taken before some additional minor adjustments on the final board.

The circuit uses a RP2040 Pico microcontroller to monitor the battery voltage from a buffered voltage divider on an ADC pin.  When the battery reaches a set value (can be stored in EEPROM) a buzzer alarm begins to sound, getting more rapid as the battery voltage approaches the low voltage cutoff value.  Once that value is surpassed (with hysteresis) the output voltage is cutoff using a P-Channel MOSFET (currently using a 17 amp version), and the buzzer becomes constant.  Once the voltage reaches the vary lowest discharge voltage for 5 serias 18650 LiPo batteries of about 3.12 volts / cell using 3V as a minimum resting voltage.  When this happens the buzzer completely stops and the circuit goes into the lowest current draw setting (as no one has disconnected it in time).

The control program was written using the Arduino IDE with the Raspberry Pi Pico/RP2040 board library.  The user can set and save specific parameters in the EEPROM such as voltage, gain (resistor divider ratio), and a brief description (string).


[DeWalt_batteryCutoff_schematic.pdf](https://github.com/jebradshaw/DeWaltBatteryCutoff/files/10188782/DeWalt_batteryCutoff_schematic.pdf)

![DeWalt_BattCutoff_pcb_picture](https://user-images.githubusercontent.com/5246863/206547494-ebd19af9-fb84-45d4-b6cc-bfe5c06f5abb.PNG)
Got 3 prototype PCB's back from OSH Park.  I made a mistak with the SOT-23 NPN transistor pinout (base and emmiter switched).

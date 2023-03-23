# DeWaltBatteryCutoff
This circuit is purely experimental and there has not been very much testing on it - at all.

PCB using the RP2040 Pico microcontroller to monitor a 20V DeWalt battery and give an alarm and eventually shutoff the battery load to avoid deep discharging.

KiCAD PCB Design
I used the pico microcontroller KiCAD model from https://github.com/ncarandini/KiCad-RP-Pico, which is really nicely done!

![Pico_DeWaltBattMonitor](![rendered_3D_Image_KiCad2](https://user-images.githubusercontent.com/5246863/227246800-28ecda69-5e41-4fba-b494-e9c93554481c.JPG)
)

The circuit uses a RP2040 Pico microcontroller to monitor the battery voltage from a buffered voltage divider on an ADC pin.  The user must turn on the board with an external temporary pushbutton and power MOSFET turns on supplying voltage to the regulator that turns on the pico microcontroller.  The Pico then latches the MOSFET on and begins to run the voltage monitoring program. When the battery reaches a set value (can be stored in EEPROM) a buzzer alarm begins to sound, getting more rapid as the battery voltage approaches the low voltage cutoff value.  Once that value is surpassed (with hysteresis) a 10 second timer begins and the output voltage is cutoff using a P-Channel MOSFET (currently using a 27 amp version FRP27P06) where the pico turns itself off.  Data is transmitted over the USB port at 115200 baud and eeprom values can be set and saved.  Output format is a serial ASCII string with the following format:

NAME, TIME, rate of discharge, battery voltage, Buzzer critical state, high voltage alarm, low voltage alarm

"PICOBATMON,T:3412691,dVdT:0.000,Bv:20.24,Bcrit:0,Halarm:17.50,Lalarm:17.00\r\n"


The control program was written using the Arduino IDE with the Raspberry Pi Pico/RP2040 board library.  The user can set and save specific parameters in the EEPROM such as voltage, gain (resistor divider ratio), and a brief description (string).


[DeWalt_batteryCutoff_schematic.pdf]([PicoDeWalt_BattMonV2 Schematic.pdf](https://github.com/jebradshaw/DeWaltBatteryCutoff/files/11052444/PicoDeWalt_BattMonV2.Schematic.pdf)
)

![DeWalt_BattCutoff_pcb_picture](![20230316_080624_resized](https://user-images.githubusercontent.com/5246863/227247175-fd1eef53-71ab-49b6-8f50-81a639970574.jpg)
)
These printed circuit boards were made using Digikeys DK Red service and I am very happy with the results.

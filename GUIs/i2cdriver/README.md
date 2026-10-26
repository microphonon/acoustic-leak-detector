# GUI for the i2cdriver

The `aquaping_gui.py` code can be modified to work with the Excamera [i2cdriver](https://www.crowdsupply.com/excamera/i2cdriver) hardware. As with the RP Pico and Arduino Nano 33 IOT microcontrollers, the i2cdriver can supply 3V3 to the AquaPing provided a decoupling capacitor is in place. It does not, however, have provision for interrupts. Connect the corresponding SDA and SCL pins with jumper wires.

No modifications to the i2cdriver or AquaPing firmware are required. Run the file `i2cdriver_gui.py` as a batch file or from the command line and follow the guidance in [GUI_instructions.pdf](https://github.com/microphonon/acoustic-leak-detector/blob/main/GUIs/GUI_instructions.pdf). This is beta software; please report any bugs.

# GUI for the I2CDriver

The `aquaping_gui.py` code can be modified to work with the Excamera [i2cdriver](https://www.crowdsupply.com/excamera/i2cdriver) hardware. As with the RP Pico and Arduino Nano 33 IOT microcontrollers, the I2CDriver can supply 3V3 to the AquaPing provided a decoupling capacitor is in place. It does not, however, have provision for leak and noise interrupts. Connect the corresponding SDA and SCL pins with jumper wires. The I2CDriver connects to the PC with a USB Micro-B cable.

In addition to python and pyserial, the I2CDriver package should be installed on the host PC with `pip install i2cdriver`. **No modifications to the I2CDriver or AquaPing firmware are required**. Run the file `i2cdriver_gui.py` as a batch file or launch from the command line. Commands sent from the GUI will be displayed as hexadecimal bytes on the terminal. 

Additional guidance can be found in [GUI_instructions.pdf](https://github.com/microphonon/acoustic-leak-detector/blob/main/GUIs/GUI_instructions.pdf). This is beta software that is specific to the I2CDriver and will NOT work with the Arduino or Raspberry PI Pico. It was generated with the assistance of claud.ai. Please report any bugs.

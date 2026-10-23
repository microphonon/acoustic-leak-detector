# Text-only Terminal Programs

This directory provides text-only terminal interfaces for two-way communication between the AquaPing acoustic leak sensor and a personal computer. There is firmware for three specific hardware types:

- Arduino Nano 33 IOT
- Raspberry Pi Pico
- Lolin D1-mini WiFi module including firmware to upload data over WiFi to the ThingSpeak server

A USB Micro-B cable connects the PC to the I2C master hardware at a baud rate of 9600. Jumper wires link the master to corresponding I2C ports on the AquaPing. The free Arduino IDE is used to flash the .ino firmware onto the device via USB serial. A UART serial terminal program like putty or minicom can be used to monitor sensor operation and send it a variety of commands. The user must specify the proper serial port. 

Note that this firmware is NOT compatible with the graphical user interface also found in this repository. Consult the Terminal_instructions.pdf file for thorough guidance on installation and operation.

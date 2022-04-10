# Firmware example for ESP8266

Sample code to periodically read/write bytes from/to the AquaPing slave on the I2C bus. 
    Developed and tested with a Lolin D1 mini WiFi module as master, but should adapt to 
    other devices using the Arduino IDE and equivalent libraries. The D1 mini uses the Espressif ESP8266 for WiFi capability, 
    although wireless is not implemented here.
    
 The D1 mini module provides a UART to USB interface to allow communication with a PC. 
    Any serial terminal program (eg. putty) should work; be sure to set equivalent baud rates. 
    The master can monitor the alarm state of the slave with a separate INTERRUPT_PIN (lowest latency) 
    as well as reading the first byte of the status_array. 
    
 Command characters are read from the serial terminal as user input. Characters are echoed to 
    the terminal then formatted into command strings as a sequence of hex bytes that are sent to slave. 
    Syntax is in separate documentation.

 The sensor status is polled at a period set by polling_interval in milliseconds. The retrieved 
    status bytes are then sent to the terminal. Each I2C read operation is signaled by the D1 mini LED blinking. 
    To allow the user to send sensor configuration commands with reasonably low latency, the terminal 
    is checked for input with a period set by sleep_delay. 
    
It is important to understand that the rate at which
    the master polls the slave (i.e. sensor) is not the same rate that the sensor polls the environment. These polling 
    loops run asynchronously. The slave MCU prevents I2C bus conflicts by moving data via Direct Memory Access, which requires no
    CPU intervention. 

The D1 mini can supply 3V3 external power to the sensor.
    Make sure a large electrolytic filter capacitor is on this line.
    Don't forget the pullup resistors on SDA and SCL; tested with 4.7k.
  


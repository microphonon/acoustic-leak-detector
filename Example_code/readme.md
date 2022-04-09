# Sample master firmware

Example sketch illustrating how a master can periodically read/write bytes from/to the AquaPing slave on the I2C bus. 
    Developed and tested with a Lolin D1 mini WiFi module as master, but should adapt to 
    other devices using the Arduino IDE and equivalent libraries. The D1 mini WiFi capability 
    is not implemented here.
    
The D1 mini module provides a UART to USB interface to allow communication with a PC. 
    Any serial terminal program (eg. putty) should work; be sure to set equivalent baud rates. 
    The master can monitor the alarm state of the slave with a separate INTERRUPT_PIN (lowest latency) 
    as well as the first byte of the status_array. The D1 mini can supply 3V3 external power to the sensor.
    Make sure a large electrolytic filter capacitor is on this line.
    
Command characters are read from the serial terminal as user input. Characters are echoed to 
    the terminal then formatted into command strings as a sequence of hex bytes that are sent to slave. 
    Syntax is in separate documentation.
    
The sensor status is polled at a period set by polling_interval in milliseconds. The retrieved 
    status bytes are then sent to the terminal. Each I2C read is signaled by the LED blinking. 
    To allow the user to send sensor configuration commands with reasonably low latency, the terminal 
    is checked for input with a period set by sleep_delay.
    
Don't forget the pullup resistors on SDA and SCL; tested with 4.7k.

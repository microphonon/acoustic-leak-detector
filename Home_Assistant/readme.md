# AquaPing demo with Home Assistant

![AquaPing Home Assistant Demo](https://github.com/microphonon/acoustic-leak-detector/blob/main/Home_Assistant/screenshot.jpg?raw=true)

This repository demonstrates how the [AquaPing](https://www.crowdsupply.com/microphonon/aquaping) acoustic leak detector can be interfaced to Home Assistant with [ESPHome](https://esphome.io/). It uses the ESP8266 WiFi module on an inexpensive [D1-mini](https://www.wemos.cc/en/latest/d1/d1_mini.html) breakout board. Although WiFi is not the preferred wireless protocol for ultralow power IoT applications, this Home Assistant application is useful for illustrating the sensor's principles of operation and its capabilities. Real-time data is displayed on a custom HA UI frontend that is also used to configure the sensor monitoring parameters.

The AquaPing is designed to listen for very weak, high frequency acoustics that may accompany a pressurized plumbing leak. The frontend screenshot shown above depicts sensor operation with music playing nearby. This is impairing the sensor's ability hear potential leak signals. An appreciable number of sample counts are being interpreted as noise causing the corresponding alert to trigger (On). Noise discrimination and rejection are important for preventing false leak alarms.

Development was done running the Home Assistant operating system inside VirtualBox on Ubuntu Linux. [ESPHome](https://esphome.io/) is installed as a native Home Assistant Add-on. The *aquaping.h* header file must be present in the config/esphome/ directory. It can be configured with the HA File Editor Add-on and is an *include* in the custom ESPHome yaml file that is found in this repository. The optional horizontal bar graphs require the [bar-card](https://github.com/custom-cards/bar-card) and [config-template-card](https://github.com/iantrich/config-template-card) modules that were installed using [Home Assistant Community Store](https://hacs.xyz/) (a github account is required). There are plenty of good online tutorials that can guide you through the various install processes.



## D1-mini hardware

![D1-mini hardware](https://github.com/microphonon/acoustic-leak-detector/blob/main/Home_Assistant/breadboard.jpg?raw=true)

The D1-mini is an inexpensive ESP8266 WiFi breakout board that is available from a variety of vendors. It communicates with the AquaPing using the I2C interface. SCL connects to pin D1 and SDA to pin D2 on the D1-mini. Pullup resistors are already present on the breakout board. The D1-mini can supply 3V3 DC power to the AquaPing *provided* the supply rail is sufficiently quiet. In the photo above, a 220 uF electrolytic decoupling capacitor is placed across 3V3 and ground on the breadboard. The NOISE and ALARM pins of the AquaPing are connected to D3 and D4, respectively. These lines provide an immediate GPIO interrupt if a suspected leak or persistent noise are detected. Other GPIO pins may be used if the configuration is appropriately modified.

Because the D1-mini operates as an always-on WiFi radio, its power consumption will be orders of magnitude greater than the AquaPing sensor. In this demo, power is supplied through the USB connection on the breakout board. If the USB connection is made to a host computer, a serial terminal program (eg. puTTY) can also be used to communicate with the AquaPing. Default baud rate is 9600; adjust as needed in the *aquaping.h* file. More information about the serial protocol can be found in the AquaPing user guide located in this repository.

Another option that could allow extended battery operation is to place the D1-mini in deep sleep, wake it up at periodic intervals, and let it re-connect to the WiFi router. The longer the period, the greater the battery life, but with increased sensor latency. This would require modifications to the D1-mini firmware.

## D1-mini firmware

C++ code to control operation of the D1-mini is in the header file *aquaping.h*. It uses the common Arduino abstractions and must be declared as an include in the yaml file. It is important to understand that this demo implements two independent, asynchronous polling loops running on the D1-mini and AquaPing:

1) **Polling the sensor.** Sensor information is downloaded to and uploaded from Home Assistant over WiFi with a period set by the 'update_interval' (ms) in the D1-mini firmware; default is 10000 ms, i.e. 10 seconds. 

2) **Periodic analysis of the environmental acoustics.** The AquaPing spends most of the time in a sleep state, but periodically wakes up to sample the acoustic environment. This interval is set by selecting one of 9 fixed periods with duration in the range 1--30 seconds; see the user manual. 

There are many examples of sensors uploading data via I2C and displaying it on the Home Assistant dashboard. Going in the other direction, however, is not well documented. This capability is needed to send configuration parameters to the AquaPing.  In this demo, frontend data is passed through the ESPHome yaml to the firmware where it is sent as byte pairs using the *Wire* library. 

## Dashboard

The Home Assistant dashboard is highly configurable. Three dashboard cards are used here for alerts, streaming data, and control. Interrupts for potential leaks and anomalous noise are immediate and displayed on the alert card. The dashboard card with the data counts is updated only on each WiFi polling cycle, which introduces some latency. Similarly, adjustments to the AquaPing control panel are transmitted to the sensor at the same rate and latency. 

The alarm_trigger_count must be set to less than or equal to the event_set_size. If set higher, it will be coerced to the event_set_size. The control panel selects the polling period with an integer index (1--9) and the corresponding time is displayed in seconds. If the training_set_size is changed, a new training session will be automatically initiated. All configuration parameters are explained in the user guide. A switch is provided to enable/disable the AquaPing LEDs.

The horizontal bar graphs are optional, quite tricky to setup, and require installation of the [bar-card](https://github.com/custom-cards/bar-card) and [config-template-card](https://github.com/iantrich/config-template-card) modules. The latter is needed to re-scale the graph maxima when the user changes the event_set_size. See the included dashboard file for the syntax. There may be alternative techniques that could work better to render these graphics. Real-time counts can also be displayed using simpler numeric indicators. These can be added to the dashboard by editing the ESPHome yaml file.

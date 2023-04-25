# AquaPing demo with Home Assistant

![AquaPing Home Assistant Demo](https://github.com/microphonon/acoustic-leak-detector/blob/main/Home_Assistant/screenshot.jpg?raw=true)

The above screenshot shows the AquaPing operating with music playing nearby. An appreciable number of sample counts are being interpreted as noise causing the corresponding alert to trigger (On). Noise rejection is important for preventing false leak alarms.

Development was done running the Home Assistant operating system inside VirtualBox on Ubuntu Linux. [ESPHome](https://esphome.io/) is installed as a native Home Assistant Add-on. The *aquaping.h* header file must be present in the config/esphome/ directory. It can be configured with the File Editor Add-on and is called by the custom ESPHome yaml file that is also included in this repository. The optional horizontal bar graphs require the [bar-card](https://github.com/custom-cards/bar-card) and [config-template-card](https://github.com/iantrich/config-template-card) modules that were installed using [Home Assistant Community Store](https://hacs.xyz/). There are plenty of good online tutorials that can guide you through the various install processes.

## D1-mini hardware

The D1-mini is an inexpensive ESP8266 WiFi breakout board that is available from a variety of vendors. It communicates with the AquaPing using the I2C interface. SCL connects to pin D1 and SDA to pin D2. Pullup resistors are already present on the breakout board. The D1-mini can supply 3V3 DC power to the AquaPing *provided* the supply rail is sufficiently quiet. In the photo below, a 220 uF electrolytic decoupling capacitor is placed across 3V3 and ground on the breadboard. The NOISE and ALARM pins of the AquaPing are connected to D3 and D4, respectvely. These lines provide an immediate GPIO interrupt if a suspected leak or persistent noise are detected. Other GPIO pins may be used if the configuration is appropriately modified.

Because the D1-mini operates as an always-on WiFi, its power consumption will be orders of magnitude greater than the AquaPing sensor. In this demo, power is supplied through the USB connection on the breakout board. If the USB connection is made to a host computer, a serial terminal program (eg. puTTY) can be used to communicate with the AquaPing. Default baud rate is 9600; adjust as needed in the *aquaping.h* file. More information about the serial protocol can be found in the AquaPing user guide located in this repository.

Another option that could allow extended battery operation is to place the D1-mini in deep sleep, wake it up at periodic intervals, and let it re-connect to the WiFi router. The longer the period, the greater the battery life, but with increased sensor latency. This would require modifications to the D1-mini firmware.

## D1-mini firmware

C++ code to control operation of the D1-mini is in the header file *aquaping.h*. It uses the common Arduino abstractions. It is important to understand that this demo implements two independent, asynchronous polling loops: 1) Sensor information is downloaded to and uploaded from Home Assistant over WiFi with a period set by the 'update_interval' (ms); default is 10000 ms, i.e. 10 seconds. 2) The AquaPing spends most of the time in a sleep state, but periodically wakes up to sample the acoustic environment. This interval is set by choosing one of 9 fixed periods of duration in the range 1--30 seconds; see the user manual.

## Dashboard

The Home Assistant dashboard is highly configurable. In this demo, three dashboard cards are used. Interrupts are immediate and displayed on the alert card. The dashboard card with the streaming data counts is updated only on each WiFi polling cycle, which introduces some latency. Similarly, adjustments to the AquaPing control panel are transmitted to the sensor at the same rate.

The horizontal bar graphs are optional, quite tricky to setup, and require installation of the bar-card and config-template-card modules. The latter is needed to re-scale the graph maxima when the user changes the event_set_size. It is declared as a dashboard global variable using config_template_card_vars. See the included dashboard file for the syntax. There may be alternative techniques that could work better to render these graphics. Real-time counts can also be displayed using simpler numeric indicators. These can be added to the dashboard by editing the ESPHome yaml file.

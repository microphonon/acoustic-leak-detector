# Home Assistant Demo

Development was done running Home Assistant inside VirtualBox on Ubuntu Linux. [ESPHome](https://esphome.io/) is installed as a native Home Assistant add-on. The aquaping.h header file must be present in the config/esphome/ directory. It can be configured with the File Editor add-on and is called by the custom ESPHome yaml file that is also included in this repository. The horizontal bar charts require the [bar-card](https://github.com/custom-cards/bar-card) and [config-template-card](https://github.com/iantrich/config-template-card) modules that were installed using [Home Assistant Community Store](https://hacs.xyz/). There are plenty of good online tutorials that can guide you through the various install processes.

## D1-mini hardware

The D1-mini is an inexpensive ESP8266 WiFi breakout board that is and available from a variety of vendors. It communicates with the AquaPing using the I2C interface. SCL connects to pin D1 and SDA to pin D2. Pullup resistors are already on the breakout board. The D1 mini can supply 3V3 DC power to the AquaPing *provided* the supply rail is sufficiently quiet. In the photo below, a 220 uF electrolytic decoupling capacitor is placed across 3V3 and ground on the breadboard. The NOISE and ALARM pins of the AquaPing are connected to D3 and D4, respectvely. These lines provide an immediate GPIO interrupt if a suspected leak or persistent noise are detected. Other GPIO pins may be used

Because the D1 mini operates as an always-on WiFi, its power consumption will be orders of magnitude greater than the AquaPing sensor. In this demo, power is supplied through the USB connection on the breakout board. If the USB connection is made to the host computer, a serial terminal program (eg. puTTY) can be used to communicate with the AquaPing. More information about the UART protocol can be found in the AquaPing user guide located in this repository.

## D1-mini firmware



# Home Assistant Demo

Development was done running Home Assistant inside VirtualBox on Ubuntu Linux. [ESPHome](https://esphome.io/) is a required native Home Assistant add-on. The aquaping.h header file must be present in the config/esphome/ directory. It can be configured with the File Editor add-on and is called by the custom ESPHome yaml file that is also included in this repository. The horizontal bar charts require the [bar-card](https://github.com/custom-cards/bar-card) and [config-template-card](https://github.com/iantrich/config-template-card) modules that were installed using [Home Assistant Community Store](https://hacs.xyz/). There are plenty of good online tutorials that can guide you through the various install processes.

## D1-mini hardware

The D1-mini is an inexpensive ESP8266 breakout board that is and available from a variety of vendors. It communicates with the AquaPing using the I2C interface. SCL connects to pin D1 and SDA to pin D2. Pullup resistors are already on the breakout board. The D1 mini can supply 3V3 DC power to the AquaPing *provided* the supply rail is sufficiently quiet. In the photo below, a 220 uF electrolytic decoupling capacitor is placed across 3V3 and ground on the breadboard. The NOISE and ALARM pins are connected to D3 and D4, respectvely.

## D1-mini firmware



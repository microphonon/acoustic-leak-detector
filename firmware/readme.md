# Modifying and compiling the code
The firmware was developed using Code Composer Studio 9.3.0 on an Ubuntu-Linux platform. CCS is a free download from Texas Instruments and provides an Eclipse style interface. The MSP430FR5994 MCU on the PCB is flashed using the Spy-Bi-Wire two-terminal interface marked RESET and TEST on the leak sensor PCB. Almost any variety of inexpensive TI Launchpads can be used for this purpose. The Launchpads have a debugger and an Energy Trace function that is useful for identifying code inefficiencies and estimating battery life. Alternatively, compiled binaries can be flashed with the MSP430-BSL made by Olimex and available at [Mouser](https://www.mouser.com/ProductDetail/Olimex-Ltd/MSP430-BSL?qs=J7x7253A5u5ktaUHQ83VeQ%3D%3D).

A capability for over-the-air firmware updates is an anticipated future upgrade. This can be accomplished using the I2C terminals SDA and SCL in addition to the RESET and TEST pins. The required MCU part number is MSP430FR59941 as distinguished from MSP430FR5994 that uses a UART connection with the master. A UART design must include TXD and RXD pins, increasing the header pin count from 7 to 9. If OTA upgrades are not necessary, either variety of MCU will work.

The Texas Instruments [MSP DSP Library](https://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/DSPLib/1_30_00_02/exports/html/index.html) must be installed on the development platform, also a free download.  Use your file explorer to copy the dsplib directory directly into the CCS project folder. Enable DSP by navigating to:

 CCS Build: MSP430 Compiler: Include Options

and adding the following line:

 ${PROJECT_ROOT}/dsplib/include
 

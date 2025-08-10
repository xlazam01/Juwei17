This is repository which contains subproject to upgrade Juwei17 mount

![Juwei17 Upgrade](Images/main.JPG)

Upgrade Features
- Larger on/off switch
- Homing
- GPS
- Weather sensors
- Fast slewing possible
- Support for TMC SPI or UART modules - ie. TMC2209, TMC2130 or TMC5160..
- USB-C connector
- Programming button - no ST-Link needed
- Loader application - semiautomatic programing for OnStep and SWS

HW Features
- minimized PCB size
- added support for Home and Limit switches
- OnWire interface
- GPS interface 
- STM32F446 as main processor
- Power switch is not needed
    - No high currect goes through switch itself
    - When switch is closed - mount is off

SW Features
- Based on OnStepX - preconfiguredm just upload
- added STM32 RTC support
- Modified Weather
    - due to excesive BME self heating
    - temperature from DS1820
    - humidity and pressure from BME280
- TLS fallback (STM32 -> GPS)

Mechanics
- CAD files
- STL for 3D print new covers

What you will need
- New hardware
- Beitian BK-280 GPS module
- BME280 module
- 2pcs 3144 hall sensors
- 16mm antivandal switch
- wires, connectors, hotglue...
- magnetic rings
    - Magnets 20x5x2mm - Warning: I experienced large deviation in sizes
    - Screws 2x8, 2x10 - Voron style self tapping screws

Building Tips
- use hotglue to fix hall sensors and wires
- mind the hall sensor "polarity" and magnets orientation in rings

Folder description
- CAD - contains models (SolidEdge *.par files)
- HW - contains KiCad files with design and schematic PDF
- Images - Few images from build
- OnstepEspLoader - Python app to support you through flashing process
- OnStepX - OnStep code (PlatfromIO and VSCode)
- SmartWebServer - ESP8266 code (Build in Arduino due to specific Espressif version used)
- STLs - files for 3D print 

ToDo
- PEC

Pictures

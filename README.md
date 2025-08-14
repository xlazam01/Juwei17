<div align="center">
  <h1>Juwei17 Mount Upgrade</h1>
  <img src="./Images/main.JPEG" alt="Juwei17 Upgrade" width="60%" />
</div>


## 🚀 Project Overview

This repository contains subprojects to upgrade the Juwei17 mount with enhanced hardware, software, and mechanics. The goal is to provide a modern, feature-rich, and user-friendly experience for astronomy enthusiasts.

---

## ✨ Upgrade Features

- **Larger on/off switch**
- **Homing**
- **GPS**
- **Weather sensors**
- **Fast slewing possible**
- **Support for TMC SPI or UART modules**  
  (TMC2209, TMC2130, TMC5160, etc.)
- **USB-C connector**
- **Programming button** (no ST-Link needed)
- **Loader application** (semi-automatic programming for OnStep and SWS)

---

## 🛠️ Hardware Features

- Minimized PCB size
- Support for Home and Limit switches
- OnWire interface
- GPS interface
- STM32F446 as main processor
- Power switch not needed:
  - No high current goes through switch itself
  - When switch is closed, mount is off

---

## 💾 Software Features

- Based on OnStepX (preconfigured, just upload)
- Added STM32 RTC support
- Modified Weather:
  - Due to excessive BME self-heating
  - Temperature from DS1820
  - Humidity and pressure from BME280
- TLS fallback (STM32 → GPS)

---

## 🏗️ Mechanics

- CAD files
- STL files for 3D printing new covers

---

## 🧰 What You Will Need

- New hardware
- Beitian BK-280 GPS module
- BME280 module
- 2× 3144 hall sensors
- 16mm antivandal switch
- Wires, connectors, hot glue, etc.
- Magnetic rings:
  - Magnets 20×5×2mm  
    ⚠️ *Warning: Large deviation in sizes experienced!*
  - Screws 2×8, 2×10 (Voron style self-tapping screws)
- optional 3144 hall sensor module for test 

---

## 🏗️ Building Tips

- Use hot glue to fix hall sensors and wires
- Mind the hall sensor "polarity" and magnets orientation in rings

---

## 📁 Folder Description

- **CAD**: SolidEdge models (`*.par` files)
- **HW**: KiCad files (design and schematic PDF)
- **Images**: Build images
- **OnstepEspLoader**: Python app to support you through flashing process
- **OnStepX**: OnStep code (PlatformIO and VSCode)
- **SmartWebServer**: ESP8266 code (built in Arduino due to specific Espressif version)
- **STLs**: Files for 3D printing

---

## 📝 ToDo

- PEC

---

## 🖼️ Photo Gallery

<div align="center">
  <img src="./Images/main.JPEG" alt="Juwei17 Upgrade" width="60%" />
</div>

<div align="center">
  <img src="./Images/IMG_3873.JPEG" alt="IMG_3873" width="40%" />
  <img src="./Images/IMG_3874.JPEG" alt="IMG_3874" width="40%" />
</div>
<div align="center">
  <img src="./Images/IMG_3877.JPEG" alt="IMG_3877" width="40%" />
  <img src="./Images/IMG_3878.JPEG" alt="IMG_3878" width="40%" />
</div>
<div align="center">
  <img src="./Images/IMG_4165.JPEG" alt="IMG_4165" width="40%" />
  <img src="./Images/IMG_4166.JPEG" alt="IMG_4166" width="40%" />
</div>
<div align="center">
  <img src="./Images/IMG_4168.JPEG" alt="IMG_4168" width="40%" />
  <img src="./Images/IMG_4169.JPEG" alt="IMG_4169" width="40%" />
</div>
<div align="center">
  <img src="./Images/IMG_4171.JPEG" alt="IMG_4171" width="40%" />
  <img src="./Images/IMG_4172.JPEG" alt="IMG_4172" width="40%" />
</div>
<div align="center">
  <img src="./Images/IMG_4173.JPEG" alt="IMG_4173" width="40%" />
  <img src="./Images/IMG_4174.JPEG" alt="IMG_4174" width="40%" />
</div>


Folder description
- CAD - contains models (SolidEdge *.par files)
- HW - contains KiCad files with design and schematic PDF
- Images - Few images from build
- OnstepEspLoader - Python app to support you through flashing process
- OnStepX - OnStep code (PlatfromIO and VSCode)
- SmartWebServer - ESP8266 code (Build in Arduino due to specific Espressif version used)
- STLs - files for 3D print 

## 🛠️ Assembly Instructions

1. **Print the required parts.**
2. **Prepare magnets for the magnet rings:**
    - Ensure all magnets are oriented in the same direction.
    - Take care to prevent magnets from snapping together unexpectedly.
    - Test magnet polarity using a 3144 module or a bare sensor.
3. **Insert magnets into the rings.**
    - Fill roughly 90-180degrees
4. **Secure magnets with a thin layer of epoxy.**
    - 5min epoxy is ideal
5. **Assemble the front cover using small screws.**
    - Put together PCB_holder and front_panel
6. **Install hall sensors and wiring into the new cover.**
    - Insert hall sensor with right orientation (double check)
    - Fix them in place with hotglue
7. **Mount the new 16mm switch and do wiring.**
    - We use NO contact for mount ON, NC contact for mount OFF
    - Switch backlight is connected to maiboard
8. **Prepare and install wiring for the GPS and BME280 modules.**
    - Solder longer wires and cut them later to perfect lenght
    - Install modules in cavities
    - Fix GPS with GPS_holder and small screws
9. **Install the DS1820 sensor in the second cover.**
    - Pressfit the sensor in place
    - Cut wires to size
10. **Secure all wires with hot glue.**
    - Aim is to not have wire getting pinched during cover installation
11. **Trim wires to length and crimp connectors.**
12. **Connect all components together.**
    - Install back cover
    - Plug in all connectors
    - Install front cover
13. **Assembly complete!**



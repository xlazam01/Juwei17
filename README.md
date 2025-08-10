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

## 🖼️ Pictures
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

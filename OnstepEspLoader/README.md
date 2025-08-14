# OnstepEspLoader

This is a Python application designed to upload
- firmware.bin binary file to STM32 over serial bootloader - use #serialScript.py for this
- SmartWebServer.ino.bin binary file to ESP8266

## Features
- Check presence of STM32 bootloader by get part ID
- Flash STM32 in serial bootloader
- Check presence of OnStepX
- Switch STM32 to transparent serial port by sending :hF#:ESPFLASH# to OnStep at 9600 baud to enable SWS upload mode.
- Flash ESP with SmartWebServer

## Requirements
- Python 3.7+
- pip

## Setup
1. Install dependencies:
   ```sh
   pip install stm32loader
   pip install pyserial
   ```
2. Connect your STM32/ESP32 device via USB.
3. Run the application:
   ```sh
   python main.py
   ```

## Usage
- Follow the buttons - they are self explanatory
- The script will handle the upload process.

## License
MIT

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

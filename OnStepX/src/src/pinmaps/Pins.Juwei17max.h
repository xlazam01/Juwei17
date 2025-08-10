// -------------------------------------------------------------------------------------------------
// Pin map for FYSETC S6
#pragma once

#if defined(STM32F446xx)

// TX2/RX2 (PA2/PA3) is on the Y+ and Z+ end stops and is reserved for GPS (etc, no command channel is associated with this port)

// Schematic isn't entirely clear, best guess:
// Serial1 RX1 Pin PA10, TX1 Pin PA9
// Serial2 RX2 Pin PA3, TX2 Pin PA2
// Serial3 RX3 Pin PC11, TX3 Pin PC10
// Serial6 RX6 Pin PC7, TX6 Pin PC6

#if SERIAL_A_BAUD_DEFAULT != OFF
 #define SERIAL_A              Serial1
 // #define SERIAL_A              HardSerial
 // #define SERIAL_A_RX           PA10
 // #define SERIAL_A_TX           PA9
#endif

#if SERIAL_B_BAUD_DEFAULT != OFF
  #define SERIAL_B              HardSerial
  #define SERIAL_B_RX           PA3
  #define SERIAL_B_TX           PA2
#endif

#if SERIAL_GPS_BAUD != OFF
  #ifndef SERIAL_GPS
    #define SERIAL_GPS              HardSerial
    #define SERIAL_GPS_RX           PC7
    #define SERIAL_GPS_TX           PC6
  #endif
#endif

#if defined(STEP_DIR_TMC_UART_PRESENT) || defined(SERVO_TMC2209_PRESENT)
  #define SERIAL_TMC_HARDWARE_UART
  #define SERIAL_TMC            Serial3          // Use a single hardware serial port to up to four drivers
  #define SERIAL_TMC_BAUD       115200           // Baud rate
  #define SERIAL_TMC_RX         PC5               // Recieving data
  #define SERIAL_TMC_TX         PB10               // Transmit data
  #define SERIAL_TMC_ADDRESS_MAP(x)   (x) // Axis1(0) is 0, Axis2(1) is 1
  #define SERIAL_TMC_RX_DISABLE false
#endif

#define PIN_INIT() { \
  HAL_WIRE.setSDA(PB7);\
  HAL_WIRE.setSCL(PB6);\
}

  //pinMode(PB0, OUTPUT);\
  //digitalWrite(PB0, HIGH);\

// The multi-purpose pins (Aux3..Aux8 can be analog pwm/dac if supported)
// I defined 7 Aux pins so they match up with the first 7 Auxiliary Feature slots avaliable in OnStep
// Aux1-3 can be used for pretty much anything
// Aux4-7 are more for dew-heaters
//#define AUX1_PIN                FAN0_PIN
//#define AUX2_PIN                FAN1_PIN
//#define AUX3_PIN                FAN2_PIN
//#define AUX4_PIN                HEATER0_PIN
//#define AUX5_PIN                HEATER1_PIN
//#define AUX6_PIN                HEATER2_PIN
//#define AUX7_PIN                HEATER3_PIN

// Misc. pins
//#ifndef DS3234_CS_PIN
//  #define DS3234_CS_PIN         PA4              // Default CS Pin for DS3234 on SPI (on EXP2 shared with the ESP8266 RST pin)
//#endif
//#ifndef BMx280_CS_PIN
//  #define BMx280_CS_PIN         PC7              // Default CS Pin for BME280/BMP280 on SPI (on EXP2 shared with LED2/Reticle)
//#endif
#ifndef ONE_WIRE_PIN
  #define ONE_WIRE_PIN          PC10              // Default Pin for OneWire bus (on E2-MOT PD-EN, right hand pin)
#endif
#define ADDON_GPIO0_PIN         PA1              // ESP8266 GPIO0 (on EXP1)
#define ADDON_RESET_PIN         PA4              // ESP8266 RST (on EXP2 shared with the DS3234 CS pin)

// The PEC index sense is a logic level input, resets the PEC index on rising edge then waits for 60 seconds before allowing another reset
#ifndef PEC_SENSE_PIN
#define PEC_SENSE_PIN           PB9              // PEC Sense, analog or digital (on X+ so it can have 3v3 or 5v on adjacent pin)
#endif

// The status LED is a two wire jumper with a 10k resistor in series to limit the current to the LED
#define STATUS_LED_PIN        PC13              // Drain (on EXP2) One could perhaps move these to the RGB leds, there's a header but no +5V present on it.
#define MOUNT_LED_PIN         PC12              // Drain (on EXP2 shared with Reticle/BME280_CS)

// The PPS pin is a 3.3V logic input, OnStep measures time between rising edges and adjusts the internal sidereal clock frequency
#ifndef PPS_SENSE_PIN
  #define PPS_SENSE_PIN         PB8          // PPS time source, GPS for example (on EXP2)
#endif

#ifndef LIMIT_SENSE_PIN
  #define LIMIT_SENSE_PIN       PA0              // Limit switch sense (on Z-)
#endif

//#define SHARED_ENABLE_PIN       PB0  
#define DEDICATED_MODE_PINS

// Axis1 RA/Azm step/dir driver
#define AXIS1_ENABLE_PIN        PB0
#define AXIS1_M0_PIN            PB3          // SPI MOSI
#define AXIS1_M1_PIN            PB4           // SPI SCK
#define AXIS1_M2_PIN            PA5              // SPI CS (UART TX)
#define AXIS1_M3_PIN            PA7          // SPI MISO (UART RX)
#define AXIS1_STEP_PIN          PB1
#define AXIS1_DIR_PIN           PC8
#define AXIS1_SENSE_HOME_PIN    PB14             // (on X-)


// Axis2 Dec/Alt step/dir driver
#define AXIS2_ENABLE_PIN        PB0
#define AXIS2_M0_PIN            PB5          // SPI MOSI
#define AXIS2_M1_PIN            PB12           // SPI SCK
#define AXIS2_M2_PIN            PA6             // SPI CS (UART TX)
#define AXIS2_M3_PIN            PA8        // SPI MISO (UART RX)
#define AXIS2_STEP_PIN          PC9
#define AXIS2_DIR_PIN           PA15
#define AXIS2_SENSE_HOME_PIN    PB13             // (on Y-)


// For rotator stepper driver
//#define AXIS3_ENABLE_PIN        PD15
//#define AXIS3_M0_PIN            SS_MOSI          // SPI MOSI
//#define AXIS3_M1_PIN            SS_SCK           // SPI SCK
//#define AXIS3_M2_PIN            PD10             // SPI CS (UART TX)
//#define AXIS3_M3_PIN            SS_MISO          // SPI MISO (UART RX)
//#define AXIS3_STEP_PIN          PD14
//#define AXIS3_DIR_PIN           PD13

// For focuser1 stepper driver
//#define AXIS4_ENABLE_PIN        PD4
//#define AXIS4_M0_PIN            SS_MOSI          // SPI MOSI
//#define AXIS4_M1_PIN            SS_SCK           // SPI SCK
//#define AXIS4_M2_PIN            PD7              // SPI CS (UART TX)
//#define AXIS4_M3_PIN            SS_MISO          // SPI MISO (UART RX)
//#define AXIS4_STEP_PIN          PD5
//#define AXIS4_DIR_PIN           PD6

// For focuser2 stepper driver
//#define AXIS5_ENABLE_PIN        PE5
//#define AXIS5_M0_PIN            SS_MOSI          // SPI MOSI
//#define AXIS5_M1_PIN            SS_SCK           // SPI SCK
//#define AXIS5_M2_PIN            PC14             // SPI CS (UART TX)
//#define AXIS5_M3_PIN            SS_MISO          // SPI MISO (UART RX)
//#define AXIS5_STEP_PIN          PE6
//#define AXIS5_DIR_PIN           PC13

// For focuser3 stepper driver
//#define AXIS6_ENABLE_PIN        PE3
//#define AXIS6_M0_PIN            SS_MOSI          // SPI MOSI
//#define AXIS6_M1_PIN            SS_SCK           // SPI SCK
//#define AXIS6_M2_PIN            PC15             // SPI CS (UART TX)
//#define AXIS6_M3_PIN            SS_MISO          // SPI MISO (UART RX)
//#define AXIS6_STEP_PIN          PE2
//#define AXIS6_DIR_PIN           PE4

// ST4 interface
#define ST4_RA_W_PIN            PC0              // ST4 RA- West  (on EXP1)
#define ST4_DEC_S_PIN           PC2              // ST4 DE- South (on EXP1)
#define ST4_DEC_N_PIN           PC1             // ST4 DE+ North (on EXP1)
#define ST4_RA_E_PIN            PC3              // ST4 RA+ East  (on EXP1)

#else
#error "Wrong processor for this configuration!"

#endif

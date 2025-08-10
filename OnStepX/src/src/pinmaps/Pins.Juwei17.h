// -------------------------------------------------------------------------------------------------
// Pin map for OnStep MaxSTM STM32F411CE/STM32F401CE PCB (or the Blackpill https://github.com/WeActTC/MiniF4-STM32F4x1)
#pragma once

#if defined(STM32F411xE) || defined(STM32F401xC)

// For an 8KB EEPROM on the MaxSTM3.6I
//#define NV_M24C64


// Serial ports (see Pins.defaults.h for SERIAL_A)
// Serial1 RX1 Pin PA10, TX1 Pin PA9
// Serial2 RX2 Pin PA3 , TX2 Pin PA2
// Serial6 RX6 Pin PA12, TX6 Pin PA11 (used for USB virtual serial)

#define DEDICATED_MODE_PINS

#if defined(STEP_DIR_TMC_UART_PRESENT) || defined(SERVO_TMC2209_PRESENT)
//  #define DRIVER_TMC_STEPPER
  #define SERIAL_TMC_RX               PA6
  #define SERIAL_TMC_TX               PB9
  #define SERIAL_TMC_BAUD             9600 
  #define SERIAL_TMC_ADDRESS_MAP(x)   ((x==4)?2 : x) // Axis1(0) is 0, Axis2(1) is 1, Axis3(2) is 2, Axis4(3) is 3, Axis5(4) is 2
  #define SERIAL_TMC_RX_DISABLE false
#endif

#define PIN_INIT() { \
  pinMode(PB9, OUTPUT);\
  digitalWrite(PB9, HIGH);\
}


#if SERIAL_A_BAUD_DEFAULT != OFF
  #define SERIAL_A              Serial
#endif
#if SERIAL_B_BAUD_DEFAULT != OFF
  #define SERIAL_B              HardSerial
  #define SERIAL_B_RX           PA3
  #define SERIAL_B_TX           PA2
#endif
#if SERIAL_C_BAUD_DEFAULT != OFF
  #define SERIAL_C              HardSerial
  #define SERIAL_C_RX           PA10
  #define SERIAL_C_TX           PA9
#endif

// Auto assign the Serial1 port pins for GPS
#if SERIAL_GPS_BAUD != OFF
  #ifndef SERIAL_GPS
    #define SERIAL_GPS          HardSerial
    #define SERIAL_GPS_RX       PA10
    #define SERIAL_GPS_TX       PA9
  #endif
#endif

// Misc. pins
#ifndef ONE_WIRE_PIN
  #define ONE_WIRE_PIN          OFF         // Default Pin for OneWire bus (note: this pin has a 0.1uF capacitor that must be removed for OneWire to function)
#endif
#define ADDON_GPIO0_PIN         PA14              // ESP8266 GPIO0 (shared with AXIS2_DIR_PIN)
#define ADDON_RESET_PIN         PA8              // ESP8266 RST

// The PEC index sense is a logic level input, resets the PEC index on rising edge then waits for 60 seconds before allowing another reset
#ifndef PEC_SENSE_PIN
  #define PEC_SENSE_PIN         OFF              // PEC Sense, analog or digital
#endif

// The status LED is a two wire jumper with a 10k resistor in series to limit the current to the LED
#define STATUS_LED_PIN          PB12         // Default LED Cathode (-)
#define MOUNT_LED_PIN           OFF         // Default LED Cathode (-)
#ifndef RETICLE_LED_PIN
  #define RETICLE_LED_PIN       OFF         // Default LED Cathode (-)
#endif

// For a piezo buzzer
#define STATUS_BUZZER_PIN       OFF             // Tone

// The PPS pin is a 3.3V logic input, OnStep measures time between rising edges and adjusts the internal sidereal clock frequency
#ifndef PPS_SENSE_PIN
    #define PPS_SENSE_PIN       PA13         // PPS time source, GPS for example (MaxSTM version 3.61 and later)
#endif

// The limit switch sense is a logic level input normally pull high (2k resistor,) shorted to ground it stops gotos/tracking
#ifndef LIMIT_SENSE_PIN
  #define LIMIT_SENSE_PIN       PB15
#endif

#define SHARED_ENABLE_PIN       PB8    
// Axis1 RA/Azm step/dir driver
#define AXIS1_ENABLE_PIN        SHARED
#define AXIS1_M0_PIN            OFF
#define AXIS1_M1_PIN            OFF
#define AXIS1_M2_PIN            PB9        // UART TX
#define AXIS1_M3_PIN            PA6         // UART RX
#define AXIS1_STEP_PIN          PB10
#define AXIS1_DIR_PIN           PB2
#define AXIS1_SENSE_HOME_PIN    PB13

// Axis2 Dec/Alt step/dir driver
#define AXIS2_ENABLE_PIN        SHARED
#define AXIS2_M0_PIN            OFF
#define AXIS2_M1_PIN            OFF
#define AXIS2_M2_PIN            PB9        // UART TX
#define AXIS2_M3_PIN            PA6         // UART RX
#define AXIS2_STEP_PIN          PA4
#define AXIS2_DIR_PIN           PB0
#define AXIS2_SENSE_HOME_PIN    PB14


// For rotator stepper driver
#define SHARED_DIRECTION_PINS                    // Hint that the direction pins are shared
#define AXIS3_ENABLE_PIN        OFF
#define AXIS3_M0_PIN            OFF              // SPI MOSI
#define AXIS3_M1_PIN            OFF              // SPI SCK
#define AXIS3_M2_PIN            OFF             // SPI CS (UART TX)
#define AXIS3_M3_PIN            OFF         // SPI MISO (UART RX)
#define AXIS3_STEP_PIN          OFF
#define AXIS3_DIR_PIN           OFF

// For focuser1 stepper driver
#define AXIS4_ENABLE_PIN        OFF
#define AXIS4_M0_PIN            OFF              // SPI MOSI
#define AXIS4_M1_PIN            OFF              // SPI SCK
#define AXIS4_M2_PIN            OFF             // SPI CS (UART TX)
#define AXIS4_M3_PIN            OFF         // SPI MISO (UART RX)
#define AXIS4_STEP_PIN          OFF
#define AXIS4_DIR_PIN           OFF

// For focuser2 stepper driver
#define AXIS5_ENABLE_PIN        OFF
#define AXIS5_M0_PIN            OFF              // SPI MOSI
#define AXIS5_M1_PIN            OFF              // SPI SCK
#define AXIS5_M2_PIN            OFF             // SPI CS (UART TX)
#define AXIS5_M3_PIN            OFF         // SPI MISO (UART RX)
#define AXIS5_STEP_PIN          OFF
#define AXIS5_DIR_PIN           OFF

// ST4 interface
#define ST4_RA_W_PIN            PA15             // ST4 RA- West
#define ST4_DEC_S_PIN           PB3              // ST4 DE- South
#define ST4_DEC_N_PIN           PB4              // ST4 DE+ North
#define ST4_RA_E_PIN            PB5              // ST4 RA+ East

#else
#error "Wrong processor for this configuration!"

#endif

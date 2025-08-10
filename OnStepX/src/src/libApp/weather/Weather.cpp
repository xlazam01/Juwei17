// -----------------------------------------------------------------------------------------------------------------------------
// Weather related functions

#include "Weather.h"

#include "../../lib/tasks/OnTask.h"

#include "../../libApp/temperature/Ds1820.h"
extern Ds1820 temperature;


extern bool xBusy;

#if WEATHER == BME280 || WEATHER == BME280_0x76 || WEATHER == BME280_SPI
  #include <Adafruit_BME280.h>          // https://github.com/adafruit/Adafruit_BME280_Library
                                        // and https://github.com/adafruit/Adafruit_Sensor
  #if WEATHER == BME280
    #define BME_ADDRESS 0x77
    Adafruit_BME280 bmx;
  #elif WEATHER == BME280_0x76
    #define BME_ADDRESS 0x76
    Adafruit_BME280 bmx;
  #elif WEATHER == BME280_SPI && defined(SSPI_SHARED)
    Adafruit_BME280 bmx(BMx280_CS_PIN, SSPI_MOSI, SSPI_MISO, SSPI_SCK); // software SPI
  #elif WEATHER == BME280_SPI
    Adafruit_BME280 bmx(BMx280_CS_PIN); // hardware SPI
  #endif
#endif

// BMP280 is the BME280 without humidity 
#if WEATHER == BMP280 || WEATHER == BMP280_0x76 || WEATHER == BMP280_SPI
  #include <Adafruit_BMP280.h>          // https://github.com/adafruit/Adafruit_BMP280_Library
                                        // and https://github.com/adafruit/Adafruit_Sensor
  #if WEATHER == BMP280
    #define BMP_ADDRESS 0x77
    Adafruit_BMP280 bmx(&HAL_WIRE);
  #elif WEATHER == BMP280_0x76
    #define BMP_ADDRESS 0x76
    Adafruit_BMP280 bmx(&HAL_WIRE);
  #elif WEATHER == BMP280_SPI
    Adafruit_BMP280 bmx(BMx280_CS_PIN); // hardware SPI
  #endif
#endif

void weatherPollWrapper() { weather.poll(); }

bool Weather::init() {
  #if WEATHER != OFF
    success = false;
    #if WEATHER == BME280 || WEATHER == BME280_0x76
      if (bmx.begin(BME_ADDRESS, &HAL_WIRE)) {
        bmx.setSampling(Adafruit_BME280::MODE_FORCED, Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::FILTER_OFF, Adafruit_BME280::STANDBY_MS_1000);
        weatherSensor = WS_BME280; success = true;
      } else { DF("WRN: Weather.init(), BME280 (I2C 0x"); if (DEBUG != OFF) SERIAL_DEBUG.print(BME_ADDRESS, HEX); DLF(") not found"); }
    #elif WEATHER == BMP280 || WEATHER == BMP280_0x76
      if (bmx.begin(BMP_ADDRESS)) {
        bmx.setSampling(Adafruit_BMP280::MODE_FORCED, Adafruit_BMP280::SAMPLING_X1, Adafruit_BMP280::SAMPLING_X1, Adafruit_BMP280::FILTER_OFF);
        weatherSensor = WS_BMP280; success = true;
      } else { DF("WRN: Weather.init(), BMP280 (I2C 0x"); if (DEBUG != OFF) SERIAL_DEBUG.print(BMP_ADDRESS, HEX); DLF(") not found"); }
    #elif WEATHER == BME280_SPI
      if (bmx.begin()) {
        bmx.setSampling(Adafruit_BME280::MODE_FORCED, Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::FILTER_OFF);
        weatherSensor = WS_BME280; success = true;
      } else { DLF("WRN: Weather.init(), BME280 (SPI) not found"); }
    #elif WEATHER == BMP280_SPI
      if (bmx.begin()) {
        bmx.setSampling(Adafruit_BMP280::MODE_FORCED, Adafruit_BMP280::SAMPLING_X1, Adafruit_BMP280::SAMPLING_X1, Adafruit_BMP280::FILTER_OFF);
        weatherSensor = WS_BMP280; success = true;
      } else { DLF("WRN: Weather.init(), BMP280 (SPI) not found"); }
    #else
      #error "Configuration: Setting WEATHER unknown sensor value!"
    #endif

    // follow any I2C device in-library init with a reset of the I2C bus speed
    #if WEATHER == BME280 || WEATHER == BME280_0x76 || WEATHER == BMP280_0x76 || WEATHER == BMP280
      #ifdef HAL_WIRE_RESET_AFTER_CONNECT
        HAL_WIRE.end();
        HAL_WIRE.begin();
        #ifdef HAL_WIRE_CLOCK
          HAL_WIRE.setClock(HAL_WIRE_CLOCK);
        #endif
      #endif
    #endif

    if (success) {
      VF("MSG: Weather, start weather monitor task (rate 5s priority 7)... ");
      if (tasks.add(5000, 0, true, 7, weatherPollWrapper, "WeaPoll")) { VLF("success"); } else { VLF("FAILED!"); }
    }
  #else
    success = true;
  #endif
  return success;
}

// poll the weather sensor once every 60 seconds
void Weather::poll() {
  #if WEATHER != OFF
    if (success && !xBusy) {
      static int phase = 0;
      switch (++phase) {
        case 1:
          temperature = ::temperature.getChannel(0);
          if (!isnan(temperature) && (temperature < -60.0F || temperature > 60.0F)) temperature = NAN;
          return;
        break;
        case 2:
          pressure = bmx.readPressure()/100.0F;
          if (!isnan(pressure) && (pressure < 100.0F || pressure > 1100.0F)) pressure = NAN;
          return;
        break;
        case 3: 
          #if WEATHER == BME280 || WEATHER == BME280_0x76 || WEATHER == BME280_SPI
            humidity = bmx.readHumidity();
            if (!isnan(humidity) && (humidity < 0.0F || humidity > 100.0F)) humidity = NAN;
          #endif
          phase = 0;
        break;
      }
      bmx.takeForcedMeasurement();

      // if any measurements are anomalous assume all are invalid
      #if WEATHER == BME280 || WEATHER == BME280_0x76 || WEATHER == BME280_SPI
        if (isnan(temperature) || isnan(pressure) || isnan(humidity)) {
          temperature = NAN;
          pressure = NAN;
          humidity = NAN;
        }
      #else
        if (isnan(temperature) || isnan(pressure)) {
          temperature = NAN;
          pressure = NAN;
        }
      #endif

  
        averageTemperature = temperature;
    }
    #if WEATHER_SUPRESS_ERRORS == OFF
      else { temperature = NAN; pressure = NAN; humidity = NAN; }
    #endif
  #endif
}

// get temperature in deg. C
float Weather::getTemperature() {
  return averageTemperature;
}

// set temperature in deg. C
bool Weather::setTemperature(float t) {
  if (weatherSensor == WS_NONE) { 
    if (t >= -60.0F && t <= 60.0F) { temperature = t; averageTemperature = t; } else return false;
  }
  return true;
}

// get barometric pressure in hPa/mb
float Weather::getPressure() {
  return pressure;
}

// set barometric pressure in hPa/mb
bool Weather::setPressure(float p) {
  if (weatherSensor == WS_NONE) { 
    if (p >= 100.0F && p <= 1100.0F) pressure = p; else return false;
  }
  return true;
}

// get relative humidity in %
float Weather::getHumidity() {
  return humidity;
}

// set relative humidity in %
bool Weather::setHumidity(float h) {
  if (weatherSensor == WS_NONE || weatherSensor == WS_BMP280) { 
    if (h >= 0.0F && h <= 100.0F) humidity = h; else return false;
  }
  return true;
}

// get dew point in deg. C
// accurate to +/- 1 deg. C for RH above 50%
float Weather::getDewPoint() {
  return averageTemperature - ((100.0F - humidity)/5.0F);
  // a more accurate formula?
  // return 243.04*(log(humidity/100.0) + ((17.625*averageTemperature)/(243.04 + averageTemperature)))/(17.625 - log(humidity/100.0) - ((17.625*averageTemperature)/(243.04 + averageTemperature)));
}

Weather weather;

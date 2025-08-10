// ----------------------------------------------------------------------------
// TMC2209.h
//
// Authors:
// Peter Polidoro peter@polidoro.io
// Howard Dutton hjd1964@gmail.com
// ----------------------------------------------------------------------------

#ifndef TMC2209_H
#define TMC2209_H
#include <Arduino.h>

// enable or disable debug messages (false/0 OFF, true/1 ON, 2 for error messages only)
#ifndef TMC2209_DEBUG
  #define TMC2209_DEBUG false
#endif

#if !defined(TMC2209_HARDWARE_SERIAL) && !defined(TMC2209_SOFTWARE_SERIAL)
  #define TMC2209_HARDWARE_SERIAL
#endif

#if defined(TMC2209_HARDWARE_SERIAL)
  //#include <HardwareSerial.h>
  #define HSSerial HardwareSerial
#elif defined(TMC2209_SOFTWARE_SERIAL)
  #include <SoftwareSerial.h>
  #define HSSerial SoftwareSerial
#endif

class TMC2209Stepper {
public:
  TMC2209Stepper() {
    blocking_ = true;
    serial_ptr_ = nullptr;
    serial_baud_rate_ = 500000;
    rx_ = -1;
    tx_ = -1;
    serial_address_ = 0;
    cool_step_enabled_ = false;
  }

  // identify which microcontroller serial port is connected to the TMC2209
  // specify rx and tx for SoftwareSerial (rx = -1 for write only)
  void setup(long serial_baud_rate, int serial_address = 0, int rx = -1, int tx = -1) {
    #if defined(TMC2209_HARDWARE_SERIAL)
      no_echo = false;
    #elif defined(TMC2209_SOFTWARE_SERIAL)
      if (serial_ptr_ == nullptr) serial_ptr_ = new SoftwareSerial(rx, tx);
      no_echo = true;
    #endif
    blocking_ = false;
    tx_only_ = (rx == -1);
    if (tx == -1) tx_only_ = false;
    serial_baud_rate_ = serial_baud_rate;
    rx_ = rx;
    tx_ = tx;

    #if defined(TMC2209_HARDWARE_SERIAL) && defined(ESP32)
      static bool hardwareSerialInitialized = false;
      if (!hardwareSerialInitialized) {
        #if TMC2209_DEBUG == 1
          Serial.println("Initializing hardware serial port");
        #endif
        if (rx_ >= 0 && tx_ >= 0) serial_ptr_->begin(serial_baud_rate_, SERIAL_8N1, rx_, tx_, invert_);
        else if (rx_ < 0 && tx_ >= 0) serial_ptr_->begin(serial_baud_rate_, SERIAL_8N1, -1, tx_, invert_);
        else serial_ptr_->begin(serial_baud_rate_);
        hardwareSerialInitialized = true;
      } else {
        #if TMC2209_DEBUG == 1
          Serial.println("Hardware serial port already initialized");
        #endif
      }
    #else
      #if TMC2209_DEBUG == 1
        Serial.println("Initializing software serial port");
      #endif
      serial_ptr_->begin(serial_baud_rate_);
      #ifdef TMC2209_SOFTWARE_SERIAL
        if (tx_ >= 0) {
          pinMode(tx_, OUTPUT);
          digitalWrite(tx_, HIGH);
        }
      #endif
    #endif

    setOperationModeToSerial(serial_baud_rate, serial_address);
    setRegistersToDefaults();
    readAndStoreRegisters();
    minimizeMotorCurrent();
    disable();
    disableAutomaticCurrentScaling();
    disableAutomaticGradientAdaptation();
    if (!isSetupAndCommunicating()) blocking_ = true;
  }

  // identify which microcontroller serial port is connected to the TMC2209
  void setup(HSSerial & serial, long serial_baud_rate = 500000, int serial_address = 0, int rx = -1, int tx = -1, bool invert = false) {
    serial_ptr_ = &serial;
    invert_ = invert;
    setup(serial_baud_rate, serial_address, rx, tx);
  }

  // if driver is not communicating, check power and communication connections
  bool isCommunicating() {
    if (tx_only_) return true;
    return (getVersion() == VERSION);
  }

  // check to make sure TMC2209 is properly setup and communicating
  bool isSetupAndCommunicating() {
    if (tx_only_) return true;
    return serialOperationMode();
  }

  // driver may be communicating but not setup if driver power is lost then
  // restored after setup so that defaults are loaded instead of setup options
  bool isCommunicatingButNotSetup() {
    if (tx_only_) return false;
    return (isCommunicating() && (not isSetupAndCommunicating()));
  }

  // driver must be enabled before use it is disabled by default
  void enable()  {
    if (blocking_) return;
    chopper_config_.toff = toff_;
    writeStoredChopperConfig();
  }

  void disable() {
    if (blocking_) return;
    chopper_config_.toff = TOFF_DISABLE;
    writeStoredChopperConfig();
  }

  // driver may also be disabled by the hardware enable input pin
  // this pin must be grounded or disconnected before driver may be enabled
  bool disabledByInputPin() {
    if (blocking_) return false;
    Input input;
    input.bytes = read(ADDRESS_IOIN);
    return input.enn;
  }

  // valid values = 1,2,4,8,...128,256
  bool setMicrostepsPerStep(uint16_t microsteps_per_step) {
    if (blocking_) return false;

    switch (microsteps_per_step) {
      case 1: chopper_config_.mres = 8; break;
      case 2: chopper_config_.mres = 7; break;
      case 4: chopper_config_.mres = 6; break;
      case 8: chopper_config_.mres = 5; break;
      case 16: chopper_config_.mres = 4; break;
      case 32: chopper_config_.mres = 3; break;
      case 64: chopper_config_.mres = 2; break;
      case 128: chopper_config_.mres = 1; break;
      case 256: chopper_config_.mres = 0; break;
      default: return false;
    }

    writeStoredChopperConfig();
    return true;
  }

  uint16_t getMicrostepsPerStep() {
    int mres[] = {256, 128, 64, 32, 16, 8, 4, 2, 1};
    chopper_config_.mres = constrain(chopper_config_.mres, (uint32_t)0, (uint32_t)8);
    return mres[chopper_config_.mres];
  }

  void setRunCurrent(uint8_t percent) {
    if (blocking_) return;

    uint8_t run_current = percentToCurrentSetting(percent);
    driver_current_.irun = run_current;
    writeStoredDriverCurrent();
  }

  void setHoldCurrent(uint8_t percent) {
    if (blocking_) return;
    uint8_t hold_current = percentToCurrentSetting(percent);
    driver_current_.ihold = hold_current;
    writeStoredDriverCurrent();
  }

  void setHoldDelay(uint8_t percent) {
    if (blocking_) return;
    uint8_t hold_delay = percentToHoldDelaySetting(percent);
    driver_current_.iholddelay = hold_delay;
    writeStoredDriverCurrent();
  }

  void setAllCurrentValues(uint8_t run_current_percent, uint8_t hold_current_percent, uint8_t hold_delay_percent) {
    if (blocking_) return;

    uint8_t run_current = percentToCurrentSetting(run_current_percent);
    uint8_t hold_current = percentToCurrentSetting(hold_current_percent);
    uint8_t hold_delay = percentToHoldDelaySetting(hold_delay_percent);

    driver_current_.irun = run_current;
    driver_current_.ihold = hold_current;
    driver_current_.iholddelay = hold_delay;
    writeStoredDriverCurrent();
  }

  struct Settings {
    bool is_communicating;
    bool is_setup;
    bool enabled;
    uint16_t microsteps_per_step;
    bool inverse_motor_direction_enabled;
    bool stealth_chop_enabled;
    uint8_t standstill_mode;
    uint8_t irun_percent;
    uint8_t irun_register_value;
    uint8_t ihold_percent;
    uint8_t ihold_register_value;
    uint8_t iholddelay_percent;
    uint8_t iholddelay_register_value;
    bool automatic_current_scaling_enabled;
    bool automatic_gradient_adaptation_enabled;
    uint8_t pwm_offset;
    uint8_t pwm_gradient;
    bool cool_step_enabled;
    bool analog_current_scaling_enabled;
    bool internal_sense_resistors_enabled;
  };
  Settings getSettings() {
    Settings settings;
    settings.is_communicating = isCommunicating();

    if (settings.is_communicating && !tx_only_) {
      readAndStoreRegisters();

      settings.is_setup = global_config_.pdn_disable;
      settings.enabled = (chopper_config_.toff > TOFF_DISABLE);
      settings.microsteps_per_step = getMicrostepsPerStep();
      settings.inverse_motor_direction_enabled = global_config_.shaft;
      settings.stealth_chop_enabled = not global_config_.enable_spread_cycle;
      settings.standstill_mode = pwm_config_.freewheel;
      settings.irun_percent = currentSettingToPercent(driver_current_.irun);
      settings.irun_register_value = driver_current_.irun;
      settings.ihold_percent = currentSettingToPercent(driver_current_.ihold);
      settings.ihold_register_value = driver_current_.ihold;
      settings.iholddelay_percent = holdDelaySettingToPercent(driver_current_.iholddelay);
      settings.iholddelay_register_value = driver_current_.iholddelay;
      settings.automatic_current_scaling_enabled = pwm_config_.pwm_autoscale;
      settings.automatic_gradient_adaptation_enabled = pwm_config_.pwm_autograd;
      settings.pwm_offset = pwm_config_.pwm_offset;
      settings.pwm_gradient = pwm_config_.pwm_grad;
      settings.cool_step_enabled = cool_step_enabled_;
      settings.analog_current_scaling_enabled = global_config_.i_scale_analog;
      settings.internal_sense_resistors_enabled = global_config_.internal_rsense;
    } else {
      settings.is_setup = settings.is_communicating;
      settings.enabled = settings.is_communicating;
      settings.microsteps_per_step = 32;
      settings.inverse_motor_direction_enabled = false;
      settings.stealth_chop_enabled = false;
      settings.standstill_mode = pwm_config_.freewheel;
      settings.irun_percent = 0;
      settings.irun_register_value = 0;
      settings.ihold_percent = 0;
      settings.ihold_register_value = 0;
      settings.iholddelay_percent = 50;
      settings.iholddelay_register_value = 0;
      settings.automatic_current_scaling_enabled = false;
      settings.automatic_gradient_adaptation_enabled = false;
      settings.pwm_offset = 0;
      settings.pwm_gradient = 0;
      settings.cool_step_enabled = false;
      settings.analog_current_scaling_enabled = false;
      settings.internal_sense_resistors_enabled = false;
    }

    return settings;
  }

  struct Status {
    uint32_t over_temperature_warning : 1;
    uint32_t over_temperature_shutdown : 1;
    uint32_t short_to_ground_a : 1;
    uint32_t short_to_ground_b : 1;
    uint32_t low_side_short_a : 1;
    uint32_t low_side_short_b : 1;
    uint32_t open_load_a : 1;
    uint32_t open_load_b : 1;
    uint32_t over_temperature_120c : 1;
    uint32_t over_temperature_143c : 1;
    uint32_t over_temperature_150c : 1;
    uint32_t over_temperature_157c : 1;
    uint32_t reserved0 : 4;
    uint32_t current_scaling : 5;
    uint32_t reserved1 : 9;
    uint32_t stealth_chop_mode : 1;
    uint32_t standstill : 1;
  };
  const static uint8_t CURRENT_SCALING_MAX = 31;
  Status getStatus() {
    DriveStatus drive_status;
    drive_status.bytes = 0;
    if (!blocking_)
      drive_status.bytes = read(ADDRESS_DRV_STATUS); 
    else
    {
      drive_status.status.over_temperature_warning = true;
      drive_status.status.over_temperature_shutdown = true;
      drive_status.status.short_to_ground_a = true;
      drive_status.status.short_to_ground_b = true;
      drive_status.status.low_side_short_a = true;
      drive_status.status.low_side_short_b = true;
      drive_status.status.open_load_a = true;
      drive_status.status.open_load_b = true;
      drive_status.status.over_temperature_120c = true;
      drive_status.status.over_temperature_143c = true;
      drive_status.status.over_temperature_150c = true;
      drive_status.status.over_temperature_157c = true;
      drive_status.status.reserved0 = 0;
      drive_status.status.current_scaling = 0;
      drive_status.status.reserved1 = 0;
      drive_status.status.stealth_chop_mode = 0;
      drive_status.status.standstill = true;
    }
    return drive_status.status;
  }

  void enableInverseMotorDirection() {
    if (blocking_) return;
    global_config_.shaft = 1;
    writeStoredGlobalConfig();
  }

  void disableInverseMotorDirection() {
    if (blocking_) return;
    global_config_.shaft = 0;
    writeStoredGlobalConfig();
  }

  enum StandstillMode { NORMAL = 0, FREEWHEELING = 1, STRONG_BRAKING = 2, BRAKING = 3};
  void setStandstillMode(TMC2209Stepper::StandstillMode mode) {
    if (blocking_) return;
    pwm_config_.freewheel = mode;
    writeStoredPwmConfig();
  }

  void enableAutomaticCurrentScaling() {
    if (blocking_) return;
    pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_ON;
    writeStoredPwmConfig();
  }

  void disableAutomaticCurrentScaling() {
    if (blocking_) return;
    pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_OFF;
    writeStoredPwmConfig();
  }

  void enableAutomaticGradientAdaptation() {
    if (blocking_) return;
    pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_ON;
    writeStoredPwmConfig();
  }

  void disableAutomaticGradientAdaptation() {
    if (blocking_) return;
    pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_OFF;
    writeStoredPwmConfig();
  }

  void setPwmOffset(uint8_t pwm_amplitude) {
    if (blocking_) return;
    pwm_config_.pwm_offset = pwm_amplitude;
    writeStoredPwmConfig();
  }

  void setPwmGradient(uint8_t pwm_amplitude) {
    if (blocking_) return;
    pwm_config_.pwm_grad = pwm_amplitude;
    writeStoredPwmConfig();
  }

  // default = 20, minimum of 2 for StealthChop auto tuning
  void setPowerDownDelay(uint8_t delay) {
    if (blocking_) return;
    write(ADDRESS_TPOWERDOWN,delay);
  }

  uint8_t getInterfaceTransmissionCounter() {
    if (blocking_) return 0;
    return read(ADDRESS_IFCNT);
  }

  void moveAtVelocity(int32_t microsteps_per_period) {
    if (blocking_) return;
    write(ADDRESS_VACTUAL,microsteps_per_period);
  }

  void moveUsingStepDirInterface() {
    if (blocking_) return;
    write(ADDRESS_VACTUAL,VACTUAL_STEP_DIR_INTERFACE);
  }

  void enableStealthChop() {
    if (blocking_) return;
    global_config_.enable_spread_cycle = 0;
    writeStoredGlobalConfig();
  }

  void disableStealthChop() {
    if (blocking_) return;
    global_config_.enable_spread_cycle = 1;
    writeStoredGlobalConfig();
  }

  uint32_t getInterstepDuration() {
    if (blocking_) return 0;
    return read(ADDRESS_TSTEP);
  }

  void setStealthChopDurationThreshold(uint32_t duration_threshold) {
    if (blocking_) return;
    write(ADDRESS_TPWMTHRS,duration_threshold);
  }

  uint16_t getStallGuardResult() {
    if (blocking_) return 0;
    return read(ADDRESS_SG_RESULT);
  }

  void setStallGuardThreshold(uint8_t stall_guard_threshold) {
    if (blocking_) return;
    write(ADDRESS_SGTHRS,stall_guard_threshold);
  }

  uint8_t getPwmScaleSum() {
    if (blocking_) return 0;
    PwmScale pwm_scale;
    pwm_scale.bytes = read(ADDRESS_PWM_SCALE);
    return pwm_scale.pwm_scale_sum;
  }

  int16_t getPwmScaleAuto() {
    if (blocking_) return 0;
    PwmScale pwm_scale;
    pwm_scale.bytes = read(ADDRESS_PWM_SCALE);
    return pwm_scale.pwm_scale_auto;
  }

  uint8_t getPwmOffsetAuto() {
    if (blocking_) return 0;
    PwmAuto pwm_auto;
    pwm_auto.bytes = read(ADDRESS_PWM_AUTO);
    return pwm_auto.pwm_offset_auto;
  }

  uint8_t getPwmGradientAuto() {
    if (blocking_) return 0;
    PwmAuto pwm_auto;
    pwm_auto.bytes = read(ADDRESS_PWM_AUTO);
    return pwm_auto.pwm_gradient_auto;
  }

  // lower_threshold: min = 1, max = 15
  // upper_threshold: min = 0, max = 15, 0-2 recommended
  void enableCoolStep(uint8_t lower_threshold = 1, uint8_t upper_threshold = 0) {
    if (blocking_) return;
    lower_threshold = constrain(lower_threshold,SEMIN_MIN,SEMIN_MAX);
    cool_config_.semin = lower_threshold;
    upper_threshold = constrain(upper_threshold,SEMAX_MIN,SEMAX_MAX);
    cool_config_.semax = upper_threshold;
    write(ADDRESS_COOLCONF,cool_config_.bytes);
    cool_step_enabled_ = true;
  }

  void disableCoolStep() {
    if (blocking_) return;
    cool_config_.semin = SEMIN_OFF;
    write(ADDRESS_COOLCONF,cool_config_.bytes);
    cool_step_enabled_ = false;
  }

  enum CurrentIncrement { CURRENT_INCREMENT_1 = 0, CURRENT_INCREMENT_2 = 1, CURRENT_INCREMENT_4 = 2, CURRENT_INCREMENT_8 = 3 };
  void setCoolStepCurrentIncrement(CurrentIncrement current_increment) {
    if (blocking_) return;
    cool_config_.seup = current_increment;
    write(ADDRESS_COOLCONF,cool_config_.bytes);
  }

  enum MeasurementCount {MEASUREMENT_COUNT_32 = 0, MEASUREMENT_COUNT_8 = 1, MEASUREMENT_COUNT_2 = 2, MEASUREMENT_COUNT_1 = 3};
  void setCoolStepMeasurementCount(MeasurementCount measurement_count) {
    if (blocking_) return;
    cool_config_.sedn = measurement_count;
    write(ADDRESS_COOLCONF, cool_config_.bytes);
  }

  void setCoolStepDurationThreshold(uint32_t duration_threshold) {
    if (blocking_) return;
    write(ADDRESS_TCOOLTHRS,duration_threshold);
  }

  uint16_t getMicrostepCounter() {
    if (blocking_) return 0;
    return read(ADDRESS_MSCNT);
  }

  void enableAnalogCurrentScaling() {
    if (blocking_) return;
    global_config_.i_scale_analog = 1;
    writeStoredGlobalConfig();
  }

  void disableAnalogCurrentScaling() {
    if (blocking_) return;
    global_config_.i_scale_analog = 0;
    writeStoredGlobalConfig();
  }

  void useExternalSenseResistors() {
    if (blocking_) return;
    global_config_.internal_rsense = 0;
    writeStoredGlobalConfig();
  }

  void useInternalSenseResistors() {
    if (blocking_) return;
    global_config_.internal_rsense = 1;
    writeStoredGlobalConfig();
  }

private:
  bool blocking_;
  unsigned long lastCommandMillis = 0;
  unsigned long nextCommandReadyMicros = 0;

  uint32_t serial_baud_rate_;
  int rx_;
  int tx_;
  bool invert_ = false;
  uint8_t serial_address_;

  // Serial Settings
  const static uint8_t BYTE_MAX_VALUE = 0xFF;
  const static uint8_t BITS_PER_BYTE = 8;

  const static uint32_t ECHO_DELAY_MAX_MICROSECONDS = 35000;
  const static uint32_t REPLY_DELAY_MAX_MICROSECONDS = 35000;

  const static uint8_t STEPPER_DRIVER_FEATURE_OFF = 0;
  const static uint8_t STEPPER_DRIVER_FEATURE_ON = 1;

  // Datagrams
  const static uint8_t WRITE_READ_REPLY_DATAGRAM_SIZE = 8;
  const static uint8_t DATA_SIZE = 4;
  union WriteReadReplyDatagram {
    struct {
      uint64_t sync : 4;
      uint64_t reserved : 4;
      uint64_t serial_address : 8;
      uint64_t register_address : 7;
      uint64_t rw : 1;
      uint64_t data : 32;
      uint64_t crc : 8;
    };
    uint64_t bytes;
  };

  const static uint8_t SYNC = 0b101;
  const static uint8_t RW_READ = 0;
  const static uint8_t RW_WRITE = 1;
  const static uint8_t READ_REPLY_SERIAL_ADDRESS = 0b11111111;

  const static uint8_t READ_REQUEST_DATAGRAM_SIZE = 4;
  union ReadRequestDatagram {
    struct {
      uint32_t sync : 4;
      uint32_t reserved : 4;
      uint32_t serial_address : 8;
      uint32_t register_address : 7;
      uint32_t rw : 1;
      uint32_t crc : 8;
    };
    uint32_t bytes;
  };

  // General Configuration Registers
  const static uint8_t ADDRESS_GCONF = 0x00;
  union GlobalConfig {
    struct {
      uint32_t i_scale_analog : 1;
      uint32_t internal_rsense : 1;
      uint32_t enable_spread_cycle : 1;
      uint32_t shaft : 1;
      uint32_t index_otpw : 1;
      uint32_t index_step : 1;
      uint32_t pdn_disable : 1;
      uint32_t mstep_reg_select : 1;
      uint32_t multistep_filt : 1;
      uint32_t test_mode : 1;
      uint32_t reserved : 22;
    };
    uint32_t bytes;
  };
  GlobalConfig global_config_;

  const static uint8_t ADDRESS_GSTAT = 0x01;
  union GlobalStatus {
    struct {
      uint32_t reset : 1;
      uint32_t drv_err : 1;
      uint32_t uv_cp : 1;
      uint32_t reserved : 29;
    };
    uint32_t bytes;
  };

  const static uint8_t ADDRESS_IFCNT = 0x02;

  const static uint8_t ADDRESS_SENDDELAY = 0x03;
  union SendDelay {
    struct {
      uint32_t reserved_0 : 8;
      uint32_t senddelay : 8;
      uint32_t reserved_1 : 16;
    };
    uint32_t bytes;
  };

  const static uint8_t ADDRESS_IOIN = 0x06;
  union Input {
    struct {
      uint32_t enn : 1;
      uint32_t reserved_0 : 1;
      uint32_t ms1 : 1;
      uint32_t ms2 : 1;
      uint32_t diag : 1;
      uint32_t reserved_1 : 1;
      uint32_t pdn_serial : 1;
      uint32_t step : 1;
      uint32_t spread_en : 1;
      uint32_t dir : 1;
      uint32_t reserved_2 : 14;
      uint32_t version : 8;
    };
    uint32_t bytes;
  };
  const static uint8_t VERSION = 0x21;


  // Velocity Dependent Driver Feature Control Register Set
  const static uint8_t ADDRESS_IHOLD_IRUN = 0x10;
  union DriverCurrent {
    struct {
      uint32_t ihold : 5;
      uint32_t reserved_0 : 3;
      uint32_t irun : 5;
      uint32_t reserved_1 : 3;
      uint32_t iholddelay : 4;
      uint32_t reserved_2 : 12;
    };
    uint32_t bytes;
  };
  DriverCurrent driver_current_;
  const static uint8_t PERCENT_MIN = 0;
  const static uint8_t PERCENT_MAX = 100;
  const static uint8_t CURRENT_SETTING_MIN = 0;
  const static uint8_t CURRENT_SETTING_MAX = 31;
  const static uint8_t HOLD_DELAY_MIN = 0;
  const static uint8_t HOLD_DELAY_MAX = 15;
  const static uint8_t IHOLD_DEFAULT = 16;
  const static uint8_t IRUN_DEFAULT = 31;
  const static uint8_t IHOLDDELAY_DEFAULT = 1;

  const static uint8_t ADDRESS_TPOWERDOWN = 0x11;
  const static uint8_t TPOWERDOWN_DEFAULT = 20;

  const static uint8_t ADDRESS_TSTEP = 0x12;

  const static uint8_t ADDRESS_TPWMTHRS = 0x13;
  const static uint32_t TPWMTHRS_DEFAULT = 0;

  const static uint8_t ADDRESS_VACTUAL = 0x22;
  const static int32_t VACTUAL_DEFAULT = 0;
  const static int32_t VACTUAL_STEP_DIR_INTERFACE = 0;

  // CoolStep and StallGuard Control Register Set
  const static uint8_t ADDRESS_TCOOLTHRS = 0x14;
  const static uint8_t TCOOLTHRS_DEFAULT = 0;
  const static uint8_t ADDRESS_SGTHRS = 0x40;
  const static uint8_t SGTHRS_DEFAULT = 0;
  const static uint8_t ADDRESS_SG_RESULT = 0x41;

  const static uint8_t ADDRESS_COOLCONF = 0x42;
  const static uint8_t COOLCONF_DEFAULT = 0;
  union CoolConfig {
    struct {
      uint32_t semin : 4;
      uint32_t reserved_0 : 1;
      uint32_t seup : 2;
      uint32_t reserved_1 : 1;
      uint32_t semax : 4;
      uint32_t reserved_2 : 1;
      uint32_t sedn : 2;
      uint32_t seimin : 1;
      uint32_t reserved_3 : 16;
    };
    uint32_t bytes;
  };
  CoolConfig cool_config_;
  bool cool_step_enabled_;
  const static uint8_t SEIMIN_UPPER_CURRENT_LIMIT = 20;
  const static uint8_t SEIMIN_LOWER_SETTING = 0;
  const static uint8_t SEIMIN_UPPER_SETTING = 1;
  const static uint8_t SEMIN_OFF = 0;
  const static uint8_t SEMIN_MIN = 1;
  const static uint8_t SEMIN_MAX = 15;
  const static uint8_t SEMAX_MIN = 0;
  const static uint8_t SEMAX_MAX = 15;

  // Microstepping Control Register Set
  const static uint8_t ADDRESS_MSCNT = 0x6A;
  const static uint8_t ADDRESS_MSCURACT = 0x6B;

  // Driver Register Set
  const static uint8_t ADDRESS_CHOPCONF = 0x6C;
  union ChopperConfig {
    struct {
      uint32_t toff : 4;
      uint32_t hstart : 3;
      uint32_t hend : 4;
      uint32_t reserved_0 : 4;
      uint32_t tbl : 2;
      uint32_t vsense : 1;
      uint32_t reserved_1 : 6;
      uint32_t mres : 4;
      uint32_t interpolation : 1;
      uint32_t double_edge : 1;
      uint32_t diss2g : 1;
      uint32_t diss2vs : 1;
    };
    uint32_t bytes;
  };
  ChopperConfig chopper_config_;
  const static uint32_t CHOPPER_CONFIG_DEFAULT = 0x10000053;
  const static uint8_t TBL_DEFAULT = 0b10;
  const static uint8_t HEND_DEFAULT = 0;
  const static uint8_t HSTART_DEFAULT = 5;
  const static uint8_t TOFF_DEFAULT = 3;
  const static uint8_t TOFF_DISABLE = 0;
  uint8_t toff_ = TOFF_DEFAULT;
  const static uint8_t MRES_256 = 0b0000;
  const static uint8_t MRES_128 = 0b0001;
  const static uint8_t MRES_064 = 0b0010;
  const static uint8_t MRES_032 = 0b0011;
  const static uint8_t MRES_016 = 0b0100;
  const static uint8_t MRES_008 = 0b0101;
  const static uint8_t MRES_004 = 0b0110;
  const static uint8_t MRES_002 = 0b0111;
  const static uint8_t MRES_001 = 0b1000;

  const static size_t MICROSTEPS_PER_STEP_MIN = 1;
  const static size_t MICROSTEPS_PER_STEP_MAX = 256;

  const static uint8_t ADDRESS_DRV_STATUS = 0x6F;
  union DriveStatus {
    struct {
      Status status;
    };
    uint32_t bytes;
  };

  const static uint8_t ADDRESS_PWMCONF = 0x70;
  union PwmConfig {
    struct  {
      uint32_t pwm_offset : 8;
      uint32_t pwm_grad : 8;
      uint32_t pwm_freq : 2;
      uint32_t pwm_autoscale : 1;
      uint32_t pwm_autograd : 1;
      uint32_t freewheel : 2;
      uint32_t reserved : 2;
      uint32_t pwm_reg : 4;
      uint32_t pwm_lim : 4;
    };
    uint32_t bytes;
  };
  PwmConfig pwm_config_;
  const static uint32_t PWM_CONFIG_DEFAULT = 0xC10D0024;
  const static uint8_t PWM_OFFSET_MIN = 0;
  const static uint8_t PWM_OFFSET_MAX = 255;
  const static uint8_t PWM_OFFSET_DEFAULT = 0x24;
  const static uint8_t PWM_GRAD_MIN = 0;
  const static uint8_t PWM_GRAD_MAX = 255;
  const static uint8_t PWM_GRAD_DEFAULT = 0x14;

  union PwmScale {
    struct {
      uint32_t pwm_scale_sum : 8;
      uint32_t reserved_0 : 8;
      uint32_t pwm_scale_auto : 9;
      uint32_t reserved_1 : 7;
    };
    uint32_t bytes;
  };
  const static uint8_t ADDRESS_PWM_SCALE = 0x71;

  union PwmAuto {
    struct {
      uint32_t pwm_offset_auto : 8;
      uint32_t reserved_0 : 8;
      uint32_t pwm_gradient_auto : 8;
      uint32_t reserved_1 : 8;
    };
    uint32_t bytes;
  };
  const static uint8_t ADDRESS_PWM_AUTO = 0x72;

  bool tx_only_ = false;
  bool no_echo = false;

  void setOperationModeToSerial(long serial_baud_rate, int serial_address = 0) {
    serial_address_ = serial_address;

    global_config_.bytes = 0;
    global_config_.i_scale_analog = 0;
    global_config_.pdn_disable = 1;
    global_config_.mstep_reg_select = 1;
    global_config_.multistep_filt = 1;

    writeStoredGlobalConfig();
  }

  void setRegistersToDefaults() {
    driver_current_.bytes = 0;
    driver_current_.ihold = IHOLD_DEFAULT;
    driver_current_.irun = IRUN_DEFAULT;
    driver_current_.iholddelay = IHOLDDELAY_DEFAULT;
    write(ADDRESS_IHOLD_IRUN,driver_current_.bytes);

    chopper_config_.bytes = CHOPPER_CONFIG_DEFAULT;
    chopper_config_.tbl = TBL_DEFAULT;
    chopper_config_.hend = HEND_DEFAULT;
    chopper_config_.hstart = HSTART_DEFAULT;
    chopper_config_.toff = TOFF_DEFAULT;
    write(ADDRESS_CHOPCONF,chopper_config_.bytes);

    pwm_config_.bytes = PWM_CONFIG_DEFAULT;
    write(ADDRESS_PWMCONF,pwm_config_.bytes);

    cool_config_.bytes = COOLCONF_DEFAULT;
    write(ADDRESS_COOLCONF,cool_config_.bytes);

    write(ADDRESS_TPOWERDOWN,TPOWERDOWN_DEFAULT);
    write(ADDRESS_TPWMTHRS,TPWMTHRS_DEFAULT);
    write(ADDRESS_VACTUAL,VACTUAL_DEFAULT);
    write(ADDRESS_TCOOLTHRS,TCOOLTHRS_DEFAULT);
    write(ADDRESS_SGTHRS,SGTHRS_DEFAULT);
    write(ADDRESS_COOLCONF,COOLCONF_DEFAULT);
  }

  void readAndStoreRegisters() {
    if (!tx_only_) {
      global_config_.bytes = readGlobalConfigBytes();
      chopper_config_.bytes = readChopperConfigBytes();
      pwm_config_.bytes = readPwmConfigBytes();
    }
  }

  uint8_t getVersion() {
    Input input;
    input.bytes = read(ADDRESS_IOIN);
    return input.version;
  }

  bool serialOperationMode() {
    GlobalConfig global_config;
    global_config.bytes = readGlobalConfigBytes();
    return global_config.pdn_disable;
  }

  void minimizeMotorCurrent() {
    driver_current_.irun = CURRENT_SETTING_MIN;
    driver_current_.ihold = CURRENT_SETTING_MIN;
    writeStoredDriverCurrent();
  }

  uint32_t reverseData(uint32_t data) {
    uint32_t reversed_data = 0;
    uint8_t right_shift;
    uint8_t left_shift;
    for (uint8_t i = 0; i < DATA_SIZE; ++i) {
      right_shift = (DATA_SIZE - i - 1) * BITS_PER_BYTE;
      left_shift = i * BITS_PER_BYTE;
      reversed_data |= ((data >> right_shift) & BYTE_MAX_VALUE) << left_shift;
    }
    return reversed_data;
  }

  template<typename Datagram>
  uint8_t calculateCrc(Datagram & datagram, uint8_t datagram_size) {
    uint8_t crc = 0;
    uint8_t byte;
    for (uint8_t i = 0; i < (datagram_size - 1); ++i) {
      byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
      for (uint8_t j = 0; j < BITS_PER_BYTE; ++j) {
        if ((crc >> 7) ^ (byte & 0x01)) crc = (crc << 1) ^ 0x07; else crc = crc << 1;
        byte = byte >> 1;
      }
    }
    return crc;
  }

  template<typename Datagram>
  void sendDatagram(Datagram & datagram, uint8_t datagram_size) {
    yield();

    uint8_t byte;

    // clear the serial receive buffer if necessary
    while (serial_ptr_->available() > 0) serial_ptr_->read();

    // and wait if necessary
    if ((long)(millis() - lastCommandMillis) < 2) {
      while ((long)(micros() - nextCommandReadyMicros) < 0) ;
    }

    #if TMC2209_DEBUG == 1
      Serial.print("Sending: ");
    #endif
    for (uint8_t i = 0; i < datagram_size; ++i) {
      byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
      serial_ptr_->write(byte);
      #if TMC2209_DEBUG == 1
        Serial.print(byte, HEX); Serial.print(", ");
      #endif
    }
    #if TMC2209_DEBUG == 1
      Serial.println("");
    #endif

    if (!no_echo && !tx_only_) {
      // wait for bytes sent out on TX line to be echoed on RX line
      int echo_count = 0;
      uint32_t echo_delay = micros();

      #if TMC2209_DEBUG == 1
        Serial.print("Clear echo bytes: ");
      #endif

      while ((long)(micros() - echo_delay) < (long)ECHO_DELAY_MAX_MICROSECONDS) {
        if (serial_ptr_->available()) {
          byte = serial_ptr_->read();
          #if TMC2209_DEBUG == 1
            Serial.print(byte, HEX); Serial.print(", ");
          #endif
          echo_count++;
          if (echo_count == datagram_size) break;
        }
      }
      #if TMC2209_DEBUG == 1
        if (echo_count == 0) Serial.println("(none)"); else Serial.println("");
      #endif

      // length 0 return is assumed to be due to HardwareSerial not being full duplex?
      if (echo_count == 0 && datagram_size == WRITE_READ_REPLY_DATAGRAM_SIZE) {
        #if TMC2209_DEBUG >= 1
          Serial.println("Write without echo detected, switching to no echo mode");
        #endif
        no_echo = true;
        tx_only_ = true;
        return;
      } else

      // wrong length return is an error
      if (echo_count < datagram_size) {
        #if TMC2209_DEBUG >= 1
          Serial.print("Error, didn't get echo: "); Serial.print(echo_count); Serial.print(" < "); Serial.println(datagram_size);
        #endif
        blocking_ = true;
        return;
      }

    } 
    #if defined(TMC2209_HARDWARE_SERIAL)
      else serial_ptr_->flush();
    #endif
  }

  void write(uint8_t register_address, uint32_t data) {
    yield();
    listen();

    WriteReadReplyDatagram write_datagram;
    write_datagram.bytes = 0;
    write_datagram.sync = SYNC;
    write_datagram.serial_address = serial_address_;
    write_datagram.register_address = register_address;
    write_datagram.rw = RW_WRITE;
    write_datagram.data = reverseData(data);
    write_datagram.crc = calculateCrc(write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);

    sendDatagram(write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);

    lastCommandMillis = millis();
    nextCommandReadyMicros = micros() + ((1000 * 1000 * 16) / serial_baud_rate_);
  }

  uint32_t read(uint8_t register_address) {
    if (tx_only_) return 0;

    yield();
    listen();

    ReadRequestDatagram read_request_datagram;
    read_request_datagram.bytes = 0;
    read_request_datagram.sync = SYNC;
    read_request_datagram.serial_address = serial_address_;
    read_request_datagram.register_address = register_address;
    read_request_datagram.rw = RW_READ;
    read_request_datagram.crc = calculateCrc(read_request_datagram,READ_REQUEST_DATAGRAM_SIZE);
    sendDatagram(read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

    uint32_t reply_start_time = micros();
    while ((long)(micros() - reply_start_time) < (long)REPLY_DELAY_MAX_MICROSECONDS) {
      if (serial_ptr_->available() >= WRITE_READ_REPLY_DATAGRAM_SIZE) break;
    }

    if (serial_ptr_->available() < WRITE_READ_REPLY_DATAGRAM_SIZE) {
      #if TMC2209_DEBUG >= 1
        Serial.print("Return timed out, read ");
        Serial.print(serial_ptr_->available());
        Serial.print(" bytes: ");
        while (serial_ptr_->available()) { Serial.print(serial_ptr_->read(), HEX); Serial.print(", "); }
        Serial.println("");
      #endif
      blocking_ = true;
      return 0;
    }
    #if TMC2209_DEBUG == 1
      Serial.println("Returned: ");
    #endif

    uint64_t byte;
    uint8_t byte_count = 0;
    WriteReadReplyDatagram read_reply_datagram;
    read_reply_datagram.bytes = 0;
    for (uint8_t i = 0; i<WRITE_READ_REPLY_DATAGRAM_SIZE; ++i) {
      byte = serial_ptr_->read();
      read_reply_datagram.bytes |= (byte << (byte_count++ * BITS_PER_BYTE));
      #if TMC2209_DEBUG == 1
        Serial.print(byte, HEX); Serial.print(", ");
      #endif
    }
    #if TMC2209_DEBUG == 1
      Serial.println("");
    #endif

    lastCommandMillis = millis();
    nextCommandReadyMicros = micros() + ((1000 * 1000 * 32) / serial_baud_rate_);

    return reverseData(read_reply_datagram.data);
  }

  uint8_t percentToCurrentSetting(uint8_t percent) {
    uint8_t constrained_percent = constrain(percent, PERCENT_MIN, PERCENT_MAX);
    uint8_t current_setting = map(constrained_percent, PERCENT_MIN, PERCENT_MAX, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX);
    return current_setting;
  }
  uint8_t currentSettingToPercent(uint8_t current_setting) {
    uint8_t percent = map(current_setting, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX, PERCENT_MIN, PERCENT_MAX);
    return percent;
  }
  uint8_t percentToHoldDelaySetting(uint8_t percent) {
    uint8_t constrained_percent = constrain(percent, PERCENT_MIN, PERCENT_MAX);
    uint8_t hold_delay_setting = map(constrained_percent, PERCENT_MIN, PERCENT_MAX, HOLD_DELAY_MIN, HOLD_DELAY_MAX);
    return hold_delay_setting;
  }
  uint8_t holdDelaySettingToPercent(uint8_t hold_delay_setting) {
    uint8_t percent = map(hold_delay_setting, HOLD_DELAY_MIN, HOLD_DELAY_MAX, PERCENT_MIN, PERCENT_MAX);
    return percent;
  }

  uint8_t pwmAmplitudeToPwmAmpl(uint8_t pwm_amplitude);
  uint8_t pwmAmplitudeToPwmGrad(uint8_t pwm_amplitude);

  void writeStoredGlobalConfig() { write(ADDRESS_GCONF,global_config_.bytes); }
  uint32_t readGlobalConfigBytes() { return read(ADDRESS_GCONF); }
  void writeStoredDriverCurrent() {
    write(ADDRESS_IHOLD_IRUN, driver_current_.bytes);
    if (driver_current_.irun >= SEIMIN_UPPER_CURRENT_LIMIT) cool_config_.seimin = SEIMIN_UPPER_SETTING; else cool_config_.seimin = SEIMIN_LOWER_SETTING;
    if (cool_step_enabled_) write(ADDRESS_COOLCONF,cool_config_.bytes);
  }
  void writeStoredChopperConfig() { write(ADDRESS_CHOPCONF, chopper_config_.bytes); }
  uint32_t readChopperConfigBytes() { return read(ADDRESS_CHOPCONF); }
  void writeStoredPwmConfig() { write(ADDRESS_PWMCONF, pwm_config_.bytes); }
  uint32_t readPwmConfigBytes() { return read(ADDRESS_PWMCONF); }

  #ifdef TMC2209_HARDWARE_SERIAL
    void listen() { }
  #else
    void listen() { if (serial_ptr_ != nullptr) serial_ptr_->listen(); }
  #endif

  HSSerial * serial_ptr_ = nullptr;

};

#endif

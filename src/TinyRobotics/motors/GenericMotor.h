#pragma once
#include "Motor.h"
#include "stdint.h"

namespace tinyrobotics {

/**
 * @class GenericMotor
 * @ingroup motors
 * @brief Motor abstraction for integrating external/custom motor drivers using
 * callbacks.
 *
 * This class provides a flexible interface for controlling motors that do not
 * fit the standard interface. The user supplies an external motor object (as a
 * void pointer) and callback functions for starting/stopping the motor and for
 * setting the speed or angle. This allows integration of custom motor drivers
 * or hardware with the TinyRobotics framework.
 *
 * - The @c valueCB callback is mandatory and is called with the desired speed
 * or angle value.
 * - The @c beginCB and @c endCB callbacks are optional and can be used to
 * define custom start/stop logic.
 * - The user motor object can be accessed via @c getMotor<T>().
 *
 * Example usage:
 * @code
 * MyMotorDriver driver;
 * GenericMotor motor(0, &driver);
 * motor.setValueCallback([](int8_t value, GenericMotor& m) {
 *   MyMotorDriver* drv = m.getMotor<MyMotorDriver>();
 *   drv->setPWM(value);
 * });
 * motor.setBeginCallback([](GenericMotor& m) {
 *   // Custom start logic
 *   return true;
 * });
 * motor.setEndCallback([](GenericMotor& m) {
 *   // Custom stop logic
 * });
 * @endcode
 */

class GenericMotor : public Motor {
 public:
  GenericMotor(uint8_t id = 0, void* motor = nullptr) {
    setID(id);
    this->motor = motor;
  }

  bool begin() override {
    if (valueCB == nullptr) {
      return false;  // Callbacks not defined, cannot start
    }
    bool result = true;
    // Call optional callback to start the motor
    if (beginCB) result = beginCB(*this);
    return result;
  }

  void setSpeed(int8_t percent) {
    value = percent;  // Store the last set speed value
    if (valueCB) {
      valueCB(percent, *this);
    }
  }

  int8_t getSpeed() {
    return value;  // Not implemented, as this is a generic callback motor
  }

  void setAngle(int8_t angle) {
    value = angle;  // Store the last set angle value
    if (valueCB) {
      valueCB(angle, *this);
    }
  }

  int8_t getAngle() {
    return value;  // Not implemented, as this is a generic callback motor
  }

  void end() override {
    if (endCB) {
      endCB(*this);  // Call callback to stop the motor
    }
  }

  /// Mandatory callback to define speed/angle control behavior
  void setValueCallback(void (*valueCB)(int8_t value, GenericMotor& motor)) {
    this->valueCB = valueCB;
  }

  /// Optional callback to define begin behavior
  void setBeginCallback(bool (*beginCB)(GenericMotor& motor)) {
    this->beginCB = beginCB;
  }

  /// Optional callback to define end behavior
  void setEndCallback(void (*endCB)(GenericMotor& motor)) {
    this->endCB = endCB;
  }

  /// Provides the user motor object
  template <typename T>
  T& getMotor() {
    return *static_cast<T*>(motor);
  }

  /// Not supported
  bool isPinsSet() const override { return true; }
  /// Not supported
  void setPin(int pin) { notSupported(); }
  /// Not supported
  void setPins(int pin1, int pin2, int pwmPin = -1, int pwmFreq = 20000) {
    notSupported();
  }
  /// Not supported
  void setPins(int stepPin, int dirPin, int enablePin = -1) { notSupported(); }

 protected:
  void* motor = nullptr;  // Optional pointer to user motor object for callbacks
  int8_t value = 0;       // Current speed or angle value
  bool (*beginCB)(GenericMotor& motor) = nullptr;
  void (*endCB)(GenericMotor& motor) = defaultEnd;
  void (*valueCB)(int8_t value, GenericMotor& motor) =
      nullptr;  // Callback function pointer for speed/angle control

  /// Default logic for end processing: just stop the motor!
  static void defaultEnd(GenericMotor& motor) {
    motor.setSpeed(
        0);  // Default behavior: stop the motor by setting speed to 0
  }
  void notSupported() {
    TRLogger.error("This method is not supported for GenericMotor");
  }
};

}  // namespace tinyrobotics
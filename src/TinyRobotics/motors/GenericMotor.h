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

template <typename T = float>
class GenericMotor : public Motor<T> {
 public:
  GenericMotor() = default;
  GenericMotor(uint8_t id, void* motor = nullptr) {
    this->setID(id);
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

  // Set value as percentage (-100 to 100)
  bool setValuePercent(T percent) override {
    value = constrain(percent, -100.0f, 100.0f);
    if (valueCB) {
      valueCB(percent, *this);
    }
    return true;
  }

  T getValuePercent() const override { return value; }

  void setAngle(T angle) {
    float valuePercent =
        map(angle, -90.0f, 90.0f, -100.0f, 100.0f);  // Map angle to percentage
    setValuePercent(
        valuePercent);  // For simplicity, treat angle as a percentage value
  }

  T getAngle() {
    return map(value, -100.0f, 100.0f, -90.0f,
               90.0f);  // Map percentage back to angle
  }

  void end() override {
    if (endCB) {
      endCB(*this);  // Call callback to stop the motor
    }
  }

  /// Mandatory callback to define speed/angle control behavior
  void setValueCallback(void (*valueCB)(T value, GenericMotor& motor)) {
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
  template <typename MotorT>
  MotorT& getMotor() {
    return *static_cast<MotorT*>(motor);
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
  T value = 0.0f;         // Current value percentage
  bool (*beginCB)(GenericMotor& motor) = nullptr;
  void (*endCB)(GenericMotor& motor) = defaultEnd;
  void (*valueCB)(T value, GenericMotor& motor) =
      nullptr;  // Callback function pointer for speed/angle control

  /// Default logic for end processing: just stop the motor!
  static void defaultEnd(GenericMotor& motor) {
    motor.setValuePercent(
        0);  // Default behavior: stop the motor by setting speed to 0
  }
  void notSupported() {
    TRLogger.error("This method is not supported for GenericMotor");
  }
};

}  // namespace tinyrobotics
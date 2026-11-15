#pragma once

/**
 * @file StepperMotorLinux.h
 * @ingroup motors
 * @brief Stepper driver for Raspberry Pi (Linux) using hardware PWM.
 *
 * Only continuous speed mode is supported: the step pin is driven with a
 * PWM signal whose frequency is the step rate, and the direction pin selects
 * the rotation direction. Counted moves (moveRevolutions, moveDistance) and
 * acceleration ramps are NOT supported, because sysfs PWM cannot count steps.
 *
 * Hardware PWM is available only on BCM pins 12, 13, 18 and 19, which share
 * two PWM channels. Do not use the same channel for a servo and a stepper.
 */

#include "Arduino.h"
#include "HardwareGPIO_RPI.h"
#include "HardwareSetupRPI.h"
#include "IMotor.h"

namespace tinyrobotics {

/**
 * @class StepperMotorLinux
 * @ingroup motors
 * @brief Continuous-speed stepper driver for Raspberry Pi using PWM step pulses.
 *
 * Usage:
 * @code
 * StepperMotorLinux<float> stepper(0);
 * stepper.setPins(18, 23, 24);  // step (PWM), direction, enable
 * stepper.setMaxSpeed(1000);    // steps per second at 100%
 * stepper.begin();
 * stepper.setValuePercent(50);  // 500 steps/s forward
 * stepper.end();                // stop pulses
 * @endcode
 */
template <typename T = float>
class StepperMotorLinux : public IMotor<T> {
 public:
  StepperMotorLinux(uint8_t id = 0) : gpio(arduino::RPI.getGPIO()) {
    this->setID(id);
  }

  /// Set the step (must be a hardware PWM pin), direction and enable pins.
  /// The enable pin is active LOW, as on common A4988/DRV8825 drivers.
  void setPins(int stepPin, int dirPin, int enablePin = -1) {
    pinStep = stepPin;
    pinDir = dirPin;
    pinEnable = enablePin;
    is_pins_set = true;
  }

  /// Steps per second at 100 percent speed.
  void setMaxSpeed(float stepsPerSec) { maxSpeedHz = stepsPerSec; }

  /// Not supported on Linux: the PWM output has no acceleration ramp. The value
  /// is stored for API compatibility only.
  void setAccelerationMs(uint16_t ms) { accelerationMs = ms; }

  /// Steps per full revolution (informational, used by unit conversions).
  void setStepsPerRevolution(int steps) { stepsPerRevolution = steps; }

  /// Configure the pins. Returns false if the pins were not set.
  bool begin() override {
    if (!is_pins_set || gpio == nullptr) return false;
    gpio->pinMode(pinStep, OUTPUT);
    gpio->pinMode(pinDir, OUTPUT);
    if (pinEnable != -1) {
      gpio->pinMode(pinEnable, OUTPUT);
      gpio->digitalWrite(pinEnable, LOW);  // enable driver
    }
    return true;
  }

  /// Continuous speed in percent (-100 to 100). Positive is forward.
  bool setValuePercent(T percent) override {
    if (!is_pins_set || gpio == nullptr) return false;
    lastValuePercent = constrain(percent, -100.0f, 100.0f);
    if (lastValuePercent == 0) {
      gpio->noTone(pinStep);
      return true;
    }
    gpio->digitalWrite(pinDir, lastValuePercent > 0 ? HIGH : LOW);
    float hz = maxSpeedHz * fabs((float)lastValuePercent) / 100.0f;
    if (hz < 1.0f) hz = 1.0f;  // sysfs PWM cannot run below 1 Hz
    gpio->tone(pinStep, (unsigned int)hz);
    return true;
  }

  T getValuePercent() const override { return lastValuePercent; }

  /// Stop the step pulses.
  void end() override {
    if (gpio != nullptr && is_pins_set) gpio->noTone(pinStep);
    lastValuePercent = 0;
  }

  bool isPinsSet() const override { return is_pins_set; }

 protected:
  arduino::HardwareGPIO_RPI* gpio = nullptr;
  T lastValuePercent = 0.0f;
  int pinStep = -1;
  int pinDir = -1;
  int pinEnable = -1;
  bool is_pins_set = false;
  float maxSpeedHz = 1000;
  uint16_t accelerationMs = 0;
  int stepsPerRevolution = 200;
};

}  // namespace tinyrobotics

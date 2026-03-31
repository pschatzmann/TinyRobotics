
#pragma once

#include <Arduino.h>
#ifdef ESP32
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif

#include "Motor.h"

namespace tinyrobotics {

/**
 * @class BrushlessMotor
 * @ingroup motors
 * @brief Simple brushless DC motor (ESC) driver using Servo library for
 * Arduino/ESP32.
 *
 * This class provides an interface to control brushless DC motors (with ESCs)
 * using the Servo or ESP32Servo library. It allows attaching to a pin, setting
 * speed as a percentage, and detaching the servo.
 *
 * Main methods:
 * - setPin(pin): Defines the pin to which the ESC control signal is connected.
 * - begin(): Initialize and attach the servo to the configured pin.
 * - setSpeed(percent): Set the motor speed as a percentage (0 to 100).
 * - end(): Stop the motor and detach the servo.
 *
 * Usage notes:
 * - Call setPin() before begin().
 * - Use setSpeed() to control the motor speed. Use end() to stop and detach.
 * - The class maps speed percentage to servo pulse width (1000-2000us typical
 * for ESCs).
 *
 * Example usage:
 * @code
 * BrushlessMotor motor;
 * motor.setPin(9);
 * motor.begin();
 * motor.setSpeed(50); // 50% throttle
 * motor.end();
 * @endcode
 */
class BrushlessMotor : public Motor {
 public:
  BrushlessMotor(uint8_t id = 0) { setID(id); }

  /** Attach the servo to a pin */
  void setPin(int pin) { this->pin = pin; }

  bool begin() {
    if (pin == -1) return false;
    servo.attach(pin);
    return true;
  }

  // Set speed as percentaage (0 to 100)
  bool setSpeed(int8_t percent) {
    if (!servo.attached()) return false;
    percent = constrain(percent, -100, 100);
    int angle = map(percent, 0, 100, 1000, 2000);  // Map to servo angle
    servo.write(angle);
    return true;
  }

  void end() override {
    setSpeed(0);
    servo.detach();
  }

  bool isPinsSet() const override { return pin != -1; }

 protected:
  Servo servo;
  int pin = -1;
  bool is_pin_assigned = false;
};

}  // namespace tinyrobotics
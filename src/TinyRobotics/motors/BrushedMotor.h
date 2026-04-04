#include <algorithm>
#pragma once
#include <Arduino.h>

#include "Motor.h"

namespace tinyrobotics {

/**
 * @class BrushedMotor
 * @ingroup motors
 * @brief High-level H-Bridge motor driver class for bidirectional DC motor
 * control.
 *
 * This class provides an easy interface for controlling a DC motor using a
 * standard H-Bridge driver (e.g., L298N, L293D, TB6612FNG). It supports:
 *   - Forward, reverse, and stop (brake) control
 *   - PWM speed control (signed, -255 to 255 or limited by setConstraints)
 *   - Speed setting in percent (-100 to 100)
 *   - Optional PWM frequency configuration (if supported by platform)
 *   - Speed limiting for safety or motor tuning
 *
 * Usage Example:
 * @code
 * HBridge motor(5, 6, 9); // IN1=5, IN2=6, PWM=9
 * motor.setConstraints(-200, 200); // Limit speed range
 * motor.setSpeed(50);        // Half speed forward
 * motor.setSpeed(-50);       // Half speed reverse
 * motor.stop();               // Brake
 * @endcode
 *
 * @note If your platform does not support analogWriteFreq, the default PWM
 * frequency will be used.
 */
class BrushedMotor : public Motor {
 public:
  /**
   * @brief Empty constructor for late pin assignment.
   */
  BrushedMotor(uint8_t id = 0)  { setID(id); }

  /**
   * @brief Set the pins for the HBridge after construction.
   */
  void setPins(int pin1, int pin2, int pwmPin = -1, int pwmFreq = 20000) {
    pinIn1 = pin1;
    pinIn2 = pin2;
    pinPWM = pwmPin;
    this->pwmFreq = pwmFreq;
  }

  bool begin() {
    if (!isPinsSet()) return false;

    pinMode(pinIn1, OUTPUT);
    pinMode(pinIn2, OUTPUT);

    pinMode(pinPWM, OUTPUT);
#ifdef SUPPORTS_ANALOG_WRITE_FREQ
    if (pwmFreq > 0) analogWriteFreq(pinPWM, pwmFreq);
#endif
    stop();
    return true;
  }

  /// Constrain the speed range to a subset of -255..255. This can be used to
  /// limit the maximum speed for safety or to match the characteristics of a
  /// specific motor.
  void setConstraints(int minSpeed, int maxSpeed) {
    minSpeed = std::max(minSpeed, -255);
    maxSpeed = std::min(maxSpeed, 255);
  }

  /// Invert the direction logic
  void setReverse(bool reverse) { is_reverse = reverse; }

  /**
   * Set motor speed as a percentage (-100 to 100).
   * @param percent Speed percentage: -100 (full reverse) to 100 (full forward)
   */
  void setSpeed(int8_t percent) {
    percent = constrain(percent, -100, 100);
    int pwmValue = map(percent, -100, 100, minSpeed, maxSpeed);
    setSpeedValue(pwmValue);
  }

  /** Stop the motor (brake) */
  void end() override { stop(); }

  /// Check if the control pins have been set (i.e., not -1).
  bool isPinsSet() const {
    return pinIn1 != -1 && pinIn2 != -1 && pinPWM != -1;
  }

 protected:
  int pinIn1 = -1;
  int pinIn2 = -1;
  int pinPWM = -1;
  int minSpeed = -255;
  int maxSpeed = 255;
  int pwmFreq = 20000;  // Default PWM frequency (20 kHz)
  bool is_reverse = false;

  bool stop() {
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, LOW);
    if (pinPWM != -1) analogWrite(pinPWM, 0);
    return true;
  }

  /** Set motor speed and direction. Speed: -255 (full reverse) to 255 (full
   * forward) */
  void setSpeedValue(int speed) {
    speed = constrain(speed, minSpeed, maxSpeed);
    if (speed > 0) {
      digitalWrite(pinIn1, is_reverse ? LOW : HIGH);
      digitalWrite(pinIn2, is_reverse ? HIGH : LOW);
      if (pinPWM != -1) analogWrite(pinPWM, speed);
    } else if (speed < 0) {
      digitalWrite(pinIn1, is_reverse ? HIGH : LOW);
      digitalWrite(pinIn2, is_reverse ? LOW : HIGH);
      if (pinPWM != -1) analogWrite(pinPWM, -speed);
    } else if (speed == 0) {
      stop();
    }
  }
};

}  // namespace tinyrobotics

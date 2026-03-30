#pragma once
#include <Arduino.h>

#include "Motor.h"

namespace tinyrobotics {

/**
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
 * motor.setSpeed(128);        // Half speed forward
 * motor.setSpeed(-255);       // Full speed reverse
 * motor.setSpeedPercent(50);  // 50% forward
 * motor.stop();               // Brake
 * @endcode
 *
 * @param pin1 First control pin (IN1)
 * @param pin2 Second control pin (IN2)
 * @param pwmPin PWM pin for speed control (optional, -1 if not used)
 * @param pwmFreq PWM frequency in Hz (optional, default 20kHz if supported)
 *
 * @note If your platform does not support analogWriteFreq, the default PWM
 * frequency will be used.
 */
class HBridge : public Motor {
 public:
  /**
   * @brief Empty constructor for late pin assignment.
   */
  HBridge() : in1(-1), in2(-1), pwm(-1) {}

  /**
   * @param pin1 First control pin (IN1)
   * @param pin2 Second control pin (IN2)
   * @param pwmPin PWM pin for speed control (optional, -1 if not used)
   */
  HBridge(int pin1, int pin2, int pwmPin = -1, int pwmFreq = 20000) {
    setPins(pin1, pin2, pwmPin, pwmFreq);
  }

  /**
   * @brief Set the pins for the HBridge after construction.
   */
  void setPins(int pin1, int pin2, int pwmPin = -1, int pwmFreq = 20000) {
    in1 = pin1;
    in2 = pin2;
    pwm = pwmPin;
    if (in1 != -1) pinMode(in1, OUTPUT);
    if (in2 != -1) pinMode(in2, OUTPUT);
#ifdef SUPPORTS_ANALOG_WRITE_FREQ
    if (pwm != -1) analogWriteFreq(pwm, pwmFreq);
#endif
    if (pwm != -1) pinMode(pwm, OUTPUT);
    stop();
  }

  /**
   * Set motor speed as a percentage (-100 to 100).
   * @param percent Speed percentage: -100 (full reverse) to 100 (full forward)
   */
  void setSpeedPercent(int percent) {
    percent = constrain(percent, -100, 100);
    int pwmValue = map(percent, -100, 100, minSpeed, maxSpeed);
    setSpeed(pwmValue);
  }

  /** Set motor speed and direction. Speed: -255 (full reverse) to 255 (full
   * forward) */
  void setSpeed(int speed) {
    speed = constrain(speed, minSpeed, maxSpeed);
    if (speed > 0) {
      digitalWrite(in1, is_reverse ? LOW : HIGH);
      digitalWrite(in2, is_reverse ? HIGH : LOW);
      if (pwm != -1) analogWrite(pwm, speed);
    } else if (speed < 0) {
      digitalWrite(in1, is_reverse ? HIGH : LOW);
      digitalWrite(in2, is_reverse ? LOW : HIGH);
      if (pwm != -1) analogWrite(pwm, -speed);
    } else {
      stop();
    }
  }

  /** Stop the motor (brake) */
  void end() override { stop(); }

  /// Constrain the speed range to a subset of -255..255. This can be used to
  /// limit the maximum speed for safety or to match the characteristics of a
  /// specific motor.
  void setConstraints(int minSpeed, int maxSpeed) {
    minSpeed = max(minSpeed, -255);
    maxSpeed = min(maxSpeed, 255);
  }

  /// Invert the direction logic
  void setReverse(bool reverse) { is_reverse = reverse; }

  /// Check if the control pins have been set (i.e., not -1).
  bool isPinsSet() const { return in1 != -1 && in2 != -1; }

 protected:
  int in1, in2, pwm;
  int minSpeed = -255;
  int maxSpeed = 255;
  bool is_reverse = false;

  bool stop() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    if (pwm != -1) analogWrite(pwm, 0);
    return true;
  }
};

}  // namespace tinyrobotics

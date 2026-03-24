#pragma once

#include "motors/HBridge.h"
#include "motors/Servo.h"

namespace tinyrobotics {

/**
 * @brief Motor boat with 1 drive motor and 1 rudder servo.
 *
 * This class abstracts a simple motor boat:
 *  - 1 drive motor (HBridge)
 *  - 1 rudder servo (ServoMotor)
 *
 * Usage Example:
 * @code
 * tinyrobotics::MotorBoat boat;
 * boat.setPins(5, 6, 9, 10); // in1, in2, pwm, rudderPin
 * boat.setThrottle(70);      // 70% throttle
 * boat.setRudder(20);        // 20 degrees left
 * boat.stop();               // brake
 * @endcode
 */

class MotorBoat {
 public:
  MotorBoat() = default;

  /**
   * @brief Set the pins for the drive motor and rudder servo.
   * @param in1 HBridge IN1
   * @param in2 HBridge IN2
   * @param pwm HBridge PWM
   * @param rudderPin Servo pin for rudder
   */
  void setPins(int in1, int in2, int pwm, int rudderPin) {
    motor_.setPins(in1, in2, pwm);
    rudder_.attach(rudderPin);
  }

  /**
   * @brief Set throttle (percent, -100 to 100). Positive = forward.
   */
  void setThrottle(int percent) { motor_.setSpeedPercent(percent); }

  /**
   * @brief Set rudder angle (degrees, left positive, right negative).
   */
  void setRudder(int angle) { rudder_.setAngle(angle); }

  void reset() {
    motor_.setSpeedPercent(0);  // stop motor
    rudder_.setAngle(0);        // center rudder
  }

 protected:
  HBridge motor_;
  ServoMotor rudder_;
};

}  // namespace tinyrobotics

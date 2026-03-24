#pragma once

#include "motors/HBridge.h"
#include "motors/Servo.h"

/**
 * @brief Car with Ackerman steering and single drive motor.
 *
 * This class abstracts a simple car:
 *  - 1 drive motor (HBridge)
 *  - 1 steering servo (ServoMotor)
 *
 * Usage Example:
 * @code
 * tinyrobotics::CarAckerman car;
 * car.setPins(5, 6, 9, 10); // in1, in2, pwm, steeringPin
 * car.setSpeed(60);         // 60% forward
 * car.setSteering(30);      // 30 degrees left
 * car.end();               // brake
 * @endcode
 */
namespace tinyrobotics {

class CarAckerman {
 public:
  CarAckerman() = default;

  /**
   * @brief Set the pins for the drive motor and steering servo.
   * @param in1 HBridge IN1
   * @param in2 HBridge IN2
   * @param pwm HBridge PWM
   * @param steeringPin Servo pin for steering
   */
  void setPins(int in1, int in2, int pwm, int steeringPin) {
    motor_.setPins(in1, in2, pwm);
    steering_.attach(steeringPin);
  }

  /**
   * @brief Set drive speed (percent, -100 to 100). Positive = forward.
   */
  void setSpeed(int percent) { motor_.setSpeedPercent(percent); }

  /**
   * @brief Set steering angle (degrees, left positive, right negative).
   */
  void setSteeringAngle(int angle) { steering_.setAngle(angle); }

  /** Stop the car (brake motor) */
  void end() {
    motor_.stop();
    steering_.setAngle(0);
  }

 protected:
  HBridge motor_;
  ServoMotor steering_;
};

}  // namespace tinyrobotics

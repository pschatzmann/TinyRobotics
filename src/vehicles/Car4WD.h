#pragma once

#include "motors/HBridge.h"

/**
 * @brief 4WD car model with direction control by adjusting the speed of 4
 * motors.
 *
 * This class abstracts a simple 4-wheel-drive car:
 *  - 4 motors (front left, front right, rear left, rear right) via HBridge
 *  - No steering servo: direction is controlled by varying motor speeds
 *
 * Usage Example:
 * @code
 * tinyrobotics::Car4WD car(m1_in1, m1_in2, m1_pwm, m2_in1, m2_in2, m2_pwm,
 *                         m3_in1, m3_in2, m3_pwm, m4_in1, m4_in2, m4_pwm);
 * car.setSpeed(60);      // 60% forward
 * car.setTurn(30);       // turn right by slowing left motors
 * car.stop();            // brake all motors
 * @endcode
 */
namespace tinyrobotics {

class Car4WD {
 public:
  Car4WD() : speed_(0), turn_(0) {}

  /**
   * @brief Set the pins for a specific motor (0=front left, 1=front right,
   * 2=rear left, 3=rear right)
   */
  void setPins(int motor, int in1, int in2, int pwm) {
    switch (motor) {
      case 0:
        m1_.setPins(in1, in2, pwm);
        break;
      case 1:
        m2_.setPins(in1, in2, pwm);
        break;
      case 2:
        m3_.setPins(in1, in2, pwm);
        break;
      case 3:
        m4_.setPins(in1, in2, pwm);
        break;
      default:
        break;
    }
  }

  /**
   * @brief Set forward/reverse speed for all motors (percent, -100 to 100).
   * Positive = forward, negative = reverse.
   */
  void setSpeed(int percent) {
    speed_ = constrain(percent, -100, 100);
    updateMotors();
  }

  /**
   * @brief Set turn (percent, -100 to 100). Positive = right, negative = left.
   * This slows down the motors on one side to turn the car.
   */
  void setTurn(int percent) {
    turn_ = constrain(percent, -100, 100);
    updateMotors();
  }

  /** Stop all motors */
  void stop() {
    m1_.stop();
    m2_.stop();
    m3_.stop();
    m4_.stop();
  }

 private:
  HBridge m1_;
  HBridge m2_;
  HBridge m3_;
  HBridge m4_;
  int speed_, turn_;

  /**
   * @brief Update all motors based on speed and turn.
   *
   * - m1: front left
   * - m2: front right
   * - m3: rear left
   * - m4: rear right
   *
   * To turn right, left motors go slower, right motors go faster (and vice
   * versa).
   */
  void updateMotors() {
    int left = speed_ - turn_;
    int right = speed_ + turn_;
    m1_.setSpeedPercent(constrain(left, -100, 100));
    m3_.setSpeedPercent(constrain(left, -100, 100));
    m2_.setSpeedPercent(constrain(right, -100, 100));
    m4_.setSpeedPercent(constrain(right, -100, 100));
  }
};

}  // namespace tinyrobotics

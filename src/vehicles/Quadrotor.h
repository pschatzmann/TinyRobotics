#pragma once

#include "motors/HBridge.h"

namespace tinyrobotics {

/// @brief Enum for quadrotor motor indexing
enum QuadrotorMotorNo {
  FRONT_LEFT = 0,
  FRONT_RIGHT = 1,
  REAR_LEFT = 2,
  REAR_RIGHT = 3
};

/**
 * @brief Simple quadrotor (quadcopter) model with 4-motor control.
 *
 * This class abstracts the control of a basic quadcopter:
 *  - 4 motors (front left, front right, rear left, rear right) via HBridge
 *  - Throttle, roll, pitch, and yaw control
 *
 *
 * Usage Example (with setPins):
 * @code
 * tinyrobotics::Quadrotor quad;
 * quad.setPins(FRONT_LEFT, m1_in1, m1_in2, m1_pwm); // front left
 * quad.setPins(FRONT_RIGHT, m2_in1, m2_in2, m2_pwm); // front right
 * quad.setPins(REAR_LEFT, m3_in1, m3_in2, m3_pwm); // rear left
 * quad.setPins(REAR_RIGHT, m4_in1, m4_in2, m4_pwm); // rear right
 * quad.setThrottle(60); // 60% throttle
 * quad.setRoll(10);     // roll right
 * quad.setPitch(-5);    // pitch down
 * quad.setYaw(15);      // yaw right
 * @endcode
 */

class Quadrotor {
 public:
  Quadrotor() = default;

  /**
   * @brief Set the pins for a specific motor (0=front left, 1=front right,
   * 2=rear left, 3=rear right)
   */
  void setPins(QuadrotorMotorNo motor, int in1, int in2, int pwm) {
    motors_[motor].setPins(in1, in2, pwm);
  }

  /** Set throttle (0-100%) for all motors */
  void setThrottle(int percent) {
    throttle_ = constrain(percent, 0, 100);
    updateMotors();
  }

  /** Set roll (-100 to 100): positive = right, negative = left */
  void setRoll(int percent) {
    roll_ = constrain(percent, -100, 100);
    updateMotors();
  }

  /** Set pitch (-100 to 100): positive = forward, negative = backward */
  void setPitch(int percent) {
    pitch_ = constrain(percent, -100, 100);
    updateMotors();
  }

  /** Set yaw (-100 to 100): positive = right, negative = left */
  void setYaw(int percent) {
    yaw_ = constrain(percent, -100, 100);
    updateMotors();
  }

  /** Stop all motors */
  void stop() {
    for (int i = 0; i < 4; ++i) {
      motors_[i].stop();
    }
  }

 private:
  HBridge motors_[4];
  int throttle_ = 0;
  int roll_ = 0;
  int pitch_ = 0;
  int yaw_ = 0;

  /**
   * @brief Update all motors based on throttle, roll, pitch, and yaw.
   *
   * This is a simple mixer for an X-configuration quadrotor:
   *  - m1: front left
   *  - m2: front right
   *  - m3: rear left
   *  - m4: rear right
   *
   * Each control input is in percent (-100 to 100).
   */
  void updateMotors() {
    // Basic mixing: throttle + pitch/roll/yaw
    int m[4];
    m[0] = throttle_ + pitch_ + roll_ - yaw_;
    m[1] = throttle_ + pitch_ - roll_ + yaw_;
    m[2] = throttle_ - pitch_ + roll_ + yaw_;
    m[3] = throttle_ - pitch_ - roll_ - yaw_;
    for (int i = 0; i < 4; ++i) {
      motors_[i].setSpeedPercent(constrain(m[i], 0, 100));
    }
  }
};

}  // namespace tinyrobotics

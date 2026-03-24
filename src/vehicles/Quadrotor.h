#pragma once

#include "motors/HBridge.h"

/**
 * @brief Simple quadrotor (quadcopter) model with 4-motor control.
 *
 * This class abstracts the control of a basic quadcopter:
 *  - 4 motors (front left, front right, rear left, rear right) via HBridge
 *  - Throttle, roll, pitch, and yaw control
 *
 * Usage Example:
 * @code
 * tinyrobotics::Quadrotor quad(m1_in1, m1_in2, m1_pwm, m2_in1, m2_in2, m2_pwm,
 *                              m3_in1, m3_in2, m3_pwm, m4_in1, m4_in2, m4_pwm);
 * quad.setThrottle(60); // 60% throttle
 * quad.setRoll(10);     // roll right
 * quad.setPitch(-5);    // pitch down
 * quad.setYaw(15);      // yaw right
 * @endcode
 */
namespace tinyrobotics {

class Quadrotor {
 public:
  Quadrotor(int m1_in1, int m1_in2, int m1_pwm, int m2_in1, int m2_in2,
            int m2_pwm, int m3_in1, int m3_in2, int m3_pwm, int m4_in1,
            int m4_in2, int m4_pwm)
      : m1_(m1_in1, m1_in2, m1_pwm),
        m2_(m2_in1, m2_in2, m2_pwm),
        m3_(m3_in1, m3_in2, m3_pwm),
        m4_(m4_in1, m4_in2, m4_pwm),
        throttle_(0),
        roll_(0),
        pitch_(0),
        yaw_(0) {}

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
    m1_.stop();
    m2_.stop();
    m3_.stop();
    m4_.stop();
  }

 private:
  HBridge m1_, m2_, m3_, m4_;
  int throttle_, roll_, pitch_, yaw_;

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
    int m1 = throttle_ + pitch_ + roll_ - yaw_;
    int m2 = throttle_ + pitch_ - roll_ + yaw_;
    int m3 = throttle_ - pitch_ + roll_ + yaw_;
    int m4 = throttle_ - pitch_ - roll_ - yaw_;
    m1_.setSpeedPercent(constrain(m1, 0, 100));
    m2_.setSpeedPercent(constrain(m2, 0, 100));
    m3_.setSpeedPercent(constrain(m3, 0, 100));
    m4_.setSpeedPercent(constrain(m4, 0, 100));
  }
};

}  // namespace tinyrobotics

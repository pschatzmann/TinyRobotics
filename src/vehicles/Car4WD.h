#pragma once

#include "Vehicle.h"
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
 * car.end();            // brake all motors
 * @endcode
 */
namespace tinyrobotics {

class Car4WD : public Vehicle {
 public:
  Car4WD() : speed_(0), turn_(0) {}

  /**
   * @brief Set the pins for a specific motor (0=front left, 1=front right,
   * 2=rear left, 3=rear right)
   */
  void setPins(int motor, int in1, int in2, int pwm) {
    if (motor >= 0 && motor < 4) {
      motors_[motor].setPins(in1, in2, pwm);
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

  /**
   * @brief Stop all motors and reset speed and turn state.
   */
  void end() override {
    for (int i = 0; i < 4; ++i) {
      motors_[i].end();
    }
    speed_ = 0;
    turn_ = 0;
  }

  /**
   * @brief Set a calibration gain for a specific motor (default 1.0).
   * @param motor Motor index (0=front left, 1=front right, 2=rear left, 3=rear
   * right)
   * @param gain  Gain factor (e.g., 1.05 for +5% output)
   */
  void setMotorGain(int motor, float gain) {
    if (motor >= 0 && motor < 4) {
      motorGain_[motor] = gain;
    }
  }

  bool isPinsSet() const {
    for (int i = 0; i < 4; ++i) {
      if (!motors_[i].isPinsSet()) return false;
    }
    return true;
  }

 protected:
  HBridge motors_[4];
  float motorGain_[4] = {1.0f, 1.0f, 1.0f, 1.0f};
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
    int leftCal = static_cast<int>(left * motorGain_[0]);
    int rightCal = static_cast<int>(right * motorGain_[1]);
    int rearLeftCal = static_cast<int>(left * motorGain_[2]);
    int rearRightCal = static_cast<int>(right * motorGain_[3]);
    motors_[0].setSpeedPercent(constrain(leftCal, -100, 100));  // front left
    motors_[2].setSpeedPercent(constrain(rearLeftCal, -100, 100));  // rear left
    motors_[1].setSpeedPercent(constrain(rightCal, -100, 100));  // front right
    motors_[3].setSpeedPercent(
        constrain(rearRightCal, -100, 100));  // rear right
  }
};

}  // namespace tinyrobotics

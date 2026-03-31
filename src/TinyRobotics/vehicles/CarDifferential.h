#pragma once

#include "TinyRobotics/motors/Motors.h"
#include "Vehicle.h"

namespace tinyrobotics {

/**
 * @class CarDifferential
 * @ingroup vehicles
 * @brief  Car model with differential drive. The direction is controlled by
 * adjusting the speed of the motors. Even motors are on the left side, odd
 * motors are on the right side.
 *
 * This class abstracts a simple N-wheel-drive car:
 *  - N motors (e.g. front left, front right, rear left, rear right) via HBridge
 *  - No steering servo: direction is controlled by varying motor speeds
 *
 * Usage Example:
 * @code
 * CarDifferentialWD<4> car;
 * car.setPins(0, 2, 3, 4); // motor 0 (front left): in1=2, in2=3, pwm=4
 * car.setPins(1, 5, 6, 7); // motor 1 (front right): in1=5, in2=6, pwm=7
 * car.setPins(2, 8, 9, 10); // motor 2 (rear left): in1=8, in2=9, pwm=10
 * car.setPins(3, 11, 12, 13); // motor 3 (rear right): in1=11, in2=12, pwm=13
 * car.setSpeed(60);      // 60% forward
 * car.setTurn(30);       // turn right by slowing left motors
 * car.end();            // brake all motors
 * @endcode
 */

template <size_t N = 4, typename BrushedMT = BrushedMotor>
class CarDifferential : public Vehicle {
 public:
  CarDifferential() : speed_(0), turn_(0) {}

  /**
   * @brief Set the pins for a specific motor (0=front left, 1=front right,
   * 2=rear left, 3=rear right)
   */
  void setPins(int motor, int in1, int in2, int pwm) {
    if (motor >= 0 && motor < 4) {
      motors_[motor].setPins(in1, in2, pwm);
      motors_[motor].setID((uint8_t)motor);
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
  void setSteeringAgle(int angle) {
    int percent = map(angle, -45, 45, -100, 100);  // Map angle to turn percent
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

  bool onMessage(const Message<float>& msg) override {
    if (!isValidMessageSource(msg.source)) return false;  
    switch (msg.content) {
      case MessageContent::Throttle:
        if (msg.unit != Unit::Percent) return false;
        setSpeed(static_cast<int>(msg.value));
        return true;
      case MessageContent::SteeringAngle:
        float angle = msg.value;
        if (!toAngleDegree(angle, msg.unit, angle))
          return false;  // Invalid unit
        this->setSteeringAgle(angle);
        return true;
      default:
        return false;  // Unhandled message content
    }
  }

  std::vector<MessageContent> getControls() const override {
    return {MessageContent::Throttle, MessageContent::SteeringAngle};
  }

 protected:
  BrushedMT motors_[N];
  float motorGain_[N] = {1.0f, 1.0f, 1.0f, 1.0f};
  float speed_, turn_;

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
    float speed[4];
    for (int i = 0; i < 4; ++i) {
      speed[i] = (i % 2 == 0) ? left * motorGain_[i] : right * motorGain_[i];
      speed[i] = constrain(speed[i], -100, 100);
    }

    // update speed
    for (int i = 0; i < 4; ++i) {
      motors_[i].setSpeedPercent(speed[i]);
    }

    // publish motor speeds as messages for telemetry
    for (int i = 0; i < 4; ++i) {
      Message<float> msg(MessageContent::MotorSpeed, speed[i], Unit::Percent);
      msg.source = MessageOrigin::Motor;
      msg.source_id = i;  // Motor index
      sendMessage(msg);
    }
  }
};

}  // namespace tinyrobotics

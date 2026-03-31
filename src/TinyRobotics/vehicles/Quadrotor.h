
#pragma once

#include "TinyRobotics/motors/Motors.h"
#include "Vehicle.h"

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
 * Usage Example (with setPins):
 * @code
 * Quadrotor quad;
 * quad.setPins(FRONT_LEFT, m1_in1, m1_in2, m1_pwm); // front left
 * quad.setPins(FRONT_RIGHT, m2_in1, m2_in2, m2_pwm); // front right
 * quad.setPins(REAR_LEFT, m3_in1, m3_in2, m3_pwm); // rear left
 * quad.setPins(REAR_RIGHT, m4_in1, m4_in2, m4_pwm); // rear right
 * quad.setThrottle(60); // 60% throttle
 * quad.setRoll(10);     // roll right
 * quad.setPitch(-5);    // pitch down
 * quad.setYaw(15);      // yaw right
 * @endcode
 * @ingroup vehicles
 */

template <typename MotorT = BrushedMotor>
class Quadrotor : public Vehicle {
 public:
  Quadrotor() = default;

  /**
   * @brief Set the pins for a specific motor (0=front left, 1=front right,
   * 2=rear left, 3=rear right)
   */
  void setPins(QuadrotorMotorNo motor, int in1, int in2, int pwm) {
    motors_[motor].setPins(in1, in2, pwm);
    motors_[motor].setID((uint8_t)motor);
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

  /** Stop all motors and reset state */
  void end() {
    for (int i = 0; i < 4; ++i) {
      motors_[i].end();
    }
    throttle_ = 0;
    roll_ = 0;
    pitch_ = 0;
    yaw_ = 0;
  }

  /**
   * @brief Set a calibration gain for a specific motor (default 1.0).
   * @param motor Motor index (QuadrotorMotorNo)
   * @param gain  Gain factor (e.g., 1.05 for +5% output)
   */
  void setMotorGain(QuadrotorMotorNo motor, float gain) {
    motorGain_[motor] = gain;
  }

  bool isPinsSet() const {
    for (int i = 0; i < 4; ++i) {
      if (!motors_[i].isPinsSet()) return false;
    }
    return true;
  }

  bool onMessage(const Message<float>& msg) override {
    float angle;
    if (msg.source != MessageOrigin::RemoteControl)
      return false;  // Only handle RC messages
    switch (msg.content) {
      case MessageContent::Throttle:
        if (msg.unit != Unit::Percent) return false;
        setThrottle(static_cast<int>(msg.value));
        return true;
      case MessageContent::Pitch:
        angle = msg.value;
        if (!toAngleDegree(angle, msg.unit, angle))
          return false;  // Invalid unit
        setPitch(angle);
        return true;
      case MessageContent::Roll:
        angle = msg.value;
        if (!toAngleDegree(angle, msg.unit, angle))
          return false;  // Invalid unit
        setRoll(angle);
        return true;
      case MessageContent::Yaw:
        angle = msg.value;
        if (!toAngleDegree(angle, msg.unit, angle))
          return false;  // Invalid unit
        setYaw(angle);
        return true;
      default:
        return false;  // Unhandled message content
    }
  }

  std::vector<MessageContent> getControls() const override {
    return {MessageContent::Throttle, MessageContent::Pitch, MessageContent::Roll,
            MessageContent::Yaw};
  }

 protected:
  MotorT motors_[4];
  float motorGain_[4] = {1.0f, 1.0f, 1.0f, 1.0f};
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
      int calibrated = static_cast<int>(m[i] * motorGain_[i]);
      motors_[i].setSpeed(constrain(calibrated, 0, 100));
    }
    // publish motor speeds as messages for telemetry
    for (int i = 0; i < 4; ++i) {
      Message<float> msg(MessageContent::MotorSpeed, m[i], Unit::Percent);
      msg.source = MessageOrigin::Motor;
      msg.source_id = i;  // Motor index
      sendMessage(msg);
    }
  }
};

}  // namespace tinyrobotics

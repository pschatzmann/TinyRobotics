#pragma once

#include "TinyRobotics/motors/Motors.h"
#include "Vehicle.h"

namespace tinyrobotics {

/**
 * @class CarAckerman
 * @ingroup vehicles
 * @brief Car with Ackerman steering and single drive motor.
 *
 * This class abstracts a simple car:
 *  - 1 drive motor (HBridge)
 *  - 1 steering servo (ServoMotor)
 *
 * Usage Example:
 * @code
 * CarAckerman car;
 * car.setPins(5, 6, 9); // in1, in2, steeringPin
 * car.setSpeed(60);         // 60% forward
 * car.setSteering(30);      // 30 degrees left
 * car.end();               // brake
 * @endcode
 */

template <typename MotorMT = BrushedMotor<float>, typename ServoMT = ServoMotor<float>>
class CarAckerman : public Vehicle {
 public:
  CarAckerman() = default;

  /**
   * @brief Set the pins for the drive motor and steering servo.
   * @param in1 HBridge IN1
   * @param in2 HBridge IN2
   * @param pwm HBridge PWM
   * @param steeringPin Servo pin for steering
   */
  void setPins(int in1, int in2, int steeringPin) {
    motor_.setPins(in1, in2);
    steering_.setPin(steeringPin);
  }

  /**
   * @brief Set drive speed (percent, -100 to 100). Positive = forward.
   */
  void setSpeed(float percent) {
    speed_ = constrain(percent * getSpeedFactor(), -100.0f, 100.0f);
    motor_.setValuePercent(speed_);
    // publish speed as message for telemetry
    Message<float> msg(MessageContent::MotorSpeed, percent, Unit::Percent);
    msg.origin = MessageOrigin::Motor;
    sendMessage(msg);
  }

  /**
   * @brief Set steering angle (degrees, left positive, right negative).
   */
  void setSteeringAngle(float angle) {
    angleDeg_ = angle;
    steering_.setAngle(angle);
    // publish steering angle as message for telemetry
    Message<float> msg(MessageContent::SteeringAngle, angle, Unit::AngleDegree);
    msg.origin = MessageOrigin::Servo;
    sendMessage(msg);
  }

  void setSteeringAngle(Angle angle){
    setSteeringAngle(static_cast<int>(angle.getValue(AngleUnit::DEG)));
  }

  Angle getSteeringAngle() const { return Angle(angleDeg_,AngleUnit::DEG); }

  float getSpeed() const { return speed_; }

  /** Stop the car (brake motor) */
  void end() {
    setSpeed(0);
    setSteeringAngle(0);
    motor_.end();
    steering_.end();
  }

  bool isPinsSet() const { return motor_.isPinsSet() && steering_.isPinsSet(); }

  bool onMessage(const Message<float>& msg) override {
    float angle;
    if (!isValidMessageSource(msg.origin)) return false;
    switch (msg.content) {
      case MessageContent::Throttle:
        if (msg.unit != Unit::Percent) return false;
        setSpeed(static_cast<int>(msg.value));
        return true;
      case MessageContent::SteeringAngle:
        angle = msg.value;
        if (!toAngleDegree(angle, msg.unit, angle))
          return false;  // Invalid unit
        setSteeringAngle(angle);
        return true;
      default:
        return false;  // Unhandled message content
    }
  }

  std::vector<MessageContent> getControls() const override {
    return {MessageContent::Throttle, MessageContent::SteeringAngle};
  }

  MotorMT& getMotor() { return motor_; }
  ServoMT& getServo() { return steering_; }

 protected:
  MotorMT motor_;
  ServoMT steering_;
  float speed_ = 0;
  float angleDeg_ = 0;
};

}  // namespace tinyrobotics

#pragma once

#include "Vehicle.h"
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
  void setPins(int in1, int in2, int pwm, int steeringPin) {
    motor_.setPins(in1, in2, pwm);
    steering_.attach(steeringPin);
  }

  /**
   * @brief Set drive speed (percent, -100 to 100). Positive = forward.
   */
  void setSpeed(int percent) {
    motor_.setSpeedPercent(percent);
    // publish speed as message for telemetry
    Message<float> msg(MessageContent::MotorSpeed, percent, Unit::Percent);
    msg.source = MessgeSource::Motor;
    sendMessage(msg);
  }

  /**
   * @brief Set steering angle (degrees, left positive, right negative).
   */
  void setSteeringAngle(int angle) {
    steering_.setAngle(angle);
    // publish steering angle as message for telemetry
    Message<float> msg(MessageContent::SteeringAngle, angle, Unit::AngleDegree);
    msg.source = MessgeSource::Servo;
    sendMessage(msg);
  }

  /** Stop the car (brake motor) */
  void end() {
    setSpeed(0);
    setSteeringAngle(0);
    motor_.end();
    steering_.end();}

  bool isPinsSet() const { return motor_.isPinsSet() && steering_.isPinsSet(); }

  bool onMessage(const Message<float>& msg) override {
    float angle;
    if (msg.source != MessgeSource::RemoteControl) return false;  // Only handle RC messages
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

 protected:
  HBridge motor_;
  ServoMotor steering_;
};

}  // namespace tinyrobotics

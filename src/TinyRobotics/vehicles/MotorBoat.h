#pragma once

#include "TinyRobotics/motors/Motors.h"
#include "Vehicle.h"

namespace tinyrobotics {

/**
 * @class MotorBoat
 * @ingroup vehicles
 * @brief Motor boat with 1 drive motor and 1 rudder servo.
 *
 * This class abstracts a simple motor boat:
 *  - 1 drive motor (HBridge)
 *  - 1 rudder servo (ServoMotor)
 *
 * Usage Example:
 * @code
 * MotorBoat boat;
 * boat.setPins(5, 6, 9, 10); // in1, in2, pwm, rudderPin
 * boat.setThrottle(70);      // 70% throttle
 * boat.setRudder(20);        // 20 degrees left
 * boat.stop();               // brake
 * @endcode
 */

template <typename MotorMT = BrushedMotor<float>, typename ServoMT = ServoMotor<float>>
class MotorBoat : public Vehicle {
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
   * @brief Set the speed as a percentage (-100 to 100). Positive = forward.
   * This applies the speed scaling factor (speedFactor) before setting throttle.
   * @param percent Speed percentage (-100 to 100)
   */
  void setSpeed(float percent) {
    setThrottle(percent*getSpeedFactor());
  }

  /**
   * @brief Set throttle (percent, -100 to 100). Positive = forward.
   */
  void setThrottle(float percent) {
    motor_.setSpeedPercent(percent);
    // publish throttle as message for telemetry
    Message<float> msg(MessageContent::MotorSpeed, percent, Unit::Percent);
    msg.source = MessageOrigin::Motor;
    sendMessage(msg);
  }

  /**
   * @brief Set rudder angle (degrees, left positive, right negative).
   */
  void setRudder(float angle) {
    rudder_.setAngle(angle);
    // publish rudder angle as message for telemetry
    Message<float> msg(MessageContent::SteeringAngle, angle, Unit::AngleDegree);
    msg.source = MessageOrigin::Rudder;
    sendMessage(msg);
  }

  void setRudder(Angle angle){
    setRudder(static_cast<int>(angle.getValue(AngleUnit::DEG)));
  }

  void end() {
    motor_.setSpeedPercent(0);  // stop motor
    rudder_.setAngle(0);        // center rudder
  }

  bool isPinsSet() const { return motor_.isPinsSet() && rudder_.isPinsSet(); }

  bool onMessage(const Message<float>& msg) override {
    float angle;
    if (!isValidMessageSource(msg.source)) return false;  
    switch (msg.content) {
      case MessageContent::Throttle:
        if (msg.unit != Unit::Percent) return false;
        setThrottle(static_cast<int>(msg.value));
        return true;
      case MessageContent::SteeringAngle:
        angle = msg.value;
        if (!toAngleDegree(angle, msg.unit, angle))
          return false;  // Invalid unit
        setRudder(static_cast<int>(msg.value));
        return true;
      default:
        return false;  // Unhandled message content
    }
  }

  std::vector<MessageContent> getControls() const override {
    return {MessageContent::Throttle, MessageContent::SteeringAngle};
  }

  MotorMT& getMotor() { return motor_; }
  ServoMT& getServo() { return rudder_; }

 protected:
  MotorMT motor_;
  ServoMT rudder_;
};

}  // namespace tinyrobotics

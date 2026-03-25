#pragma once

#include "TinyRobotics/motors/HBridge.h"
#include "TinyRobotics/motors/Servo.h"
#include "Vehicle.h"

namespace tinyrobotics {

/**
 * @brief Motor boat with 1 drive motor and 1 rudder servo.
 *
 * This class abstracts a simple motor boat:
 *  - 1 drive motor (HBridge)
 *  - 1 rudder servo (ServoMotor)
 *
 * Usage Example:
 * @code
 * tinyrobotics::MotorBoat boat;
 * boat.setPins(5, 6, 9, 10); // in1, in2, pwm, rudderPin
 * boat.setThrottle(70);      // 70% throttle
 * boat.setRudder(20);        // 20 degrees left
 * boat.stop();               // brake
 * @endcode
 */

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
   * @brief Set throttle (percent, -100 to 100). Positive = forward.
   */
  void setThrottle(int percent) {
    motor_.setSpeedPercent(percent);
    // publish throttle as message for telemetry
    Message<float> msg(MessageContent::MotorSpeed, percent, Unit::Percent);
    msg.source = MessgeOrigin::Motor;
    sendMessage(msg);
  }

  /**
   * @brief Set rudder angle (degrees, left positive, right negative).
   */
  void setRudder(int angle) {
    rudder_.setAngle(angle);
    // publish rudder angle as message for telemetry
    Message<float> msg(MessageContent::SteeringAngle, angle, Unit::AngleDegree);
    msg.source = MessgeOrigin::Rudder;
    sendMessage(msg);
  }

  void end() {
    motor_.setSpeedPercent(0);  // stop motor
    rudder_.setAngle(0);        // center rudder
  }

  bool isPinsSet() const {
    return motor_.isPinsSet() && rudder_.isPinsSet();
  }

  bool onMessage(const Message<float>& msg) override {
    float angle;
    if (msg.source != MessgeOrigin::RemoteControl) return false;  // Only handle RC messages
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

 protected:
  HBridge motor_;
  ServoMotor rudder_;
};

}  // namespace tinyrobotics

#pragma once

#include "Vehicle.h"
#include "TinyRobotics/motors/Motors.h"

/**
 * @class AirPlane
 * @ingroup vehicles
 * @brief Simple fixed-wing airplane model with motor, rudder, elevator, and
 * aileron control.
 *
 * This class abstracts the control of a basic RC airplane:
 *  - Motor (throttle) via ServoMotor (ESC)
 *  - Rudder (yaw), Elevator (pitch), and Ailerons (roll) via ServoMotor
 *
 * Usage Example:
 * @code
 * AirPlane plane;
 * // setup pins
 * plane.setPinsMotor(5, 6, 9);      // HBridge pins
 * plane.setPinRudder(10);                         // rudder servo
 * plane.setPinElevator(11);                     // elevator servo
 * plane.setPinsAilerons(12, 13); // aileron servos
 * // control airplane
 * plane.setThrottle(70);               // 70% throttle
 * plane.setRudder(20);                 // 20 degrees left
 * plane.setElevator(-10);              // 10 degrees down
 * plane.setAilerons(15, -15);          // left up, right down
 * @endcode
 */
namespace tinyrobotics {

template <typename BrushedMT = BrushedMotor, typename ServoMT = ServoMotor>
class AirPlane : public Vehicle {
 public:
  AirPlane() = default;

  /** Attach the motor HBridge. */
  void setPinsMotor(int in1, int in2, int pwm) {
    motor_.setPins(in1, in2, pwm);
  }

  /** Attach the rudder servo. */
  void setPinRudder(int pin) { rudder_.attach(pin); }

  /** Attach the elevator servo. */
  void setPinElevator(int pin) { elevator_.attach(pin); }

  /** Attach the left aileron servo. */
  void setPinsAilerons(int leftPin, int rightPin) {
    aileronLeft_.attach(leftPin);
    aileronRight_.attach(rightPin);
  }

  /** Set throttle (0-100%) */
  void setThrottle(int percent) {
    percent = constrain(percent, 0, 100);
    motor_.setSpeedPercent(percent);

    // publish motor speed as message for telemetry
    Message<float> msg(MessageContent::MotorSpeed, percent, Unit::Percent);
    msg.source = MessageOrigin::Motor;
    sendMessage(msg);
  }

  /** Set rudder angle (degrees, -30 to 30 typical) */
  void setRudder(int angle) {
    rudder_.setAngle(angle);

    // publish rudder update as message for telemetry
    Message<float> msg(MessageContent::Yaw, angle, Unit::AngleDegree);
    msg.source = MessageOrigin::Rudder;
    sendMessage(msg);
  }

  /** Set elevator angle (degrees, -30 to 30 typical) */
  void setElevator(int angle) {
    elevator_.setAngle(angle);
    // publish elevator update as message for telemetry
    Message<float> msg(MessageContent::Pitch, angle, Unit::AngleDegree);
    msg.source = MessageOrigin::Elevator;
    sendMessage(msg);
  }

  /** Set aileron angles (degrees, left and right) */
  void setAilerons(int leftAngle, int rightAngle) {
    aileronLeft_.setAngle(leftAngle);
    aileronRight_.setAngle(rightAngle);

    // publish aileron update as message for telemetry
    Message<float> msgLeft(MessageContent::Roll, leftAngle, Unit::AngleDegree);
    msgLeft.source = MessageOrigin::Aileron;
    msgLeft.source_id = 0;  // Left aileron
    sendMessage(msgLeft);
    Message<float> msgRight(MessageContent::Roll, rightAngle,
                            Unit::AngleDegree);
    msgRight.source = MessageOrigin::Aileron;
    msgRight.source_id = 1;  // Right aileron
    sendMessage(msgRight);
  }

  /** Reset state of all controls */
  void end() {
    setThrottle(0);  
    setRudder(0);
    setElevator(0);
    setAilerons(0, 0);
  }

  bool isPinsSet() const {
    return motor_.isPinsSet() && rudder_.isPinsSet() && elevator_.isPinsSet() &&
           aileronLeft_.isPinsSet() && aileronRight_.isPinsSet();
  }

  /** Set pitch (degrees): positive = nose up, negative = nose down */
  void setPitch(int angle) { setElevator(angle); }

  /** Set roll (degrees): positive = left wing up, negative = right wing up */
  void setRoll(int angle) { setAilerons(angle, -angle); }

  /** Set yaw (degrees): positive = nose left, negative = nose right */
  void setYaw(int angle) { setRudder(angle); }

  bool onMessage(const Message<float>& msg) override {
    float angle;
    if (!isValidMessageSource(msg.source)) return false;  
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
  BrushedMT motor_;
  int rudderPin_, elevatorPin_, aileronLeftPin_, aileronRightPin_;
  ServoMT rudder_;
  ServoMT elevator_;
  ServoMT aileronLeft_;
  ServoMT aileronRight_;
};

}  // namespace tinyrobotics
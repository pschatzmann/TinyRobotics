#pragma once

#include "TinyRobotics/motors/Motors.h"
#include "Vehicle.h"

namespace tinyrobotics {

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

template <typename MotorMT = BrushedMotor<float>, typename ServoMT = ServoMotor<float>>
class AirPlane : public Vehicle {
 public:
  enum ControlSurface {
    Rudder = 0,
    Elevator = 1,
    AileronLeft = 2,
    AileronRight = 3,
    NumSurfaces = 4
  };

 public:
  AirPlane() = default;

  /** Attach the motor HBridge. */
  void setPinsMotor(int in1, int in2, int pwm) {
    motor_.setPins(in1, in2, pwm);
    motor_.setID(0);  // Motor ID 0
  }

  /** Attach the rudder servo. */
  void setPinRudder(int pin) {
    pins_[Rudder] = pin;
    servos_[Rudder].setPin(pin);
    servos_[Rudder].setID(static_cast<uint8_t>(Rudder));
  }

  /** Attach the elevator servo. */
  void setPinElevator(int pin) {
    pins_[Elevator] = pin;
    servos_[Elevator].setPin(pin);
    servos_[Elevator].setID(static_cast<uint8_t>(Elevator));
  }

  /** Attach the aileron servos. */
  void setPinsAilerons(int leftPin, int rightPin) {
    pins_[AileronLeft] = leftPin;
    pins_[AileronRight] = rightPin;
    servos_[AileronLeft].setPin(leftPin);
    servos_[AileronRight].setPin(rightPin);
    servos_[AileronLeft].setID(static_cast<uint8_t>(AileronLeft));
    servos_[AileronRight].setID(static_cast<uint8_t>(AileronRight));
  }

  void setPinServo(ControlSurface surface, int pin) {
    if (surface >= 0 && surface < NumSurfaces) {
      pins_[surface] = pin;
      servos_[surface].setPin(pin);
      servos_[surface].setID(static_cast<uint8_t>(surface));
    }
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
    servos_[Rudder].setAngle(angle);
    // publish rudder update as message for telemetry
    Message<float> msg(MessageContent::Yaw, angle, Unit::AngleDegree);
    msg.source = MessageOrigin::Rudder;
    sendMessage(msg);
  }

  /** Set elevator angle (degrees, -30 to 30 typical) */
  void setElevator(int angle) {
    servos_[Elevator].setAngle(angle);
    // publish elevator update as message for telemetry
    Message<float> msg(MessageContent::Pitch, angle, Unit::AngleDegree);
    msg.source = MessageOrigin::Elevator;
    sendMessage(msg);
  }

  /** Set aileron angles (degrees, left and right) */
  void setAilerons(int leftAngle, int rightAngle) {
    servos_[AileronLeft].setAngle(leftAngle);
    servos_[AileronRight].setAngle(rightAngle);
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
    if (!motor_.isPinsSet()) return false;
    for (const auto& s : servos_) {
      if (!s.isPinsSet()) return false;
    }
    return true;
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
    return {MessageContent::Throttle, MessageContent::Pitch,
            MessageContent::Roll, MessageContent::Yaw};
  }

  MotorMT& getMotor() { return motor_; }
  ServoMT& getServo(ControlSurface surface) { return servos_[surface]; }

 protected:
  MotorMT motor_;
  std::vector<int> pins_ = std::vector<int>(NumSurfaces, -1);
  std::vector<ServoMT> servos_ = std::vector<ServoMT>(NumSurfaces, ServoMT());
};

}  // namespace tinyrobotics
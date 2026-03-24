#pragma once

#include "motors/HBridge.h"
#include "motors/Servo.h"

/**
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

class AirPlane {
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
  }

  /** Set rudder angle (degrees, -30 to 30 typical) */
  void setRudder(int angle) { rudder_.setAngle(angle); }

  /** Set elevator angle (degrees, -30 to 30 typical) */
  void setElevator(int angle) { elevator_.setAngle(angle); }

  /** Set aileron angles (degrees, left and right) */
  void setAilerons(int leftAngle, int rightAngle) {
    aileronLeft_.setAngle(leftAngle);
    aileronRight_.setAngle(rightAngle);
  }

  /** Reset state of all controls */
  void end() {
    motor_.setSpeedPercent(0);  //
    rudder_.setAngle(0);
    elevator_.setAngle(0);
    aileronLeft_.setAngle(0);
    aileronRight_.setAngle(0);
  }

 protected:
  HBridge motor_;
  int rudderPin_, elevatorPin_, aileronLeftPin_, aileronRightPin_;
  ServoMotor rudder_;
  ServoMotor elevator_;
  ServoMotor aileronLeft_;
  ServoMotor aileronRight_;
};

/**
 * @brief High-level controller for AirPlane: maps pitch, roll, yaw, throttle to
 * actuators.
 *
 * Usage Example:
 * @code
 * tinyrobotics::AirPlane plane(...);
 * tinyrobotics::AirPlaneControl ctrl(plane);
 * ctrl.setThrottle(70);
 * ctrl.setPitch(-10);
 * ctrl.setRoll(15);
 * ctrl.setYaw(20);
 * @endcode
 */
class AirPlaneControl {
 public:
  AirPlaneControl(AirPlane& plane) : plane_(plane) {}

  /** Set throttle (0-100%) */
  void setThrottle(int percent) { plane_.setThrottle(percent); }

  /** Set pitch (degrees): positive = nose up, negative = nose down */
  void setPitch(int angle) { plane_.setElevator(angle); }

  /** Set roll (degrees): positive = left wing up, negative = right wing up */
  void setRoll(int angle) {
    // For simple differential ailerons: left = +angle, right = -angle
    plane_.setAilerons(angle, -angle);
  }

  /** Set yaw (degrees): positive = nose left, negative = nose right */
  void setYaw(int angle) { plane_.setRudder(angle); }

  /** Reset all controls */
  void end() { plane_.end(); }

 protected:
  AirPlane& plane_;
};

}  // namespace tinyrobotics
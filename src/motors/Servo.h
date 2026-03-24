#pragma once
#include <Arduino.h>
#include <Servo.h>

namespace tinyrobotics {

/**
 * @brief High-level wrapper for Arduino Servo control (RC servo motors).
 *
 * This class provides an easy interface for controlling standard RC servo motors
 * using the Arduino Servo library. It allows you to:
 *   - Attach/detach a servo to a digital pin
 *   - Set the servo angle in degrees (using ROS right-handed convention: 0 is forward, +left, -right)
 *   - Set the servo position by pulse width (microseconds)
 *   - Read back the last set angle (in degrees)
 *   - Constrain the allowed angle range for safety or mechanical limits
 *
 * Usage Example:
 * @code
 * tinyrobotics::ServoMotor servo;
 * servo.attach(9);           // Attach to pin 9
 * servo.setConstraints(-45, 45); // Limit range to -45..45 degrees
 * servo.setAngle(45);        // Set to 45 degrees left
 * int angle = servo.getAngle(); // Read last angle
 * servo.detach();            // Detach when done
 * @endcode
 *
 * @note Requires the Arduino Servo library.
 */
class ServoMotor {
 public:
  ServoMotor() = default;

  /** Attach the servo to a pin */
  void attach(int pin) { servo.attach(pin); }

  /** Detach the servo */
  void detach() { servo.detach(); }

  /** Set servo angle (degrees, 90 .. -90): 0 is forward */
  void setAngle(int rosAngleDegrees) {
    // potentially constrain the angles
    if (rosAngleDegrees < minAngle) rosAngleDegrees = minAngle;
    if (rosAngleDegrees > maxAngle) rosAngleDegrees = maxAngle;
    int angle = map(rosAngleDegrees, -90, 90, 0, 180);
    servo.write(constrain(angle, 0, 180));
  }

  /** Get the last written angle (degrees) */
  int getAngle() const {
    int angle = servo.read();
    return map(angle, 0, 180, -90, 90);
  }

  /// Set constraints on the allowed angle range (degrees). This can be used to
  /// prevent the servo from moving beyond physical limits or to limit the range
  /// of motion for safety: by default, the full range of -90 to 90 degrees is allowed.
  void setConstraints(int minAngle, int maxAngle) {
    minAngle = max(minAngle, -90);
    maxAngle = min(maxAngle, 90);
  }

 private:
  ::Servo servo;
};

}  // namespace tinyrobotics

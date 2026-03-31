#pragma once
#include <cmath>
#include "TinyRobotics/units/Units.h"

namespace tinyrobotics {

/**
 * @brief Simple 3D orientation class (yaw, pitch, roll in radians)
 */
class Orientation3D {
 public:
  Orientation3D() = default;
  Orientation3D(float yaw, float pitch, float roll)
      : yaw(yaw), pitch(pitch), roll(roll) {}
  Orientation3D(Angle yaw, Angle pitch, Angle roll) {
    set(yaw, pitch, roll);
  }

  float yaw = 0.0f;   ///< Yaw angle (radians)
  float pitch = 0.0f; ///< Pitch angle (radians)
  float roll = 0.0f;  ///< Roll angle (radians)

  Angle getYaw() const { return Angle(yaw, AngleUnit::RAD); }
  Angle getPitch() const { return Angle(pitch, AngleUnit::RAD); }
  Angle getRoll() const { return Angle(roll, AngleUnit::RAD); }


  void set(float newYaw, float newPitch, float newRoll) {
    yaw = newYaw;
    pitch = newPitch;
    roll = newRoll;
  }

  void set(Angle newYaw, Angle newPitch, Angle newRoll) {
    yaw = newYaw.getAngle(AngleUnit::RAD);
    pitch = newPitch.getAngle(AngleUnit::RAD);
    roll = newRoll.getAngle(AngleUnit::RAD);
  }

  void wrap() {
    wrapAngle(yaw);
    wrapAngle(pitch);
    wrapAngle(roll);
  }

 private:
  static void wrapAngle(float& a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
  }
};

} // namespace tinyrobotics

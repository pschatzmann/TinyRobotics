#pragma once
#include "Speed.h"

namespace tinyrobotics {

using VelocityUnit = SpeedUnit;

/**
 * @brief Represents a 3D speed or velocity vector with unit support.
 *
 * This class encapsulates a 3D speed (or velocity) measurement, storing x, y, and z components
 * along with a unit (meters per second, kilometers per hour, feet per second, or miles per hour).
 * It provides methods to retrieve each component in any supported unit, handling conversion as needed.
 *
 * Speed3D is useful for robotics, navigation, and simulation applications where 3D motion must be
 * represented and manipulated in a type-safe and unit-aware manner. It is compatible with the Speed
 * class for 1D speed and can be used interchangeably with Velocity3D.
 *
 * Example usage:
 *   Velocity3D v(1.0, 0.0, 0.0, SpeedUnit::MPS); // 1 m/s along x
 *   float vx_kph = v.getX(SpeedUnit::KPH);    // Convert x component to km/h
 */
class Velocity3D {
 public:
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  VelocityUnit unit = VelocityUnit::MPS;

  Velocity3D() = default;
  Velocity3D(float x, float y, float z, VelocityUnit unit)     : x(x), y(y), z(z), unit(unit) {}

  float getX(VelocityUnit desiredUnit) const {
    if (unit == desiredUnit) return x;
    Speed tempSpeed(x, unit);
    return tempSpeed.getValue(desiredUnit);
  }
  float getY(VelocityUnit desiredUnit) const {
    if (unit == desiredUnit) return y;
    Speed tempSpeed(y, unit);
    return tempSpeed.getValue(desiredUnit);
  }
  float getZ(VelocityUnit desiredUnit) const {
    if (unit == desiredUnit) return z;
    Speed tempSpeed(z, unit);
    return tempSpeed.getValue(desiredUnit);
  }
};

using Speed3D = Velocity3D;

} // namespace tinyrobotics
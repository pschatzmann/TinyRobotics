#pragma once
#include "Speed.h"

namespace tinyrobotics {

/**
 * @typedef VelocityUnit
 * @brief Alias for SpeedUnit to represent velocity units.
 * @ingroup units
 */
using VelocityUnit = SpeedUnit;

/**
 * @class Velocity3D
 * @ingroup units
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
 * @code
 *   Velocity3D v(1.0, 0.0, 0.0, SpeedUnit::MPS); // 1 m/s along x
 *   float vx_kph = v.getX(SpeedUnit::KPH);    // Convert x component to km/h
 * @endcode
 */
class Velocity3D {
  Velocity3D operator+(const Velocity3D& other) const {
    return Velocity3D(x + other.getX(unit), y + other.getY(unit), z + other.getZ(unit), unit);
  }

  Velocity3D operator-(const Velocity3D& other) const {
    return Velocity3D(x - other.getX(unit), y - other.getY(unit), z - other.getZ(unit), unit);
  }

  Velocity3D operator*(float scalar) const {
    return Velocity3D(x * scalar, y * scalar, z * scalar, unit);
  }

  Velocity3D operator/(float scalar) const {
    if (scalar == 0) return Velocity3D(0, 0, 0, unit);
    return Velocity3D(x / scalar, y / scalar, z / scalar, unit);
  }

  bool operator==(const Velocity3D& other) const {
    return x == other.getX(unit) && y == other.getY(unit) && z == other.getZ(unit);
  }

  bool operator!=(const Velocity3D& other) const { return !(*this == other); }

  bool operator<(const Velocity3D& other) const {
    return (x < other.getX(unit)) && (y < other.getY(unit)) && (z < other.getZ(unit));
  }

  bool operator<=(const Velocity3D& other) const {
    return (x <= other.getX(unit)) && (y <= other.getY(unit)) && (z <= other.getZ(unit));
  }

  bool operator>(const Velocity3D& other) const {
    return (x > other.getX(unit)) && (y > other.getY(unit)) && (z > other.getZ(unit));
  }

  bool operator>=(const Velocity3D& other) const {
    return (x >= other.getX(unit)) && (y >= other.getY(unit)) && (z >= other.getZ(unit));
  }

  Velocity3D& operator+=(const Velocity3D& other) {
    x += other.getX(unit);
    y += other.getY(unit);
    z += other.getZ(unit);
    return *this;
  }

  Velocity3D& operator-=(const Velocity3D& other) {
    x -= other.getX(unit);
    y -= other.getY(unit);
    z -= other.getZ(unit);
    return *this;
  }

  Velocity3D& operator*=(float scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }

  Velocity3D& operator/=(float scalar) {
    if (scalar == 0) {
      x = y = z = 0;
    } else {
      x /= scalar;
      y /= scalar;
      z /= scalar;
    }
    return *this;
  }
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
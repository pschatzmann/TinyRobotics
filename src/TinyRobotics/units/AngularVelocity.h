#pragma once
#include <cmath>

namespace tinyrobotics {

/**
 * @enum AngularVelocityUnit
 * @ingroup units
 * @brief Supported angular velocity units for conversion and representation.
 */
enum class AngularVelocityUnit { RadPerSec, DegPerSec };

/**
 * @class AngularVelocity
 * @ingroup units
 * @brief Represents a 1D angular velocity with unit support.
 */
class AngularVelocity {
  AngularVelocity operator+(const AngularVelocity& other) const {
    float otherValue = other.getValue(unit);
    return AngularVelocity(angularVelocity + otherValue, unit);
  }

  AngularVelocity operator-(const AngularVelocity& other) const {
    float otherValue = other.getValue(unit);
    return AngularVelocity(angularVelocity - otherValue, unit);
  }

  AngularVelocity operator*(float scalar) const {
    return AngularVelocity(angularVelocity * scalar, unit);
  }

  AngularVelocity operator/(float scalar) const {
    if (scalar == 0) return AngularVelocity(0, unit);
    return AngularVelocity(angularVelocity / scalar, unit);
  }

  bool operator==(const AngularVelocity& other) const {
    return angularVelocity == other.getValue(unit);
  }

  bool operator!=(const AngularVelocity& other) const { return !(*this == other); }

  bool operator<(const AngularVelocity& other) const {
    return angularVelocity < other.getValue(unit);
  }

  bool operator<=(const AngularVelocity& other) const {
    return angularVelocity <= other.getValue(unit);
  }

  bool operator>(const AngularVelocity& other) const {
    return angularVelocity > other.getValue(unit);
  }

  bool operator>=(const AngularVelocity& other) const {
    return angularVelocity >= other.getValue(unit);
  }

  AngularVelocity& operator+=(const AngularVelocity& other) {
    angularVelocity += other.getValue(unit);
    return *this;
  }

  AngularVelocity& operator-=(const AngularVelocity& other) {
    angularVelocity -= other.getValue(unit);
    return *this;
  }

  AngularVelocity& operator*=(float scalar) {
    angularVelocity *= scalar;
    return *this;
  }

  AngularVelocity& operator/=(float scalar) {
    if (scalar == 0) {
      angularVelocity = 0;
    } else {
      angularVelocity /= scalar;
    }
    return *this;
  }
 public:
  float angularVelocity = 0.0f;
  AngularVelocityUnit unit = AngularVelocityUnit::RadPerSec;

  AngularVelocity() = default;
  AngularVelocity(float angularVelocity, AngularVelocityUnit unit)
      : angularVelocity(angularVelocity), unit(unit) {}

  void setValue(float newAngularVelocity, AngularVelocityUnit newUnit) {
    angularVelocity = newAngularVelocity;
    unit = newUnit;
  }
  float getValue(AngularVelocityUnit desiredUnit) const {
    if (unit == desiredUnit) return angularVelocity;
    switch (unit) {
      case AngularVelocityUnit::RadPerSec:
        if (desiredUnit == AngularVelocityUnit::DegPerSec)
          return angularVelocity * 180.0f / static_cast<float>(M_PI);
        break;
      case AngularVelocityUnit::DegPerSec:
        if (desiredUnit == AngularVelocityUnit::RadPerSec)
          return angularVelocity * static_cast<float>(M_PI) / 180.0f;
        break;
    }
    return 0;  // Invalid conversion
  }
};

/**
 * @brief Represents a 3D angular velocity vector with unit support.
 *
 * This class encapsulates angular velocity in three dimensions (x, y, z),
 * along with a unit (radians per second or degrees per second). It provides
 * methods to retrieve each component in any supported unit, handling conversion
 * as needed.
 *
 * AngularVelocity3D is useful for robotics, navigation, and simulation
 * applications where 3D rotational motion must be represented and manipulated
 * in a type-safe and unit-aware manner. It is compatible with the
 * AngularVelocity class for 1D angular velocity.
 *
 * Example usage:
 *   AngularVelocity3D w(0.1, 0.0, 0.0, AngularVelocityUnit::RadPerSecond); //
 * 0.1 rad/s about x float wx_deg = w.getX(AngularVelocityUnit::DegPerSecond);
 * // Convert x to deg/s
 */
/**
 * @class AngularVelocity3D
 * @ingroup units
 * @brief Represents a 3D angular velocity vector with unit support.
 */
class AngularVelocity3D {
 public:
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  AngularVelocityUnit unit = AngularVelocityUnit::RadPerSec;

  AngularVelocity3D() = default;
  AngularVelocity3D(float x, float y, float z, AngularVelocityUnit unit)
      : x(x), y(y), z(z), unit(unit) {}

  float getX(AngularVelocityUnit desiredUnit) const {
    if (unit == desiredUnit) return x;
    AngularVelocity temp(x, unit);
    return temp.getValue(desiredUnit);
  }
  float getY(AngularVelocityUnit desiredUnit) const {
    if (unit == desiredUnit) return y;
    AngularVelocity temp(y, unit);
    return temp.getValue(desiredUnit);
  }
  float getZ(AngularVelocityUnit desiredUnit) const {
    if (unit == desiredUnit) return z;
    AngularVelocity temp(z, unit);
    return temp.getValue(desiredUnit);
  }
};

}  // namespace tinyrobotics

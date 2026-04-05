#pragma once
#include <cmath>

namespace tinyrobotics {

/**
 * @enum AngleUnit
 * @ingroup units
 * @brief Supported angle units for conversion and representation.
 */
enum class AngleUnit { DEG, RAD };

/**
 * @class Angle
 * @ingroup units
 * @brief Represents an angle with unit conversion and wrap-around support.
 *
 * The Angle class encapsulates an angle value and its unit, supporting degrees
 * (DEG) and radians (RAD). It provides methods to set and retrieve the angle in
 * any supported unit, automatically handling conversions and wrap-around
 * (0-360° or 0-2π rad).
 *
 * - Internal state is always consistent with the last set value and unit.
 * - Designed for embedded and robotics applications where unit flexibility and
 * efficiency are required.
 * - Supports addition and subtraction with automatic wrap-around.
 * - Use getValue() to retrieve the angle in any unit; use setValue() to update
 * the value and unit.
 *
 * Example:
 * @code
 *   Angle a(90.0, AngleUnit::DEG);
 *   float rad = a.getValue(AngleUnit::RAD); // Convert to radians
 *   a.add(Angle(45.0, AngleUnit::DEG));    // Add 45 degrees
 *   Angle b = a - Angle(180.0, AngleUnit::DEG); // Subtract 180 degrees
 * @endcode
 *
 * @note Invalid conversions return -1.0f.
 *
 * @see AngleUnit
 */
class Angle {
 public:
  Angle() = default;
  Angle(float angle, AngleUnit unit) { setValue(angle, unit); }

  void setValue(float newAngle, AngleUnit newUnit) {
    angle = newAngle;
    unit = newUnit;
  }

  float getValue(AngleUnit desiredUnit) const {
    if (unit == desiredUnit) return angle;
    switch (unit) {
      case AngleUnit::DEG:
        if (desiredUnit == AngleUnit::RAD) return angle * (float)M_PI / 180.0f;
        break;
      case AngleUnit::RAD:
        if (desiredUnit == AngleUnit::DEG) return angle * 180.0f / (float)M_PI;
        break;
    }
    return -1;  // Invalid conversion
  }

  void add(Angle other) {
    float otherAngle = other.getValue(unit);
    if (otherAngle >= 0) {
      angle += otherAngle;
    }
    float maxValue = getMaxValue(unit);
    if (angle >= maxValue) angle -= maxValue;
    if (angle < 0) angle += maxValue;
  }

  float getMaxValue(AngleUnit desiredUnit) const {
    switch (desiredUnit) {
      case AngleUnit::DEG:
        return 360.0f;
      case AngleUnit::RAD:
        return 2.0f * (float)M_PI;
    }
    return -1;  // Invalid unit
  }

  Angle operator+(const Angle& other) const {
    Angle result = *this;
    result.add(other);
    return result;
  }

  Angle operator-(const Angle& other) const {
    Angle result = *this;
    float otherAngle = other.getValue(unit);
    if (otherAngle >= 0) {
      result.angle -= otherAngle;
    }
    float maxValue = getMaxValue(unit);
    if (result.angle >= maxValue) result.angle -= maxValue;
    if (result.angle < 0) result.angle += maxValue;
    return result;
  }

 protected:
  float angle = 0.0f;
  AngleUnit unit = AngleUnit::DEG;
};

}  // namespace tinyrobotics
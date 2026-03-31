#pragma once
#include <cmath>

namespace tinyrobotics {

enum class AngleUnit { DEG, RAD };

/**
 * @brief A simple class to represent an angle with a specific unit (degrees or
 * radians). It provides methods to set and get the angle in different units, as
 * well as basic arithmetic operations (addition and subtraction) that handle
 * unit conversion and wrap-around correctly. The class can be used for various
 * applications such as navigation, robotics, or any scenario that requires
 * angle representation and manipulation. The internal representation of the
 * angle is stored in the unit specified at construction, and all operations
 * ensure that the angle remains within the valid range (0 to 360 degrees or 0
 * to 2π radians) depending on the unit.
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
        if (desiredUnit == AngleUnit::RAD) return angle * M_PI / 180.0f;
        break;
      case AngleUnit::RAD:
        if (desiredUnit == AngleUnit::DEG) return angle * 180.0f / M_PI;
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
        return 2 * M_PI;
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
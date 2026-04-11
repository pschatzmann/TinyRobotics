#pragma once

namespace tinyrobotics {

/**
 * @enum DistanceUnit
 * @ingroup units
 * @brief Supported distance units for conversion and representation.
 */
enum class DistanceUnit { M, CM, MM, FEET };

/**
 * @class Distance
 * @ingroup units
 * @brief Represents a distance measurement with unit conversion support.
 *
 * The Distance class encapsulates a distance value and its unit, supporting
 * meters (M), centimeters (CM), millimeters (MM), and feet (FEET). It provides
 * methods to set and retrieve the distance in any supported unit, automatically
 * handling conversions.
 *
 * - Internal state is always consistent with the last set value and unit.
 * - Designed for embedded and robotics applications where unit flexibility and
 * efficiency are required.
 * - Use getValue() to retrieve the distance in any unit; use setValue() to
 * update the value and unit.
 *
 * Example:
 * @code
 *   Distance d(1.0, DistanceUnit::M);
 *   float feet = d.getValue(DistanceUnit::FEET); // Convert to feet
 *   d.setValue(100.0, DistanceUnit::CM);         // Update value in centimeters
 *   float meters = d.getValue(DistanceUnit::M);  // Convert back to meters
 * @endcode
 *
 * @note Invalid conversions return -1.0f.
 *
 * @see DistanceUnit
 */
class Distance {
 public:
  Distance() = default;
  Distance(float distance, DistanceUnit unit) { setValue(distance, unit); }

  void setValue(float newDistance, DistanceUnit newUnit) {
    distance = newDistance;
    unit = newUnit;
  }

  float getValue(DistanceUnit desiredUnit) const {
    if (unit == desiredUnit) return distance;
    switch (unit) {
      case DistanceUnit::M:
        if (desiredUnit == DistanceUnit::CM) return distance * 100;
        if (desiredUnit == DistanceUnit::MM) return distance * 1000;
        if (desiredUnit == DistanceUnit::FEET) return distance * 3.28084f;
        break;
      case DistanceUnit::CM:
        if (desiredUnit == DistanceUnit::M) return distance / 100;
        if (desiredUnit == DistanceUnit::MM) return distance * 10;
        if (desiredUnit == DistanceUnit::FEET) return distance * 0.0328084f;
        break;
      case DistanceUnit::MM:
        if (desiredUnit == DistanceUnit::M) return distance / 1000;
        if (desiredUnit == DistanceUnit::CM) return distance / 10;
        if (desiredUnit == DistanceUnit::FEET) return distance * 0.00328084f;
        break;
      case DistanceUnit::FEET:
        if (desiredUnit == DistanceUnit::M) return distance / 3.28084f;
        if (desiredUnit == DistanceUnit::CM) return distance / 0.0328084f;
        if (desiredUnit == DistanceUnit::MM) return distance / 0.00328084f;
        break;
    }
    return -1;  // Invalid conversion
  }

  Distance operator+(const Distance& other) const {
    float otherDistance = other.getValue(unit);
    return Distance(distance + otherDistance, unit);
  }

  Distance operator-(const Distance& other) const {
    float otherDistance = other.getValue(unit);
    return Distance(distance - otherDistance, unit);
  }

  Distance operator*(float scalar) const {
    return Distance(distance * scalar, unit);
  }

  Distance operator/(float scalar) const {
    if (scalar == 0) return Distance(0, unit);
    return Distance(distance / scalar, unit);
  }

  bool operator==(const Distance& other) const {
    return distance == other.getValue(unit);
  }

  bool operator!=(const Distance& other) const { return !(*this == other); }

  bool operator<(const Distance& other) const {
    return distance < other.getValue(unit);
  }

  bool operator<=(const Distance& other) const {
    return distance <= other.getValue(unit);
  }

  bool operator>(const Distance& other) const {
    return distance > other.getValue(unit);
  }

  bool operator>=(const Distance& other) const {
    return distance >= other.getValue(unit);
  }

  Distance& operator+=(const Distance& other) {
    distance += other.getValue(unit);
    return *this;
  }

  Distance& operator-=(const Distance& other) {
    distance -= other.getValue(unit);
    return *this;
  }

  Distance& operator*=(float scalar) {
    distance *= scalar;
    return *this;
  }

  Distance& operator/=(float scalar) {
    if (scalar == 0) {
      distance = 0;
    } else {
      distance /= scalar;
    }
    return *this;
  }

 protected:
  float distance = 0.0f;
  DistanceUnit unit = DistanceUnit::M;  ///< Unit of the distance. @see DistanceUnit
};

/**
 * @brief Represents a 3D distance or position vector with unit support.
 *
 * This class encapsulates a 3D distance measurement, storing x, y, and z
 * components along with a unit (meters, centimeters, millimeters, or feet). It
 * provides methods to retrieve each component in any supported unit, handling
 * conversion as needed.
 *
 * Distance3D is useful for robotics, navigation, mapping, and simulation
 * applications where 3D positions or displacements must be represented and
 * manipulated in a type-safe and unit-aware manner. It is compatible with the
 * Distance class for 1D distance.
 *
 * Example usage:
 *   Distance3D p(1.0, 2.0, 3.0, DistanceUnit::M); // 1m, 2m, 3m in x, y, z
 *   float x_cm = p.getX(DistanceUnit::CM);        // Convert x to centimeters
 */
class Distance3D {
 public:
  Distance3D() = default;
  Distance3D(float x, float y, float z, DistanceUnit unit)
      : x(x), y(y), z(z), unit(unit) {}

  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  DistanceUnit unit =
      DistanceUnit::M;  ///< Unit of the 3D distance. @see DistanceUnit

  float getX(DistanceUnit desiredUnit) const {
    if (unit == desiredUnit) return x;
    Distance tempDistance(x, unit);
    return tempDistance.getValue(desiredUnit);
  }
  float getY(DistanceUnit desiredUnit) const {
    if (unit == desiredUnit) return y;
    Distance tempDistance(y, unit);
    return tempDistance.getValue(desiredUnit);
  }
  float getZ(DistanceUnit desiredUnit) const {
    if (unit == desiredUnit) return z;
    Distance tempDistance(z, unit);
    return tempDistance.getValue(desiredUnit);
  }
};

}  // namespace tinyrobotics
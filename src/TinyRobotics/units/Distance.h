#pragma once

namespace tinyrobotics {

enum class DistanceUnit { M, CM, MM, FEET };

/**
 * @brief This class represents a distance measurement with a specific unit
 * (meters, centimeters, millimeters, or feet). It provides methods to set and
 * get the distance in different units, as well as handle unit conversion when
 * retrieving the distance in a desired unit. The internal representation of the
 * distance is stored in the unit specified at construction, and all operations
 * ensure that the distance can be accurately converted between supported units.
 * This class can be used for various applications such as navigation, robotics,
 * or any scenario that requires distance representation and manipulation. The
 * getDistance method allows for easy retrieval of the distance in the desired
 * unit, while the setDistance method allows for updating the distance
 * measurement with a new value and unit. The class is designed to be simple and
 * efficient for use in embedded systems, with basic unit conversion logic that
 * covers common distance units.
 *
 */
class Distance {
 public:
  Distance() = default;
  Distance(float distance, DistanceUnit unit) { setDistance(distance, unit); }

  void setDistance(float newDistance, DistanceUnit newUnit) {
    distance = newDistance;
    unit = newUnit;
  }

  float getDistance(DistanceUnit desiredUnit) const {
    if (unit == desiredUnit) return distance;
    switch (unit) {
      case DistanceUnit::M:
        if (desiredUnit == DistanceUnit::CM) return distance * 100;
        if (desiredUnit == DistanceUnit::MM) return distance * 1000;
        if (desiredUnit == DistanceUnit::FEET) return distance * 3.28084;
        break;
      case DistanceUnit::CM:
        if (desiredUnit == DistanceUnit::M) return distance / 100;
        if (desiredUnit == DistanceUnit::MM) return distance * 10;
        if (desiredUnit == DistanceUnit::FEET) return distance * 0.0328084;
        break;
      case DistanceUnit::MM:
        if (desiredUnit == DistanceUnit::M) return distance / 1000;
        if (desiredUnit == DistanceUnit::CM) return distance / 10;
        if (desiredUnit == DistanceUnit::FEET) return distance * 0.00328084;
        break;
      case DistanceUnit::FEET:
        if (desiredUnit == DistanceUnit::M) return distance / 3.28084;
        if (desiredUnit == DistanceUnit::CM) return distance / 0.0328084;
        if (desiredUnit == DistanceUnit::MM) return distance / 0.00328084;
        break;
    }
    return -1;  // Invalid conversion
  }

 protected:
  float distance = 0.0f;
  DistanceUnit unit = DistanceUnit::M;
};

class Distance3D {
  public:
    Distance3D() = default;
    Distance3D(float x, float y, float z, DistanceUnit unit)
        : x(x), y(y), z(z), unit(unit) {}
  
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    DistanceUnit unit = DistanceUnit::M;  

    float getX(DistanceUnit desiredUnit) const {
        if (unit == desiredUnit) return x;
        Distance tempDistance(x, unit);
        return tempDistance.getDistance(desiredUnit);
      }
    float getY(DistanceUnit desiredUnit) const {
        if (unit == desiredUnit) return y;
        Distance tempDistance(y, unit);
        return tempDistance.getDistance(desiredUnit);
      }
    float getZ(DistanceUnit desiredUnit) const {
        if (unit == desiredUnit) return z;
        Distance tempDistance(z, unit);
        return tempDistance.getDistance(desiredUnit); 
    }
};

}  // namespace tinyrobotics
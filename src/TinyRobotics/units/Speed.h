#pragma once

namespace tinyrobotics {

enum class SpeedUnit { MPS, KPH, FPS, MPH };

/**
 * @brief This class represents a speed measurement with a specific unit (meters
 * per second, kilometers per hour, feet per second, or miles per hour). It
 * provides methods to set and get the speed in different units, as well as
 * handle unit conversion when retrieving the speed in a desired unit. The
 * internal representation of the speed is stored in the unit specified at
 * construction, and all operations ensure that the speed can be accurately
 * converted between supported units. This class can be used for various
 * applications such as navigation, robotics, or any scenario that requires
 * speed representation and manipulation. The getSpeed method allows for easy
 * retrieval of the speed in the desired unit, while the setSpeed method allows
 * for updating the speed measurement with a new value and unit. The class is
 * designed to be simple and efficient for use in embedded systems, with basic
 * unit conversion logic that covers common speed units.
 *
 */
class Speed {
 public:
  Speed() = default;
  Speed(float speed, SpeedUnit unit) { setSpeed(speed, unit); }

  void setSpeed(float newSpeed, SpeedUnit newUnit) {
    speed = newSpeed;
    unit = newUnit;
  }

  float getSpeed(SpeedUnit desiredUnit) const {
    if (unit == desiredUnit) return speed;
    switch (unit) {
      case SpeedUnit::MPS:
        if (desiredUnit == SpeedUnit::KPH) return speed * 3.6;
        if (desiredUnit == SpeedUnit::FPS) return speed * 3.28084;
        if (desiredUnit == SpeedUnit::MPH) return speed * 2.23694;
        break;
      case SpeedUnit::KPH:
        if (desiredUnit == SpeedUnit::MPS) return speed / 3.6;
        if (desiredUnit == SpeedUnit::FPS) return speed * 0.911344;
        if (desiredUnit == SpeedUnit::MPH) return speed * 0.621371;
        break;
      case SpeedUnit::FPS:
        if (desiredUnit == SpeedUnit::MPS) return speed / 3.28084;
        if (desiredUnit == SpeedUnit::KPH) return speed * 1.09728;
        if (desiredUnit == SpeedUnit::MPH) return speed * 0.681818;
        break;
      case SpeedUnit::MPH:
        if (desiredUnit == SpeedUnit::MPS) return speed / 2.23694;
        if (desiredUnit == SpeedUnit::KPH) return speed * 1.60934;
        if (desiredUnit == SpeedUnit::FPS) return speed * 1.46667;
        break;
    }
    return -1;  // Invalid conversion
  }

 protected:
  float speed = 0.0f;
  SpeedUnit unit = SpeedUnit::MPS;
};

}  // namespace tinyrobotics
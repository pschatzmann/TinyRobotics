#pragma once

namespace tinyrobotics {

/**
 * @enum SpeedUnit
 * @ingroup units
 * @brief Supported speed units for conversion and representation.
 */
enum class SpeedUnit { MPS, KPH, FPS, MPH };

/**
 * @class Speed
 * @ingroup units
 * @brief Represents a speed measurement with unit conversion support.
 *
 * The Speed class encapsulates a speed value and its unit, supporting meters
 * per second (MPS), kilometers per hour (KPH), feet per second (FPS), and miles
 * per hour (MPH). It provides methods to set and retrieve the speed in any
 * supported unit, automatically handling conversions.
 *
 * - Internal state is always consistent with the last set value and unit.
 * - Designed for embedded and robotics applications where unit flexibility and
 * efficiency are required.
 * - Use getValue() to retrieve the speed in any unit; use setValue() to update
 * the value and unit.
 *
 * Example:
 * @code
 *   Speed s(10.0, SpeedUnit::MPS);
 *   float mph = s.getValue(SpeedUnit::MPH); // Convert to miles per hour
 *   s.setValue(36.0, SpeedUnit::KPH);       // Update value in km/h
 *   float mps = s.getValue(SpeedUnit::MPS); // Convert back to m/s
 * @endcode
 *
 * @note Invalid conversions return -1.0f.
 *
 * @see SpeedUnit
 */
class Speed {
 public:
  Speed() = default;
  Speed(float speed, SpeedUnit unit) { setValue(speed, unit); }

  void setValue(float newSpeed, SpeedUnit newUnit) {
    speed = newSpeed;
    unit = newUnit;
  }

  float getValue(SpeedUnit desiredUnit) const {
    if (unit == desiredUnit) return speed;
    switch (unit) {
      case SpeedUnit::MPS:
        if (desiredUnit == SpeedUnit::KPH) return speed * 3.6f;
        if (desiredUnit == SpeedUnit::FPS) return speed * 3.28084f;
        if (desiredUnit == SpeedUnit::MPH) return speed * 2.23694f;
        break;
      case SpeedUnit::KPH:
        if (desiredUnit == SpeedUnit::MPS) return speed / 3.6f;
        if (desiredUnit == SpeedUnit::FPS) return speed * 0.911344f;
        if (desiredUnit == SpeedUnit::MPH) return speed * 0.621371f;
        break;
      case SpeedUnit::FPS:
        if (desiredUnit == SpeedUnit::MPS) return speed / 3.28084f;
        if (desiredUnit == SpeedUnit::KPH) return speed * 1.09728f;
        if (desiredUnit == SpeedUnit::MPH) return speed * 0.681818f;
        break;
      case SpeedUnit::MPH:
        if (desiredUnit == SpeedUnit::MPS) return speed / 2.23694f;
        if (desiredUnit == SpeedUnit::KPH) return speed * 1.60934f;
        if (desiredUnit == SpeedUnit::FPS) return speed * 1.46667f;
        break;
    }
    return -1;  // Invalid conversion
  }

 protected:
  float speed = 0.0f;
  SpeedUnit unit = SpeedUnit::MPS;  ///< Unit of the speed. @see SpeedUnit
};

using Velocity = Speed;

}  // namespace tinyrobotics
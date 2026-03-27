#pragma once
#include <cstdint>

#include "Config.h"

namespace tinyrobotics {

enum class FrameType : uint8_t {
  WORLD,
  ODOMETRY,
  BASE,
  CAMERA,
  LIDAR,
  WHEEL,
  CUSTOM,
  OBSTACLE,
  TEMP
};

enum class CellState : int8_t { UNKNOWN = -1, FREE = 0, OCCUPIED = 100 };

using DistanceM = DEFAULT_TYPE;
using AngleDeg = DEFAULT_TYPE;

/**
 * @brief Units for message values.
 */
enum class Unit {
  Percent,           ///< Percentage (0-100)
  MetersPerSecond,   ///< Speed in meters per second
  RadiansPerSecond,  ///< Angular speed in radians per second
  Meters,            ///< Distance in meters
  Centimeters,       ///< Distance in centimeters
  Millimeters,       ///< Distance in millimeters
  AngleDegree,       ///< Angle in degrees
  AngleRadian,       ///< Angle in radians
  TemperatureC,      ///< Temperature in Celsius
  TemperatureF,      ///< Temperature in Fahrenheit
  Pixel,             ///< Pixel units (e.g., for image processing)
};

/// Convert a value from the given unit to degrees.
bool toAngleDegree(DEFAULT_TYPE in, Unit unit, AngleDeg& out) {
  switch (unit) {
    case Unit::AngleDegree:
      out = in;
      return true;
    case Unit::AngleRadian:
      out = in * 180.0 / M_PI;
      return true;
    default:
      return false;  // Unsupported unit for angle
  }
}

}  // namespace tinyrobotics
#pragma once

#include "Config.h"
#include <cmath> 

namespace tinyrobotics {

/**
 * @enum FrameType
 * @ingroup utils
 * @brief Frame type for coordinate systems and reference frames.
 */
enum class FrameType : uint8_t {
  WORLD,     ///< World/global reference frame (fixed, absolute)
  ODOMETRY,  ///< Odometry frame (incremental, mobile robot pose)
  BASE,      ///< Robot or vehicle base frame (body frame)
  CAMERA,    ///< Camera sensor frame
  LIDAR,     ///< LIDAR sensor frame
  WHEEL,     ///< Wheel or actuator frame
  CUSTOM,    ///< Custom user-defined frame
  OBSTACLE,  ///< Obstacle or object frame
  TEMP       ///< Temporary or auxiliary frame
};

/**
 * @enum CellState
 * @ingroup utils
 * @brief Cell state for occupancy grid mapping (e.g., UNKNOWN, FREE, OCCUPIED).
 */
enum class CellState : int8_t { UNKNOWN = -1, FREE = 0, OCCUPIED = 100 };

/// Default numeric type for distances:  This can be changed
using DistanceM = DEFAULT_TYPE;

/// Default numeric type for angles in messages (degrees): This can be changed
using AngleDeg = DEFAULT_TYPE;

/**
 * @enum Unit
 * @ingroup utils
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
inline bool toAngleDegree(DEFAULT_TYPE in, Unit unit, AngleDeg& out) {
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
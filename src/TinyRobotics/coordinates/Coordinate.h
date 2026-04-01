#pragma once
#include <cmath>
#include <functional>
#include <string>

#include "TinyRobotics/serialize/Serializable.h"
#include "TinyRobotics/units/Units.h"
#include "TinyRobotics/utils/Common.h"
#include "TinyRobotics/utils/Config.h"

namespace tinyrobotics {

/**
 * @class Coordinate
 * @ingroup coordinates
 * @brief A generic 3D coordinate class for robotics, navigation, and spatial
 * calculations.
 *
 * The Coordinate class represents a point in 3D space with x, y, and z values
 * (in meters by default). It is templated to support different numeric types
 * (e.g., float, double) for precision or memory needs. This class provides
 * essential geometric operations for robotics, mapping, and navigation,
 * including:
 *   - Calculating Euclidean distance to another coordinate (with selectable
 * units)
 *   - Computing the horizontal bearing (heading) and vertical elevation angle
 * to another point
 *   - Navigating to a new coordinate given a distance, heading, and altitude
 * change
 *   - Comparing coordinates for proximity within a specified tolerance
 *   - Basic arithmetic operations (addition, subtraction, assignment)
 *   - Serialization and deserialization to/from string for storage or
 * communication
 *
 * The x, y, and z values are interpreted as meters in a local or global
 * Cartesian frame.
 *
 * @tparam T Numeric type for coordinates (default: float)
 *
 * Example usage:
 * @code
 * Coordinate<float> a(1.0, 2.0, 0.5);
 * Coordinate<float> b(2.0, 3.0, 1.0);
 * float dist = a.distance(b); // Euclidean distance in meters
 * float bearing = a.bearing(b); // Horizontal angle in degrees
 * float elev = a.elevation(b); // Vertical angle in degrees
 * Coordinate<float> c = a.navigate(5.0, 90); // Move 5m east from a
 * @endcode
 *
 * This class is suitable for use in path planning, SLAM, mapping, sensor
 * fusion, and any application requiring 2D or 3D spatial representation and
 * geometric calculations.
 * @ingroup coordinates
 */

template <typename T = DistanceM>
class Coordinate : public Serializable {
 public:
  Coordinate() = default;
  Coordinate(T x, T y, T z = 0) : x(x), y(y), z(z) {}
  Coordinate(const Coordinate& other) : x(other.x), y(other.y), z(other.z) {}
  Coordinate(Distance x, Distance y, Distance z = 0)
      : x(x.getValue(DistanceUnit::M)),
        y(y.getValue(DistanceUnit::M)),
        z(z.getValue(DistanceUnit::M)) {}

  T x = 0;
  T y = 0;
  T z = 0;

  /// Calculate the Euclidean distance to another coordinate, with optional unit
  /// conversion
  DistanceM distance(const Coordinate& other,
                     DistanceUnit unit = DistanceUnit::M) const {
    float distM = distanceM(other);
    Distance dist(distM, DistanceUnit::M);
    return dist.getValue(unit);
  }

  /// Calculate the horizontal bearing (heading) in degrees from this coordinate
  /// to another
  float bearing(const Coordinate& other,
                AngleUnit unit = AngleUnit::DEG) const {
    AngleDeg bearingDegValue = bearingDeg(other);
    Angle angle(bearingDegValue, AngleUnit::DEG);
    return angle.getValue(unit);
  }

  /// Calculate the elevation angle from this coordinate to another
  float elevation(const Coordinate& other,
                  AngleUnit unit = AngleUnit::DEG) const {
    AngleDeg angleDeg = elevationDeg(other);
    Angle angle(angleDeg, AngleUnit::DEG);
    return angle.getValue(unit);
  }

  /// Navigate to a new coordinate given a distance, heading, and optional
  /// altitude change
  Coordinate navigate(Distance distance, Angle bearing,
                      Distance altDiff = 0) const {
    AngleDeg bearingDeg = bearing.getValue(AngleUnit::DEG);
    DistanceM distanceM = distance.getValue(DistanceUnit::M);
    DistanceM altDiffM = altDiff.getValue(DistanceUnit::M);
    return navigate(distanceM, bearingDeg, altDiffM);
  }

  /// Navigate to a new coordinate given a distance, heading, and optional
  /// altitude change
  Coordinate navigate(DistanceM distanceM, float headingDegrees,
                      float altDiffM = 0) const {
    AngleDeg headingRad = headingDegrees * M_PI / 180.0;
    DistanceM newX = x + distanceM * cos(headingRad);
    DistanceM newY = y + distanceM * sin(headingRad);
    return Coordinate(newX, newY, z + altDiffM);  // Keep same altitude
  }

  /// Calculate the altitude difference in meters between this coordinate and
  /// another
  DistanceM altitudeDifference(const Coordinate& other) const {
    return other.z - z;
  }

  /// Check if this coordinate is within a certain distance of another
  /// coordinate
  bool equals(const Coordinate& other, DistanceM limit) const {
    return distance(other) < limit;
  }

  /// Compare two GPS coordinates for proximity within specified distance and
  /// altitude limits
  bool equalsWithAltitude(const Coordinate& other, DistanceM limit,
                          DistanceM altLimit) const {
    return distance(other) < limit &&
           fabs(altitudeDifference(other)) < altLimit;
  }

  /// Calculate new Coordinate by adding offset defined in other to current
  /// coordinate
  Coordinate operator+(const Coordinate& other) const {
    return Coordinate(x + other.x, y + other.y, z + other.z);
  }

  /// Calculate new coordinate by subtracting offset in other from this one
  Coordinate operator-(const Coordinate& other) const {
    return Coordinate(x - other.x, y - other.y, z - other.z);
  }

  /// Add offset defined in other to current coordinate
  void operator+=(const Coordinate& other) {
    x += other.x;
    y += other.y;
    z += other.z;
  }

  /// Subtract offset in other from this one
  void operator-=(const Coordinate& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
  }

  /// Assign values from another coordinate
  void operator=(const Coordinate& other) {
    x = other.x;
    y = other.y;
    z = other.z;
  }

  /// Equality operators for use in std::unordered_map and comparisons
  bool operator==(const Coordinate<T>& other) const {
    return x == other.x && y == other.y && z == other.z;
  }

  bool operator!=(const Coordinate<T>& other) const {
    return !(*this == other);
  }

  /// Lexicographical comparison for STL containers (priority_queue, set, etc.)
  bool operator<(const Coordinate& other) const {
    if (x != other.x) return x < other.x;
    if (y != other.y) return y < other.y;
    return z < other.z;
  }

  std::string toString() const {
    char buf[100];
    snprintf(buf, sizeof(buf), "%s: %.8f, %.8f, %.2f", getTypeName(), x, y, z);
    return std::string(buf);
  }

  bool fromString(const std::string& str) {
    if (str.find(getTypeName()) == std::string::npos)
      return false;  // Must start with type name
    size_t colon = str.find(':');
    if (colon == std::string::npos) return false;
    size_t comma1 = str.find(',');
    if (comma1 == std::string::npos) return false;
    size_t comma2 = str.find(',', comma1 + 1);
    x = std::stof(str.substr(colon, comma1));
    y = std::stof(str.substr(comma1 + 1, comma2 - comma1 - 1));
    if (comma2 != std::string::npos) {
      z = std::stof(str.substr(comma2 + 1));
    } else {
      z = 0;
    }
    return true;
  }

  void setValues(T newX, T newY, T newZ = 0) {
    x = newX;
    y = newY;
    z = newZ;
  }

  const char* getTypeName() const { return "Coordinate"; }

protected:

  AngleDeg bearingDeg(const Coordinate& other) const {
    float dx = other.x - x;
    float dy = other.y - y;
    return atan2(dy, dx) * 180.0 / M_PI;  // Degrees
  }

  // Returns the vertical angle (elevation) in degrees to another coordinate
  AngleDeg elevationDeg(const Coordinate& other) const {
    float dz = other.z - z;
    float dxy =
        sqrt((other.x - x) * (other.x - x) + (other.y - y) * (other.y - y));
    return atan2(dz, dxy) * 180.0 / M_PI;
  }

  DistanceM distanceM(const Coordinate& other) const {
    auto dx = x - other.x;
    auto dy = y - other.y;
    auto dz = z - other.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
  }
};

}  // namespace tinyrobotics


namespace std {
template <typename T>
struct hash<tinyrobotics::Coordinate<T>> {
  std::size_t operator()(const tinyrobotics::Coordinate<T>& c) const noexcept {
    std::size_t h1 = std::hash<T>{}(c.x);
    std::size_t h2 = std::hash<T>{}(c.y);
    return h1 ^ (h2 << 1);
  }
};
}  // namespace std

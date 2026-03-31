#pragma once

#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace tinyrobotics {

/**
 * @brief Represents a geodetic GPS coordinate with latitude, longitude, and
 * optional altitude.
 *
 * The GPSCoordinate class encapsulates a single point on Earth using WGS84
 * latitude and longitude (in degrees), and optional altitude (in meters). It
 * provides methods for:
 *   - Calculating geodesic distance and bearing to another coordinate (using
 * the haversine formula)
 *   - Computing elevation angle and altitude difference
 *   - Navigating to a new coordinate given a distance and bearing (great-circle
 * navigation)
 *   - Comparing coordinates with a specified tolerance (for proximity checks)
 *   - Serializing/deserializing to and from string representations for storage
 * or transmission
 *
 * Fields:
 *   - latitude: WGS84 latitude in degrees (-90 to 90)
 *   - longitude: WGS84 longitude in degrees (-180 to 180)
 *   - altitude: Altitude above ellipsoid in meters (optional)
 *
 * Usage:
 *   - Use isValid() to check if the coordinate is usable for navigation.
 *   - Use distance(), bearing(), and elevation() to compute spatial
 * relationships.
 *   - Use navigate() to project a new coordinate from the current one.
 *   - Use equals() to compare two coordinates within a tolerance.
 *   - Use toString() and fromString() for serialization.
 *
 * Example:
 *   GPSCoordinate a(48.8584, 2.2945, 35); // Eiffel Tower
 *   GPSCoordinate b(51.5007, -0.1246, 15); // London
 *   float dist = a.distance(b); // meters
 *   float brng = a.bearing(b); // degrees
 *   GPSCoordinate c = a.navigate(1000, 90); // 1km east
 *
 * This class is suitable for robotics, mapping, navigation, and geospatial
 * applications on embedded or desktop systems.
 * @ingroup coordinates
 */

class GPSCoordinate {
 public:
  float latitude = 360;   // Degrees (-90 to 90)
  float longitude = 360;  // Degrees (-180 to 180)
  float altitude = 0;     // Meters above ellipsoid (optional)

  GPSCoordinate() = default;

  GPSCoordinate(float lat, float lon, float alt = 0)
      : latitude(lat), longitude(lon), altitude(alt) {}

  /// Check if the values are valid
  bool isValid() const {
    return latitude >= -90 && latitude <= 90 && longitude >= -180 &&
           longitude <= 180;
  }

  /// Check if the values are valid
  operator bool() const { return isValid(); }

  /// Calculate distance to other GPS coordinate
  float distance(const GPSCoordinate& other,
                 DistanceUnit unit = DistanceUnit::M) const {
    float distM = distanceM(other);
    Distance dist(distM, DistanceUnit::M);
    return dist.getValue(unit);
  }

  /// Calculate the bearing (heading) in degrees from this coordinate to another
  /// GPS coordinate.
  float bearing(const GPSCoordinate& other,
                AngleUnit unit = AngleUnit::DEG) const {
    float bearingDeg = bearingDegree(other);
    Angle angle(bearingDeg, AngleUnit::DEG);
    return angle.getValue(unit);
  }

  /// Calculate the elevation angle in degrees from this coordinate to another
  /// GPS coordinate.
  float elevation(const GPSCoordinate& other,
                  AngleUnit unit = AngleUnit::DEG) const {
    float angleDeg = elevationDeg(other);
    Angle angle(angleDeg, AngleUnit::DEG);
    return angle.getValue(unit);
  }

  /// Calculate the altitude difference in meters between this coordinate and
  /// another
  float altitudeDifference(const GPSCoordinate& other) const {
    return other.altitude - altitude;
  }

  /// Calculate a new GPS coordinate given a distance (in meters) and bearing
  GPSCoordinate navigate(Distance distance, Angle bearing,
                         Distance altDiff) const {
    float bearingDeg = bearing.getValue(AngleUnit::DEG);
    float distanceM = distance.getValue(DistanceUnit::M);
    float altDiffM = altDiff.getValue(DistanceUnit::M);
    return navigate(distanceM, bearingDeg, altDiffM);
  }

  /// Calculate a new GPS coordinate given a distance (in meters) and bearing
  GPSCoordinate navigate(float distance_m, float bearing_deg,
                         float alt_diff_m = 0) const {
    float R = 6371000;  // Earth radius in meters
    float bearing_rad = bearing_deg * M_PI / 180.0;
    float lat1 = latitude * M_PI / 180.0;
    float lon1 = longitude * M_PI / 180.0;

    float lat2 = asin(sin(lat1) * cos(distance_m / R) +
                      cos(lat1) * sin(distance_m / R) * cos(bearing_rad));
    float lon2 =
        lon1 + atan2(sin(bearing_rad) * sin(distance_m / R) * cos(lat1),
                     cos(distance_m / R) - sin(lat1) * sin(lat2));

    return GPSCoordinate(lat2 * 180.0 / M_PI, lon2 * 180.0 / M_PI,
                         altitude + alt_diff_m);
  }

  /// Compare two GPS coordinates for proximity within a specified distance
  /// limit
  bool equals(const GPSCoordinate& other, float limit) const {
    return distance(other) < limit;
  }

  /// Compare two GPS coordinates for proximity within specified distance and
  /// altitude limits
  bool equalsWithAltitude(const GPSCoordinate& other, float limit,
                          float altLimit) const {
    return distance(other) < limit &&
           fabs(altitudeDifference(other)) < altLimit;
  }

  /// Serialize GPS coordinate to string representation
  std::string toString() const {
    char buf[100];
    snprintf(buf, sizeof(buf), "%s: %.8f, %.8f, %.2fm", getTypeName(), latitude,
             longitude, altitude);
    return std::string(buf);
  }

  /// load GPS coordinate from string representation (must match toString()
  /// format)
  bool fromString(const std::string& str) {
    float lat, lon, alt;
    char typeName[80]{};
    // The format string must match toString()
    int n =
        sscanf(str.c_str(), "%79[^:]: %f, %f, %fm", typeName, &lat, &lon, &alt);
    if (n == 4) {
      if (strcmp(typeName, getTypeName()) != 0) return false;
      latitude = lat;
      longitude = lon;
      altitude = alt;
      return true;
    }
    return false;
  }

  const char* getTypeName() const { return "GPSCoordinate"; }

 protected:
  /// Calculate the bearing (heading) in degrees from this coordinate to another
  /// GPS coordinate.
  float bearingDegree(const GPSCoordinate& other) const {
    float lat1 = latitude * M_PI / 180.0;
    float lat2 = other.latitude * M_PI / 180.0;
    float dLon = (other.longitude - longitude) * M_PI / 180.0;

    float y = sin(dLon) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    float brng = atan2(y, x);
    return fmod((brng * 180.0 / M_PI + 360), 360);  // Degrees
  }

  float elevationDeg(const GPSCoordinate& other) const {
    float dz = other.altitude - altitude;
    float dxy = distanceM(other);
    return atan2(dz, dxy) * 180.0 / M_PI;  // Degrees
  }

  /// Distance in meters between this coordinate and another GPS coordinate
  float distanceM(const GPSCoordinate& other) const {
    float R = 6371000;  // Earth radius in meters
    float lat1 = latitude * M_PI / 180.0;
    float lat2 = other.latitude * M_PI / 180.0;
    float dLat = (other.latitude - latitude) * M_PI / 180.0;
    float dLon = (other.longitude - longitude) * M_PI / 180.0;

    float a = sin(dLat / 2) * sin(dLat / 2) +
              cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;  // Distance in meters
  }
};

}  // namespace tinyrobotics
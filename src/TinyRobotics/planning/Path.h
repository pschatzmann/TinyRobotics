#pragma once

#include <algorithm>
#include <optional>

#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/utils/AllocatorPSRAM.h"
#include "TinyRobotics/utils/Common.h"

namespace tinyrobotics {

/**
 * @class ScheduledWayPoint
 * @ingroup planning
 * @brief Represents a waypoint with an associated timestamp.
 *
 * Used for time-based path following or trajectory execution, where each waypoint
 * should be reached at a specific time.
 *
 * @tparam T Coordinate type (e.g., Coordinate, GPSCoordinate).
 */
template <typename T>
class ScheduledWayPoint {
 public:
  T coordinate;
  unsigned long timestamp;  // Time at which this waypoint should be reached
};

/**
 * @class WayPointAndSpeed
 * @ingroup planning
 * @brief Represents a waypoint with an associated speed.
 *
 * Used for path following or trajectory execution, where each waypoint specifies
 * the desired speed at that point.
 *
 * @tparam T Coordinate type (e.g., Coordinate, GPSCoordinate).
 */
template <typename T>
class WayPointAndSpeed {
 public:
  T coordinate;
  float speed;
};

/**
 * @class Path
 * @ingroup planning
 * @brief A simple path class that holds a sequence of waypoints. Each waypoint
 * is e.g. a coordinate (e.g., x,y,z) that the robot should follow. The path can
 * be used for navigation, motion planning, or trajectory execution. It provides
 * methods to add waypoints.
 *
 * @tparam T Coordinate e.g. GPSCoordinate, Coordinate, ScheduledWayPoint,
 * WayPointAndSpeed etc.
 */

template <typename T = Coordinate<DistanceM>, typename Allocator = AllocatorPSRAM<T>>
class Path {
 public:
  Path() = default;

  /// Construct a path from a variable number of waypoints
  template <typename... Args>
  Path(const Args&... args) {
    addWaypoints(args...);
  }

  Path(const std::vector<T>& waypoints) : waypoints(waypoints) {}

  void addWaypoint(const T& waypoint) { waypoints.push_back(waypoint); }

  void setWaypoints(const std::vector<T>& new_waypoints) {
    waypoints = new_waypoints;
  }

  bool setWaypoint(int pos, const T& waypoint) {
    if (pos >= 0 && pos < waypoints.size()) {
      waypoints[pos] = waypoint;
      return true;
    }
    return false;
  }

  std::vector<T, Allocator> getWaypoints() const { return waypoints; }

  std::optional<T> getWaypoint(size_t index) const {
    if (index < waypoints.size()) {
      return waypoints[index];
    }
    return std::nullopt;  // Out of bounds
  }

  std::optional<T> getLastWaypoint() const {
    if (!waypoints.empty()) {
      return waypoints.back();
    }
    return std::nullopt;  // No waypoints
  }

  /// Remove the first waypoint in the path (e.g., after reaching it). Returns true if a waypoint was removed, false if the path was already empty.
  bool removeHead() {
    if (!waypoints.empty()) {
      waypoints.erase(waypoints.begin());
      return true;
    }
    return false;  // No waypoints to remove
  }

  // Non-const version
  T& operator[](size_t index) { return waypoints[index]; }
  // Const version
  const T& operator[](size_t index) const { return waypoints[index]; }

  size_t size() const { return waypoints.size(); }

  bool isEmpty() const { return waypoints.empty(); }

  void reverse() { std::reverse(waypoints.begin(), waypoints.end()); }

  void clear() { waypoints.clear(); }

  /// Calculate the total distance
  float distance() const {
    float total_distance = 0.0f;
    for (size_t i = 1; i < waypoints.size(); ++i) {
      total_distance += waypoints[i - 1].distance(waypoints[i]);
    }
    return total_distance;
  }

  // Helper to add multiple waypoints recursively
  void addWaypoints() {}
  template <typename First, typename... Rest>
  void addWaypoints(const First& first, const Rest&... rest) {
    addWaypoint(first);
    addWaypoints(rest...);
  }

 protected:
  std::vector<T, Allocator> waypoints;


};

}  // namespace tinyrobotics
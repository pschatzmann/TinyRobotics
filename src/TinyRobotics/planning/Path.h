#pragma once

#include <algorithm>
#include <optional>

#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/utils/AllocatorPSRAM.h"
#include "TinyRobotics/utils/Common.h"

namespace tinyrobotics {

template <typename T>
class ScheduledWayPoint {
 public:
  T coordinate;
  unsigned long timestamp;  // Time at which this waypoint should be reached
};

template <typename T>
class WayPointAndSpeed {
 public:
  T coordinate;
  float speed;
};

/**
 * @brief A simple path class that holds a sequence of waypoints. Each waypoint
 * is e.g. a coordinate (e.g., x,y,z) that the robot should follow. The path can
 * be used for navigation, motion planning, or trajectory execution. It provides
 * methods to add waypoints.
 *
 * @tparam T Coordinate e.g. GPSCoordinate, Coordinate, ScheduledWayPoint,
 * WayPointAndSpeed etc.
 */

template <typename T = Coordinate<DistanceM>>
class Path {
 public:
  Path() = default;

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

  std::vector<T> getWaypoints() const { return waypoints; }

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

  T& operator[](size_t index) const { return waypoints[index]; }

  size_t size() const { return waypoints.size(); }

  void reverse() { std::reverse(waypoints.begin(), waypoints.end()); }

  void clear() { waypoints.clear(); }

 protected:
  std::vector<T, AllocatorPSRAM<T>> waypoints;
};

}  // namespace tinyrobotics
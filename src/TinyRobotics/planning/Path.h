#pragma once

#include <algorithm>

#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/utils/AllocatorPSRAM.h"
#include "TinyRobotics/utils/Common.h"
#include "TinyRobotics/utils/Optional.h"

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
template <typename CoordinateT>
class ScheduledWayPoint {
 public:
  CoordinateT coordinate;
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
template <typename CoordinateT>
class WayPointAndSpeed {
 public:
  CoordinateT coordinate;
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

template <typename CoordinateT = Coordinate<DistanceM>, typename AllocatorT = AllocatorPSRAM<CoordinateT>>
class Path {
 public:

  /// Default constructor
  Path() = default;

  /// Construct a path from a variable number of waypoints
  template <typename... Args>
  Path(const Args&... args) {
    addWaypoints(args...);
  }

  /// Construct a path from a vector of waypoints
  Path(const std::vector<CoordinateT>& waypoints) : waypoints(waypoints) {}

  /// Add a single waypoint to the path
  void addWaypoint(const CoordinateT& waypoint) { waypoints.push_back(waypoint); }

  /// Add multiple waypoints from a vector
  void addWaypoints(const std::vector<CoordinateT>& new_waypoints) {
    waypoints.insert(waypoints.end(), new_waypoints.begin(), new_waypoints.end());
  }

  /// Helper for variadic addWaypoints (base case)
  void addWaypoints() {}

  /// Helper for variadic addWaypoints (recursive case)
  template <typename First, typename... Rest>
  void addWaypoints(const First& first, const Rest&... rest) {
    addWaypoint(first);
    addWaypoints(rest...);
  }

  /// Replace all waypoints with a new vector
  void setWaypoints(const std::vector<CoordinateT>& new_waypoints) {
    waypoints = new_waypoints;
  }

  /// Set a waypoint at a specific position
  bool setWaypoint(int pos, const CoordinateT& waypoint) {
    if (pos >= 0 && pos < waypoints.size()) {
      waypoints[pos] = waypoint;
      return true;
    }
    return false;
  }

  /// Get all waypoints as a vector
  std::vector<CoordinateT, AllocatorT> getWaypoints() const { return waypoints; }

  /// Get a waypoint by index (returns std::nullopt if out of bounds)
  std::optional<CoordinateT> getWaypoint(size_t index) const {
    if (index < waypoints.size()) {
      return waypoints[index];
    }
    return std::nullopt;  // Out of bounds
  }

  /// Get the last waypoint (returns std::nullopt if empty)
  std::optional<CoordinateT> getLastWaypoint() const {
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

  /// Non-const index operator
  CoordinateT& operator[](size_t index) { return waypoints[index]; }

  /// Const index operator
  const CoordinateT& operator[](size_t index) const { return waypoints[index]; }

  /// Get the number of waypoints
  size_t size() const { return waypoints.size(); }

  /// Check if the path is empty
  bool isEmpty() const { return waypoints.empty(); }

  /// Reverse the order of waypoints
  void reverse() { std::reverse(waypoints.begin(), waypoints.end()); }

  /// Remove all waypoints
  void clear() { waypoints.clear(); }

  /// Calculate the total distance
  float distance() const {
    float total_distance = 0.0f;
    for (size_t i = 1; i < waypoints.size(); ++i) {
      total_distance += waypoints[i - 1].distance(waypoints[i]);
    }
    return total_distance;
  }

 protected:
  std::vector<CoordinateT, AllocatorT> waypoints;
};

}  // namespace tinyrobotics
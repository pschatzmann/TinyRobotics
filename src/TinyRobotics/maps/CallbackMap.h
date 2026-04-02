#pragma once
#include <cmath>
#include <cstdint>
#include <vector>

#include "TinyRobotics/utils/Common.h"

namespace tinyrobotics {

/**
 * @class CallbackMap
 * @brief A map-like utility for generating and validating neighbor coordinates
 * in a configurable pattern.
 *
 * CallbackMap generates neighbor coordinates around a given point at a
 * specified distance and angular resolution. It supports a user-provided
 * callback to determine if a coordinate is valid (e.g., obstacle-free).
 *
 * Typical use: pathfinding, navigation, or virtual occupancy grids where
 * neighbors are not stored but computed on demand.
 *
 * @tparam T Numeric type for coordinates (default: DistanceM)
 *
 * Example usage:
 * @code
 * CallbackMap<> map(1.0f, 16); // 1m distance, 16 directions
 * map.setIsValidCallback([](Coordinate<DistanceM> c) { return c.x > 0; });
 * auto neighbors = map.getNeighbors(Coordinate<DistanceM>(0,0));
 * @endcode
 */

template <typename T = DistanceM>
class CallbackMap {
 public:
  CallbackMap() = default;
  CallbackMap(float distance, int neighborCount)
      : defaultDistance(distance), defaultNeighborCount(neighborCount) {}

  bool isValid(Coordinate<T>coord) const {
    if (isValidCB) {
      return isValidCB(coord);
    }
    return true;  // Default to all cells valid if no callback provided
  }

  void setIsValidCallback(bool (*callback)(Coordinate<T>)) {
    isValidCB = callback;
  }

  /// Get world coordinates of neighboring cells (for pathfinding or navigation)
  std::vector<Coordinate<T>> getNeighbors(Coordinate<T> from) const {
    std::vector<Coordinate<T>> neighbors;
    // Generate neighbors at defaultDistance in defaultResolution directions
    for (int i = 0; i < defaultNeighborCount; ++i) {
      float angle = 2.0f * M_PI * i / defaultNeighborCount;
      float dx = defaultDistance * std::cos(angle);
      float dy = defaultDistance * std::sin(angle);
      Coordinate<T> neighbor(from.x + dx, from.y + dy);
      if (isValid(neighbor)) {
        neighbors.push_back(neighbor);
      }
    }
    return neighbors;
  }

  void setDistance(float distance) { defaultDistance = distance; }
  void setNeighborCount(int neighborCount) {
    defaultNeighborCount = neighborCount;
  }

 protected:
  int defaultNeighborCount =
      36;  // Default to 36 neighbors (10 degree resolution)
  float defaultDistance = 1.0f;
  bool (*isValidCB)(Coordinate<T> coord) =
      nullptr;  // Optional callback for cell validity
};

}  // namespace tinyrobotics
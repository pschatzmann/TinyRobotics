#pragma once
#include <cmath>
#include <cstdint>
#include <vector>

#include "TinyRobotics/maps/IMap.h"
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
class CallbackMap : public IMap<T> {
 public:
  /// @brief Default constructor.
  CallbackMap() = default;

  /// @brief Construct a CallbackMap with extents, resolution, and neighbor
  /// count.
  /// @param distanceX Map extent in X direction (meters)
  /// @param distanceY Map extent in Y direction (meters)
  /// @param resolutionM Grid resolution (meters)
  /// @param neighborCount Number of neighbors to generate
  CallbackMap(float distanceX, float distanceY, float resolutionM,
              int neighborCount)
      : defaultResolutionM(resolutionM),
        defaultNeighborCount(neighborCount),
        distanceX(distanceX),
        distanceY(distanceY) {}

  // --- IMap interface implementations ---

  /// @brief Get the number of cells in the X direction.
  int getXCount() const override {
    return (defaultResolutionM > 0 && distanceX > 0)
               ? static_cast<int>(distanceX / defaultResolutionM)
               : 0;
  }

  /// @brief Get the number of cells in the Y direction.
  int getYCount() const override {
    return (defaultResolutionM > 0 && distanceY > 0)
               ? static_cast<int>(distanceY / defaultResolutionM)
               : 0;
  }

  /// @brief Get the map resolution in meters.
  float getResolution() const override { return defaultResolutionM; }

  /// @brief Generate valid neighbor coordinates around a given point.
  /// @param from Center coordinate
  /// @return Vector of valid neighbor coordinates
  std::vector<Coordinate<T>> getNeighbors(Coordinate<T> from) const override {
    std::vector<Coordinate<T>> neighbors;
    for (int i = 0; i < defaultNeighborCount; ++i) {
      float angle = 2.0f * static_cast<float>(M_PI) * i / defaultNeighborCount;
      float dx = defaultResolutionM * std::cos(angle);
      float dy = defaultResolutionM * std::sin(angle);
      Coordinate<T> neighbor(from.x + dx, from.y + dy);
      if (isValid(neighbor)) {
        neighbors.push_back(neighbor);
      }
    }
    return neighbors;
  }

  /// @brief Check if a coordinate is valid (not occupied).
  /// @param coord Coordinate to check
  /// @return True if valid (not OCCUPIED), false otherwise
  bool isValid(const Coordinate<T>& coord) const override {
    if (isValidCB) {
      return isValidCB(coord) != CellState::OCCUPIED;
    }
    // If no callback is set, treat all cells as valid
    return true;
  }

  /// @brief Get the cell state at grid coordinates (x, y).
  /// @param x X grid index
  /// @param y Y grid index
  /// @param state Output cell state
  /// @return True if cell state is available
  bool getCell(int x, int y, CellState& state) const override {
    Coordinate<T> c = toWorld(x, y);
    if (isValidCB) {
      state = isValidCB(c);
      return true;
    }
    return false;
  }

  /// @brief Convert grid indices to world coordinates.
  /// @param x X grid index
  /// @param y Y grid index
  /// @return World coordinate
  Coordinate<T> toWorld(int x, int y) const override {
    return Coordinate<T>(x * defaultResolutionM, y * defaultResolutionM);
  }

  /// @brief Set the callback for cell validity checking.
  /// @param callback Function pointer taking a coordinate and returning
  /// CellState
  void setIsValidCallback(CellState (*callback)(Coordinate<T>)) {
    isValidCB = callback;
  }

  /// @brief Set the map resolution in meters.
  /// @param resolutionM Grid resolution
  void setResolution(float resolutionM) { defaultResolutionM = resolutionM; }

  /// @brief Set the number of neighbors to generate.
  /// @param neighborCount Number of neighbors
  void setNeighborCount(int neighborCount) {
    defaultNeighborCount = neighborCount;
  }

  /// @brief Set the map extent in X direction.
  /// @param dx Extent in meters
  void setDistanceX(float dx) { distanceX = dx; }

  /// @brief Set the map extent in Y direction.
  /// @param dy Extent in meters
  void setDistanceY(float dy) { distanceY = dy; }

 protected:
  int defaultNeighborCount = 36;
  float defaultResolutionM = 1.0f;
  float distanceX = 0;
  float distanceY = 0;
  CellState (*isValidCB)(Coordinate<T> coord) = nullptr;
};

}  // namespace tinyrobotics

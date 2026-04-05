#pragma once
#include <cmath>
#include <cstdint>
#include <vector>

#include "IMap.h"
#include "TinyRobotics/utils/AllocatorPSRAM.h"
#include "TinyRobotics/utils/Common.h"

namespace tinyrobotics {

/**
 * @class GridMap
 * @ingroup maps
 * @brief A grid map for spatial representation, navigation, and planning.
 *
 * The GridMap class models the environment as a regular grid of cells, each
 * storing a state (e.g., occupancy, height, or other user-defined data). It
 * supports:
 *   - Conversion between world coordinates and grid cell indices
 *   - Efficient cell state access and modification
 *   - Binary Bayesian updates for probabilistic occupancy mapping
 *   - Custom cell validity logic (e.g., for dynamic obstacles)
 *   - Extraction of valid neighbor cells for pathfinding (8-connected)
 *
 * Typical use cases include:
 *   - Robot navigation and path planning
 *   - Occupancy grid mapping (SLAM)
 *   - Environment modeling for simulation or AI
 *
 * Features:
 *   - Template parameter for cell state type (e.g., occupancy, height)
 *   - World-to-cell and cell-to-world coordinate conversion
 *   - Fast access to cell state by index or coordinate
 *   - Probabilistic update using log-odds for sensor fusion
 *   - Customizable cell validity via callback
 *   - Neighbor cell extraction for graph-based search
 *
 * Example:
 * @code
 * GridMap<CellState> map(100, 100, 0.1f); // 10m x 10m, 10cm resolution
 * Coordinate<float> pos(1.2, 3.4);
 * map.setCell(pos, CellState::OCCUPIED);
 * auto state = map.getCell(5, 5);
 * @endcode
 *
 * @tparam StateType The type stored in each cell (e.g., occupancy, height)
 * @tparam T Numeric type for coordinates (default: float)
 */

template <typename StateType = CellState, typename T = DistanceM>
class GridMap : public IMap<T> {
 public:
  /// Cell structure to represent grid cell indices
  struct Cell {
    size_t cx;  // Cell X index
    size_t cy;  // Cell Y index
  };

  GridMap() = default;
  GridMap(int xCount, int yCount, DistanceM resolutionM)
      : resolution(resolutionM) {
    resize(xCount, yCount);
  }
  GridMap(int xCount, int yCount, Distance resolution)
      : xCount(xCount), yCount(yCount) {
    this->resolution = resolution.getValue(DistanceUnit::M);
    resize(xCount, yCount);
  }

  /// World to cell conversion
  bool worldToCell(DistanceM wx, DistanceM wy, Cell& cell) const {
    cell.cx = static_cast<int>(std::floor((wx - origin.x) / resolution));
    cell.cy = static_cast<int>(std::floor((wy - origin.y) / resolution));
    return (cell.cx >= 0 && cell.cx < xCount && cell.cy >= 0 &&
            cell.cy < yCount);
  }

  /// Cell to world (center of cell)
  void cellToWorld(int cx, int cy, DistanceM& wx, DistanceM& wy) const {
    wx = origin.x + (cx + 0.5) * resolution;
    wy = origin.y + (cy + 0.5) * resolution;
  }

  /// Provide the workd coordinates for the cell
  Coordinate<DistanceM> toWorld(int cx, int cy) const {
    Coordinate<DistanceM> coord;
    cellToWorld(cx, cy, coord.x, coord.y);
    return coord;
  }

  /// Provide access to cell state by cell index
  bool getCell(int cx, int cy, StateType& result) {
    if (cx < 0 || cx >= xCount || cy < 0 || cy >= yCount)
      return false;  // Out of bounds
    result = data[cy * xCount + cx];
    return true;
  }
  bool getCell(int cx, int cy, StateType& result) const {
    if (cx < 0 || cx >= xCount || cy < 0 || cy >= yCount)
      return false;  // Out of bounds
    result = data[cy * xCount + cx];
    return true;
  }
  /// Provide access to cell state by coordinate
  bool getCell(Coordinate<T>& coord, StateType& result) {
    Cell cell;
    if (worldToCell(coord.x, coord.y, cell)) {
      return getCell(cell.cx, cell.cy);
    }
    return false;  // Out of bounds
  }

  void setCell(Cell& cell, StateType value) {
    if (cell.cx >= 0 && cell.cx < xCount && cell.cy >= 0 && cell.cy < yCount)
      data[cell.cy * xCount + cell.cx] = value;
  }

  /// Set cell state (for initialization or manual updates)
  void setCell(int cx, int cy, StateType value) {
    if (cx >= 0 && cx < xCount && cy >= 0 && cy < yCount)
      data[cy * xCount + cx] = value;
  }

  /// Set cell state by coordinate (converts to cell index internally)
  void setCell(Coordinate<T>& coord, StateType value) {
    Cell cell;
    if (worldToCell(coord.x, coord.y, cell)) {
      setCell(cell.cx, cell.cy, value);
    }
  }

  /// Determine all neighboring cells (8-connected) for a given cell coordinate.
  std::vector<Cell> getNeighborCells(const Coordinate<DistanceM> from) const {
    Cell cell;
    worldToCell(from.x, from.y, cell);
    int cx = static_cast<int>(cell.cx);
    int cy = static_cast<int>(cell.cy);
    std::vector<Cell> neighbors;
    if (cx < xCount - 1)
      neighbors.push_back(
          {static_cast<size_t>(cx + 1), static_cast<size_t>(cy)});
    if (cx > 0)
      neighbors.push_back(
          {static_cast<size_t>(cx - 1), static_cast<size_t>(cy)});
    if (cy < yCount - 1)
      neighbors.push_back(
          {static_cast<size_t>(cx), static_cast<size_t>(cy + 1)});
    if (cy > 0)
      neighbors.push_back(
          {static_cast<size_t>(cx), static_cast<size_t>(cy - 1)});

    if (cx < xCount - 1 && cy < yCount - 1)
      neighbors.push_back(
          {static_cast<size_t>(cx + 1), static_cast<size_t>(cy + 1)});
    if (cx > 0 && cy < yCount - 1)
      neighbors.push_back(
          {static_cast<size_t>(cx - 1), static_cast<size_t>(cy + 1)});
    if (cx < xCount - 1 && cy > 0)
      neighbors.push_back(
          {static_cast<size_t>(cx + 1), static_cast<size_t>(cy - 1)});
    if (cx > 0 && cy > 0)
      neighbors.push_back(
          {static_cast<size_t>(cx - 1), static_cast<size_t>(cy - 1)});
    return neighbors;
  }

  /// Get world coordinates of neighboring cells (for pathfinding or navigation)
  std::vector<Coordinate<T>> getNeighbors(Coordinate<T> from) const {
    std::vector<Coordinate<T>> neighbors;
    for (auto& cell : getNeighborCells(from)) {
      Coordinate<T> neighbor;
      cellToWorld(cell.cx, cell.cy, neighbor.x, neighbor.y);
      neighbors.push_back(neighbor);
    }
    return neighbors;
  }

  /// Define a custom validity callback (e.g., for dynamic obstacles or special
  /// terrain)
  void setValidityCallback(bool (*cb)(int cx, int cy, void* ref)) {
    is_valid_cb = cb;
  }

  void resize(int newXCount, int newYCount) {
    xCount = newXCount;
    yCount = newYCount;
    data.resize(xCount * yCount);
  }

  int getXCount() const { return xCount; }

  int getYCount() const { return yCount; }

  float getResolution() const { return resolution; }

  /**
   * @brief Check if a coordinate is within the map bounds.
   * @param coord The coordinate to check.
   * @return true if the coordinate is inside the map, false otherwise.
   */
  bool isValid(const Coordinate<T>& coord) const {
    int cx, cy;
    cx = static_cast<int>((coord.x - origin.x) / resolution);
    cy = static_cast<int>((coord.y - origin.y) / resolution);
    return cx >= 0 && cx < xCount && cy >= 0 && cy < yCount;
  }

 protected:
  // Grid parameters
  int xCount;            // Number of cells in x direction
  int yCount;            // Number of cells in y direction
  float resolution;      // Meters per cell
  Coordinate<T> origin;  // World coordinate of cell (0,0)
  bool (*is_valid_cb)(int cx, int cy, void*) =
      isValid;  // Optional callback for custom validity checks

  // Map data: e.g. 0=free, 100=occupied, -1=unknown
  std::vector<StateType, AllocatorPSRAM<StateType>> data;

  /// Default validity check: a cell is valid if it's not occupied. This can be
  /// overridden with a custom callback for more complex logic (e.g., dynamic
  /// obstacles, special terrain, etc.).
  static bool isValid(int cx, int cy, void* ref) {
    GridMap<StateType>* self = (GridMap<StateType>*)ref;
    StateType result;
    if (!self->getCell(cx, cy, result)) return false;  // Out of bounds
    return result != CellState::OCCUPIED;
  }
};

}  // namespace tinyrobotics

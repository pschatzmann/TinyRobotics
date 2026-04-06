#pragma once
#include <vector>

#include "IMap.h"
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/utils/AllocatorPSRAM.h"
#include "TinyRobotics/utils/Common.h"
#include <TinyRobotics/serialize/MapSerializer.h>

namespace tinyrobotics {

/**
 * @class GridMapBitVector
 * @ingroup maps
 * @brief A grid map using two bit vectors to represent CellState efficiently.
 *
 * The GridMapBitVector class models the environment as a regular grid of cells,
 * each storing a state (FREE, OCCUPIED, UNKNOWN) using two std::vector<bool>
 * for memory efficiency. So each cell occupies only 2 buts!
 *
 * CellState encoding:
 *   - 00: FREE
 *   - 01: OCCUPIED
 *   - 10: UNKNOWN
 *
 * @tparam T Numeric type for coordinates (default: float)
 */

template <typename T = DistanceM>
class GridBitMap : public IMap<T> {
 public:
  struct Cell {
    size_t cx;
    size_t cy;
  };

  GridBitMap() = default;
  GridBitMap(int xCount, int yCount, float resolutionM)
      : xCount(xCount), yCount(yCount), resolution(resolutionM) {
    resize(xCount, yCount);
  }
  GridBitMap(int xCount, int yCount, Distance resolution)
      : xCount(xCount), yCount(yCount) {
    this->resolution = resolution.getValue(DistanceUnit::M);
    resize(xCount, yCount);
  }

  void resize(int newXCount, int newYCount) {
    xCount = newXCount;
    yCount = newYCount;
    occupied.resize(xCount * yCount, false);
    free.resize(xCount * yCount, false);
  }

  int getXCount() const { return xCount; }

  int getYCount() const { return yCount; }

  float getResolution() const { return resolution; }

  // World to cell conversion
  bool worldToCell(float wx, float wy, Cell& cell) const {
    cell.cx = static_cast<int>((wx - origin.x) / resolution);
    cell.cy = static_cast<int>((wy - origin.y) / resolution);
    return (cell.cx >= 0 && cell.cx < xCount && cell.cy >= 0 &&
            cell.cy < yCount);
  }

  // Cell to world (center of cell)
  void cellToWorld(int cx, int cy, float& wx, float& wy) const {
    wx = origin.x + (cx + 0.5f) * resolution;
    wy = origin.y + (cy + 0.5f) * resolution;
  }

  // Get cell state by index
  bool getCell(int cx, int cy, CellState& result) const {
    if (cx < 0 || cx >= xCount || cy < 0 || cy >= yCount) return false;
    size_t idx = cy * xCount + cx;
    if (occupied[idx]) {
      result = CellState::OCCUPIED;
    } else if (free[idx]) {
      result = CellState::FREE;
    } else {
      result = CellState::UNKNOWN;
    }
    return true;
  }

  // Set cell state by index
  void setCell(int cx, int cy, CellState value) {
    if (cx < 0 || cx >= xCount || cy < 0 || cy >= yCount) return;
    size_t idx = cy * xCount + cx;
    switch (value) {
      case CellState::FREE:
        occupied[idx] = false;
        free[idx] = true;
        break;
      case CellState::OCCUPIED:
        occupied[idx] = true;
        free[idx] = false;
        break;
      case CellState::UNKNOWN:
        occupied[idx] = false;
        free[idx] = false;
        break;
    }
  }

  // Get cell state by coordinate
  bool getCell(const Coordinate<T>& coord, CellState& result) const {
    Cell cell;
    if (worldToCell(coord.x, coord.y, cell)) {
      return getCell(cell.cx, cell.cy, result);
    }
    return false;
  }

  // Set cell state by coordinate
  void setCell(const Coordinate<T>& coord, CellState value) {
    Cell cell;
    if (worldToCell(coord.x, coord.y, cell)) {
      setCell(cell.cx, cell.cy, value);
    }
  }

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

  /// Get world coordinates of neighboring cells (for pathfinding or navigation)
  std::vector<Coordinate<>> getNeighbors(Coordinate<T> from) const {
    std::vector<Coordinate<T>> neighbors;
    for (auto& cell : getNeighborCells(from)) {
      Coordinate<T> neighbor;
      cellToWorld(cell.cx, cell.cy, neighbor.x, neighbor.y);
      neighbors.push_back(neighbor);
    }
    return neighbors;
  }

  /// Convert cell indices to world coordinates (returns Coordinate<T>)
  Coordinate<T> toWorld(int cx, int cy) const override {
    float wx, wy;
    cellToWorld(cx, cy, wx, wy);
    return Coordinate<T>(wx, wy);
  }

  /// Write map to output
  size_t writeTo(Print& out) {
    return serializer.write(*this, out);
  }

  /// Read map from input
  size_t readFrom(Stream& in) {
    return serializer.read(*this, in);
  }


 protected:
  int xCount = 0;
  int yCount = 0;
  float resolution = 0.1f;
  Coordinate<T> origin;
  std::vector<bool> occupied;
  std::vector<bool> free;
  // Serialization
  GridMapSerializer<GridBitMap, CellState, T> serializer;

  /// Determine all neighboring cells (8-connected) for a given cell coordinate.
  std::vector<Cell> getNeighborCells(const Coordinate<T> from) const {
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
};

}  // namespace tinyrobotics

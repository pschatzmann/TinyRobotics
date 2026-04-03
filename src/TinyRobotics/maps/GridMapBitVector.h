#pragma once
#include <vector>
#include "TinyRobotics/utils/AllocatorPSRAM.h"
#include "TinyRobotics/utils/Common.h"
#include "TinyRobotics/coordinates/Coordinate.h"

namespace tinyrobotics {

/**
 * @class GridMapBitVector
 * @ingroup maps
 * @brief A grid map using two bit vectors to represent CellState efficiently.
 *
 * The GridMapBitVector class models the environment as a regular grid of cells, each
 * storing a state (FREE, OCCUPIED, UNKNOWN) using two std::vector<bool> for memory efficiency.
 * So each cell occupies only 2 buts!
 * 
 * CellState encoding:
 *   - 00: FREE
 *   - 01: OCCUPIED
 *   - 10: UNKNOWN
 *
 * @tparam T Numeric type for coordinates (default: float)
 */

class GridMapBitVector {
 public:
  struct Cell {
    size_t cx;
    size_t cy;
  };

  GridMapBitVector() = default;
  GridMapBitVector(int xCount, int yCount, float resolutionM)
      : xCount(xCount), yCount(yCount), resolution(resolutionM) {
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
    return (cell.cx >= 0 && cell.cx < xCount && cell.cy >= 0 && cell.cy < yCount);
  }

  // Cell to world (center of cell)
  void cellToWorld(int cx, int cy, float& wx, float& wy) const {
    wx = origin.x + (cx + 0.5f) * resolution;
    wy = origin.y + (cy + 0.5f) * resolution;
  }

  // Get cell state by index
  bool getCell(int cx, int cy, CellState& result) const {
    if (cx < 0 || cx >= xCount || cy < 0 || cy >= yCount)
      return false;
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
    if (cx < 0 || cx >= xCount || cy < 0 || cy >= yCount)
      return;
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
  bool getCell(const Coordinate<float>& coord, CellState& result) const {
    Cell cell;
    if (worldToCell(coord.x, coord.y, cell)) {
      return getCell(cell.cx, cell.cy, result);
    }
    return false;
  }

  // Set cell state by coordinate
  void setCell(const Coordinate<float>& coord, CellState value) {
    Cell cell;
    if (worldToCell(coord.x, coord.y, cell)) {
      setCell(cell.cx, cell.cy, value);
    }
  }

 protected:
  int xCount = 0;
  int yCount = 0;
  float resolution = 0.1f;
  Coordinate<float> origin;
  std::vector<bool> occupied;
  std::vector<bool> free;
};

} // namespace tinyrobotics

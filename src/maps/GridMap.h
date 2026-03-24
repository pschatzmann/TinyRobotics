#pragma once
#include <cmath>
#include <cstdint>
#include <vector>

#include "utils/Common.h"
#include "utils/AllocatorPSRAM.h"

namespace tinyrobotics {

/**
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
class GridMap {
 public:
  /// Cell structure to represent grid cell indices
  struct Cell {
    size_t cx;  // Cell X index
    size_t cy;  // Cell Y index
  };

  GridMap(int xCount, int yCount, DistanceM resolutionM,
          Coordinate<T> origin = Coordinate<T>(0, 0))
      : xCount(xCount),
        yCount(yCount),
        resolution(resolutionM),
        origin(origin),
        data(xCount * yCount, -1) {}

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

  /// Provide access to cell state by cell index
  StateType getCell(int cx, int cy) const {
    if (cx < 0 || cx >= xCount || cy < 0 || cy >= yCount)
      return -1;  // Out of bounds
    return data[cy * xCount + cx];
  }

  /// Provide access to cell state by coordinate
  StateType getCell(Coordinate<T>& coord) const {
    Cell cell;
    if (worldToCell(coord.x, coord.y, cell)) {
      return getCell(cell.cx, cell.cy);
    }
    return -1;  // Out of bounds
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

  /// Update with sensor reading (binary Bayesian update)
  void updateCell(int cx, int cy, bool occupied, float prob_hit = 0.7) {
    if (cx < 0 || cx >= xCount || cy < 0 || cy >= yCount) return;

    int idx = cy * xCount + cx;
    float current_prob = data[idx] / 100.0;

    // Convert to log-odds for efficient updates
    float log_odds = std::log(current_prob / (1.0 - current_prob));

    // Update based on measurement
    float meas_prob = occupied ? prob_hit : (1.0 - prob_hit);
    float meas_log_odds = std::log(meas_prob / (1.0 - meas_prob));
    log_odds += meas_log_odds;

    // Clamp and convert back
    float new_prob = 1.0 / (1.0 + std::exp(-log_odds));
    data[idx] = static_cast<int8_t>(std::round(new_prob * 100));
  }

  // /// Find all valid neighboring cells (8-connected) and return them as path
  // /// segments
  // std::vector<PathSegment<Coordinate>> getSegments(Coordinate& from) const {
  //   std::vector<PathSegment<Coordinate>> result;
  //   for (const auto& cell : getNeighborCells(from.cx, from.cy)) {
  //     if (is_valid_cb(cell.first, cell.second, this)) {
  //       Coordinate to(cell.first, cell.second);
  //       float distance = to.distance(from);
  //       result.push_back({from, to, distance, false});
  //     }
  //   }
  //   return result;
  // }

  // /// Find all valid neighboring cells (8-connected) and return them as path
  // /// segments
  // std::vector<PathSegment<Coordinate>> getSegments(Coordinate& from) const {
  //   std::vector<PathSegment<Coordinate>> result;
  //   for (const auto& cell : getNeighborCells(from.cx, from.cy)) {
  //     if (is_valid_cb(cell.first, cell.second, this)) {
  //       Coordinate to(cell.first, cell.second);
  //       float distance = to.distance(from);
  //       result.push_back({from, to, distance, false});
  //     }
  //   }
  //   return result;
  // }


  /// Define a custom validity callback (e.g., for dynamic obstacles or special
  /// terrain)
  void setValidityCallback(bool (*cb)(int cx, int cy, void* ref)) {
    is_valid_cb = cb;
  }

 protected:
  // Grid parameters
  int xCount;            // Number of cells in x direction
  int yCount;            // Number of cells in y direction
  float resolution;      // Meters per cell
  Coordinate<T> origin;  // World coordinate of cell (0,0)
  bool (*is_valid_cb)(int cx, int cy) =
      isValid;  // Optional callback for custom validity checks

  // Map data: e.g. 0=free, 100=occupied, -1=unknown
  std::vector<StateType, AllocatorPSRAM<StateType>> data;

  /// Default validity check: a cell is valid if it's not occupied. This can be
  /// overridden with a custom callback for more complex logic (e.g., dynamic
  /// obstacles, special terrain, etc.).
  static bool isValid(int cx, int cy, void* ref) {
    GridMap<StateType>* self = (GridMap<StateType>*)ref;
    return self->getCell(cx, cy) != CellState::OCCUPIED;
  }

  /// Determine all neighboring cells (8-connected) for a given cell coordinate.
  std::vector<Cell> getNeighborCells(int cx, int cy) const {
    std::vector<Cell> neighbors;
    if (cx < xCount - 1) neighbors.push_back({cx + 1, cy});
    if (cx > 0) neighbors.push_back({cx - 1, cy});
    if (cy < yCount - 1) neighbors.push_back({cx, cy + 1});
    if (cy > 0) neighbors.push_back({cx, cy - 1});

    if (cx < xCount - 1 && cy < yCount - 1)
      neighbors.push_back({cx + 1, cy + 1});
    if (cx > 0 && cy < yCount - 1) neighbors.push_back({cx - 1, cy + 1});
    if (cx < xCount - 1 && cy > 0) neighbors.push_back({cx + 1, cy - 1});
    if (cx > 0 && cy > 0) neighbors.push_back({cx - 1, cy - 1});
    return neighbors;
  }
};

}  // namespace tinyrobotics

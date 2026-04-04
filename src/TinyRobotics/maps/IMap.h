#pragma once
#include <vector>

#include "TinyRobotics/coordinates/Coordinate.h"

namespace tinyrobotics {

/**
 * @class IMapNeighbors
 * @ingroup maps
 * @brief Abstract interface for neighbor queries in map-like data structures.
 *
 * Provides a minimal interface for pathfinding and navigation algorithms to
 * query neighboring coordinates and check coordinate validity. Any map or graph
 * class that supports neighbor queries can implement this interface, enabling
 * use with generic planning algorithms (e.g., A*, Dijkstra).
 *
 * @tparam T Scalar type for coordinates and math (default: DistanceM)
 *
 * ### Required Methods
 * - getNeighbors(Coordinate<T> from): Return a vector of neighboring
 * coordinates for a given node.
 * - isValid(const Coordinate<T>& coord): Return true if the coordinate is valid
 * (e.g., inside map or graph).
 */
template <typename T = DistanceM>
class IMapNeighbors {
 public:
  /**
   * @brief Get world coordinates of neighboring cells (for pathfinding or
   * navigation).
   * @param from The cell coordinate to find neighbors for.
   * @return Vector of neighboring coordinates.
   */
  virtual std::vector<Coordinate<T>> getNeighbors(Coordinate<T> from) const = 0;

  /**
   * @brief Check if a coordinate is inside the map bounds.
   * @param coord The coordinate to check.
   * @return True if valid, false otherwise.
   */
  virtual bool isValid(const Coordinate<T>& coord) const = 0;
};

/**
 * @class IMap
 * @ingroup maps
 * @brief Abstract interface for 2D grid maps and occupancy maps in
 * TinyRobotics.
 *
 * IMap extends IMapNeighbors and provides a comprehensive interface for
 * grid-based or cell-based maps for frontier exploring. It supports cell
 * access, coordinate transforms, and neighbor queries, making it suitable for
 * pathfinding, navigation, and mapping algorithms. All grid or occupancy map
 * implementations (e.g., GridMap, GridBitMap) should inherit from this
 * interface and implement all pure virtual methods.
 *
 *
 * Features:
 *   - Query map dimensions and resolution
 *   - Access cell state (FREE, OCCUPIED, UNKNOWN) by index
 *   - Convert cell indices to world coordinates
 *   - Query neighbors and check coordinate validity (from IMapNeighbors)
 *
 * @tparam T Scalar type for coordinates and math (default: DistanceM)
 *
 * ### Required Methods
 * - getXCount(), getYCount(): Map dimensions in cells
 * - getResolution(): Cell size in meters
 * - getNeighbors(Coordinate<T> from): Neighboring cell coordinates for
 * pathfinding (from IMapNeighbors)
 * - isValid(const Coordinate<T>& coord): Check if a coordinate is inside the
 * map (from IMapNeighbors)
 * - getCell(int x, int y, CellState& state): Get cell state by index
 * - toWorld(int x, int y): Convert cell indices to world coordinates
 */

template <typename T = DistanceM>
class IMap : public IMapNeighbors<T> {
 public:
  /**
   * @brief Get the number of cells in the X direction.
   */
  virtual int getXCount() const = 0;

  /**
   * @brief Get the number of cells in the Y direction.
   */
  virtual int getYCount() const = 0;

  /**
   * @brief Get the map resolution (cell size in meters).
   */
  virtual float getResolution() const = 0;

  /**
   * @brief Get the state of a cell by integer indices.
   * @param x Cell X index
   * @param y Cell Y index
   * @param state Output: Cell state (FREE, OCCUPIED, UNKNOWN)
   * @return True if cell is valid, false otherwise.
   */
  virtual bool getCell(int x, int y, CellState& state) const = 0;

  /**
   * @brief Convert cell indices to world coordinates.
   * @param x Cell X index
   * @param y Cell Y index
   * @return World coordinate of the cell center.
   */
  virtual Coordinate<T> toWorld(int x, int y) const = 0;
};

}  // namespace tinyrobotics
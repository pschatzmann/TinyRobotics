#pragma once
#include <vector>
#include "TinyRobotics/coordinates/Coordinate.h"

/**
 * @class IMap
 * @ingroup maps
 * @brief Abstract interface for 2D grid maps in TinyRobotics.
 *
 * Provides a generic interface for occupancy grid maps, supporting cell access,
 * coordinate transforms, and neighbor queries. All map implementations (e.g., GridBitMap)
 * must inherit from this interface and implement all pure virtual methods.
 *
 * @tparam T Scalar type for coordinates and math (default: DistanceM)
 *
 * ### Required Methods
 * - getXCount(), getYCount(): Map dimensions in cells
 * - getResolution(): Cell size in meters
 * - getNeighbors(): Neighboring cell coordinates for pathfinding
 * - isValid(): Check if a coordinate is inside the map
 * - getCell(): Get cell state (FREE, OCCUPIED, UNKNOWN) by index
 * - toWorld(): Convert cell indices to world coordinates
 */
namespace tinyrobotics {

template <typename T = DistanceM>    
class IMap {
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
   * @brief Get world coordinates of neighboring cells (for pathfinding or navigation).
   * @param from The cell coordinate to find neighbors for.
   * @return Vector of neighboring coordinates.
   */
  virtual std::vector<Coordinate<T>> getNeighbors(Coordinate<T> from) const = 0;

  /**
   * @brief Check if a coordinate is inside the map bounds.
   * @param coord The coordinate to check.
   * @return True if valid, false otherwise.
   */
  virtual bool isValid(const Coordinate<float>& coord) const  = 0;

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

}
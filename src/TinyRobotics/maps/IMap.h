#pragma once
#include <vector>
#include "TinyRobotics/coordinates/Coordinate.h"

namespace tinyrobotics {

template <typename T = DistanceM>    
class IMap {
public:
  virtual int getXCount() const = 0;
  virtual int getYCount() const = 0;
  virtual bool getCell(int x, int y, CellState& state) const = 0;
  virtual Coordinate<T> toWorld(int x, int y) const = 0;

  virtual float getResolution() const = 0;

  /// Get world coordinates of neighboring cells (for pathfinding or navigation)
  virtual std::vector<Coordinate<T>> getNeighbors(Coordinate<T> from) const = 0;

  /// Check if a world coordinate is valid (within bounds and not an obstacle)
  virtual bool isValid(const Coordinate<float>& coord) const  = 0;
};

}
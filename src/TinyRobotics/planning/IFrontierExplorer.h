#pragma once
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/maps/IMap.h"
#include <cstddef>

namespace tinyrobotics {

/**
 * @interface IFrontierExplorer
 * @ingroup planning
 * @brief Interface for frontier-based exploration utilities.
 *
 * Provides a common interface for different frontier exploration strategies.
 *
 * @tparam T Scalar type for coordinates and map (e.g., float, DistanceM). Default: DistanceM.
 */
template <typename T=DistanceM>
class IFrontierExplorer {
public:
  virtual void setCurrentPosition(const Coordinate<T>& pos) = 0;
  virtual Coordinate<T> getCurrentPosition() const = 0;
  virtual bool getNextFrontier(Coordinate<T>& nextCell) = 0;
};

} // namespace tinyrobotics

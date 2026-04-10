#pragma once

#include "TinyRobotics/units/Distance.h"

namespace tinyrobotics {

/**
 * @class IObstacleDetector
 * @ingroup sensors
 * @brief Interface for obstacle detectors.
 *
 * Provides a callback registration method for obstacle detection events.
 */
class IObstacleDetector  {
 public:
  virtual ~IObstacleDetector() = default;

  /**
   * @brief Set a callback to be invoked when an obstacle is detected.
   * @param cb Callback function pointer with signature void(Distance, void*)
   * @param userData User-provided pointer passed to the callback
   */
  virtual void setObstacleDetectedCallback(void (*cb)(Distance, void*), void* userData) = 0;
};

} // namespace tinyrobotics

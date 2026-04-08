#pragma once
#include "TinyRobotics/units/Units.h"

namespace tinyrobotics {

/**
 * @brief Interface for speed sources.
 */
class ISpeedSource {
 public:
  virtual ~ISpeedSource() = default;
  /**
   * @brief Get the current speed.
   * @return Speed value (with units)
   */

  virtual Speed getSpeed(uint8_t motor = 0) const = 0;


  /// Publish actual speed for a specific motor
  virtual void setThrottlePercent(float value, uint8_t motor = 0) = 0;


  /// For sources with inertia, call this in your main loop with the elapsed time (in milliseconds) to update the speed estimate for a specific motor.
  virtual Speed updateSpeed(uint32_t deltaTimeMs, uint8_t motor = 0) = 0;

  virtual size_t getMotorCount() const = 0;
};

} // namespace tinyrobotics

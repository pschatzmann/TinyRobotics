#pragma once
#include <cmath>

#include "TinyRobotics/units/Units.h"
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/coordinates/Orientation3D.h"

namespace tinyrobotics {

/**
 * @brief Abstract interface for 3D odometry models.
 * Provides access to current linear and angular velocities for Odometry3D.
 */
class IOdometryModel3D : public MessageHandler {
 public:
  virtual ~IOdometryModel3D() = default;
  /**
   * @brief Register a callback to be invoked on relevant events (e.g., input change, update).
   * @param callback Function pointer with signature void callback(void* userData)
   * @param userData User-provided pointer passed to the callback
   */
  virtual void registerCallback(void (*callback)(void*), void* userData) {}
  /**
   * @brief Get the current linear velocity (vx, vy, vz) in m/s (robot frame).
   */
  virtual void getLinearVelocity(float& vx, float& vy, float& vz) const = 0;
  /**
   * @brief Get the current angular velocity (wx, wy, wz) in rad/s (robot frame).
   */
  virtual void getAngularVelocity(float& wx, float& wy, float& wz) const = 0;
};

} // namespace tinyrobotics

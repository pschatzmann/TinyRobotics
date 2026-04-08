#pragma once
#include "TinyRobotics/units/Units.h"

namespace tinyrobotics {

/**
 * @brief Abstract interface for 2D odometry models.
 */
class IOdometryModel2D : public MessageHandler {
 public:
  /**
   * @brief Virtual destructor for interface.
   */
  virtual ~IOdometryModel2D() = default;

  /**
   * @brief Get the current speed of the vehicle.
   * @return Speed of the vehicle (meters/second)
   */
  virtual Speed getSpeed() const = 0;

  /**
   * @brief Get the current steering angle of the vehicle.
   * @return Steering angle (radians)
   */
  virtual Angle getSteeringAngle() const = 0;

  /**
   * @brief Update the speed estimate based on elapsed time.
   * @param deltaTimeMs Time interval (milliseconds)
   */
  virtual void updateSpeed(uint32_t deltaTimeMs) = 0;

  /**
   * @brief Set the speed source (e.g., encoder, estimator) for this model.
   * @param speedSource Reference to an ISpeedSource implementation
   */
  virtual void setSpeedSource(ISpeedSource& speedSource) = 0;
  /**
   * @brief Register a callback to be invoked on relevant events (e.g., input change, update).
   * @param callback Function pointer with signature void callback(void* userData)
   * @param userData User-provided pointer passed to the callback
   */
  virtual void registerCallback(void (*callback)(void*), void* userData) = 0;
  /**
   * @brief Compute heading change (deltaTheta) for odometry kinematics.
   * @param speed Speed of the vehicle (meters/second)
   * @param steeringAngle Steering angle (radians)
   * @param deltaTimeMs Time interval (milliseconds)
   * @return Change in heading (radians)
   */
  virtual float computeDeltaTheta(uint16_t deltaTimeMs) const = 0;
  /**
   * @brief Compute position change (deltaX, deltaY) for odometry kinematics.
   * @param speed Speed of the vehicle (meters/second)
   * @param theta Heading (radians)
   * @param deltaTimeMs Time interval (milliseconds)
   * @param[out] deltaX Change in X position (meters)
   * @param[out] deltaY Change in Y position (meters)
   */
  virtual void computeDeltaXY(float theta, uint32_t deltaTimeMs, float& deltaX, float& deltaY) const = 0;
};

} // namespace tinyrobotics

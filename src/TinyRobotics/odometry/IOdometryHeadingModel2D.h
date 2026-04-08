#pragma once
#include "TinyRobotics/units/Units.h"

namespace tinyrobotics {

/**
 * @brief Abstract interface for 2D odometry models.
 */
class IOdometryHeadingModel2D {
 public:
  virtual ~IOdometryHeadingModel2D() = default;
  virtual void setSpeed(Speed speed)  = 0;
  virtual void setSpeed(Speed speedLeft, Speed speedRight)  = 0;
  virtual void setSteeringAngle(Angle angle) = 0;
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

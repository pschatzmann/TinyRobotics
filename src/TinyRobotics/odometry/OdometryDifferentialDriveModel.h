#pragma once
#include <cmath>

#include "TinyRobotics/odometry/IOdometryHeadingModel2D.h"
#include "TinyRobotics/units/Units.h"
#include "stdint.h"

namespace tinyrobotics {

/**
 * @class OdometryDifferentialDriveModel
 * @ingroup odometry
 * @brief Odometry model for differential drive robots.
 *
 * Computes heading change (deltaTheta) using left and right wheel speeds.
 */
class OdometryDifferentialDriveModel : public IOdometryHeadingModel2D {
 public:
  OdometryDifferentialDriveModel(Distance wheelBase) : wheelBase(wheelBase) {}

  void setSpeed(Speed left, Speed right) {
    leftSpeed = left;
    rightSpeed = right;
  }

  void setSpeed(Speed speed) {}

  /**
   * @brief Compute heading change (deltaTheta) for differential drive.
   * @param deltaTimeMs Time interval (milliseconds)
   * @return Change in heading (radians)
   */
  float computeDeltaTheta(uint16_t deltaTimeMs) const override {
    float wb = wheelBase.getValue(DistanceUnit::M);
    float vLeft = leftSpeed.getValue(SpeedUnit::MPS);
    float vRight = rightSpeed.getValue(SpeedUnit::MPS);
    float omega = (vRight - vLeft) / wb;
    return omega * static_cast<float>(deltaTimeMs) / 1000.0f;
  }

  void computeDeltaXY(float theta, float deltaTimeMs, float& deltaX,
                      float& deltaY) const override {
    // Differential drive: use average wheel speed for linear velocity
    float vLeft = leftSpeed.getValue(SpeedUnit::MPS);
    float vRight = rightSpeed.getValue(SpeedUnit::MPS);
    float v = 0.5f * (vLeft + vRight);
    float dt = deltaTimeMs / 1000.0f;
    deltaX = v * std::cos(theta) * dt;
    deltaY = v * std::sin(theta) * dt;
  }

 private:
  Distance wheelBase;
  Speed leftSpeed;
  Speed rightSpeed;
};

}  // namespace tinyrobotics

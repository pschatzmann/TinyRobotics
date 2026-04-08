#pragma once
#include <cmath>

#include "TinyRobotics/odometry/IOdometryModel2D.h"
#include "TinyRobotics/odometry/ISpeedSource.h"
#include "TinyRobotics/units/Units.h"

namespace tinyrobotics {

/**
 * @class OdometryModel2D
 * @ingroup odometry
 * @brief Odometry model for Ackermann, differential drive, and boat kinematics.
 *
 * This model computes the change in heading (theta) for:
 * - Ackermann steering vehicles (car-like, with nonzero wheelBase)
 * - Differential drive robots (wheelBase = 0, steeringAngle is angular
 * velocity)
 * - Boats (set wheelBase to the distance to the rudder; steeringAngle is rudder
 * angle)
 *
 * ## Kinematic Handling
 * - If wheelBase > 0: Uses Ackermann/boat kinematics: omega = v *
 * tan(steeringAngle) / wheelBase
 * - If wheelBase == 0: Uses differential drive convention: omega =
 * steeringAngle (interpreted as angular velocity)
 *
 * ## Boat Note
 * For boats, set wheelBase to the distance from the center of mass to the
 * rudder, and provide the rudder angle as steeringAngle.
 *
 * @author TinyRobotics contributors
 * @date 2026-04-07
 */
class OdometryModel2D : public IOdometryModel2D {
  void registerCallback(void (*callback)(void*), void* userData) override {}

 public:
  OdometryModel2D(Distance wheelBase) : wheelBase(wheelBase) {}

  void setSpeedSource(ISpeedSource& speedSource) override {
    p_speedSource = &speedSource;
  }

  void setSteeringAngle(Angle angle) {
    this->steeringAngle = angle;
    if (callback) {
      callback(userData);
    }
  }
  void setSpeed(Speed speed) {
    this->speed = speed;
    if (callback) {
      callback(userData);
    }
  }

  virtual void updateSpeed(uint32_t deltaTimeMs) {
    assert(p_speedSource != nullptr);
    speed = p_speedSource->updateSpeed(deltaTimeMs);
  }

  /**
   * @brief Compute heading change (deltaTheta) for Ackermann kinematics.
   * @param speed Speed of the vehicle (meters/second)
   * @param steeringAngle Steering angle (radians)
   * @param deltaTimeMs Time interval (milliseconds)
   * @return Change in heading (radians)
   */
  float computeDeltaTheta(uint16_t deltaTimeMs) const override {
    float wb = wheelBase.getValue(DistanceUnit::M);
    float speedMps = speed.getValue(SpeedUnit::MPS);
    float steeringAngleRad = steeringAngle.getValue(AngleUnit::RAD);
    float omega = (wb > 0.0f) ? speedMps * std::tan(steeringAngleRad) / wb
                              : steeringAngleRad;
    return omega * static_cast<float>(deltaTimeMs) / 1000.0f;
  }

  void computeDeltaXY(float theta, uint32_t deltaTimeMs, float& deltaX,
                      float& deltaY) const override {
    float speedMps = speed.getValue(SpeedUnit::MPS);
    float dt = deltaTimeMs / 1000.0f;
    deltaX = speedMps * std::cos(theta) * dt;
    deltaY = speedMps * std::sin(theta) * dt;
  }

  bool onMessage(const Message<float>& msg) {
    if (msg.origin != MessageOrigin::Vehicle) return false;
    //  update speed and steering angle from messages
    switch (msg.content) {
      case MessageContent::MotorSpeed:
        if (msg.unit != Unit::Percent) return false;
        p_speedSource->setThrottlePercent(msg.value);
        setSpeed(
            p_speedSource->getSpeed());  // Assuming single motor for simplicity
        return true;
      case MessageContent::SteeringAngle:
        steeringAngle = Angle(msg.value, msg.unit == Unit::AngleRadian ? AngleUnit::RAD : AngleUnit::DEG);
        return true;
      default:
        return false;  // Unhandled message content
    }
  }

  void setCallback(void (*callback)(void*), void* userData) {
    this->callback = callback;
    this->userData = userData;
  }

  Speed getSpeed() const override { return speed; }
  Angle getSteeringAngle() const override { return steeringAngle; }

 private:
  Distance wheelBase;
  Speed speed;
  Angle steeringAngle;
  ISpeedSource* p_speedSource = nullptr;;
  void (*callback)(void*) = nullptr;
  void* userData = nullptr;
};

}  // namespace tinyrobotics

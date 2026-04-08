#pragma once
#include <cmath>

#include "ISpeedSource.h"
#include "TinyRobotics/communication/Message.h"
#include "TinyRobotics/odometry/IOdometryModel2D.h"
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
class OdometryDifferentialDriveModel : public IOdometryModel2D {
 public:
  OdometryDifferentialDriveModel(Distance wheelBase) : wheelBase(wheelBase) {}

  virtual void updateSpeed(uint32_t deltaTimeMs) {
    leftSpeed = p_speedSource->updateSpeed(deltaTimeMs, 0);
    rightSpeed = p_speedSource->updateSpeed(deltaTimeMs, 1);
  }

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

  void computeDeltaXY(float theta, uint32_t deltaTimeMs, float& deltaX,
                      float& deltaY) const override {
    // Differential drive: use average wheel speed for linear velocity
    float vLeft = leftSpeed.getValue(SpeedUnit::MPS);
    float vRight = rightSpeed.getValue(SpeedUnit::MPS);
    float v = 0.5f * (vLeft + vRight);
    float dt = deltaTimeMs / 1000.0f;
    deltaX = v * std::cos(theta) * dt;
    deltaY = v * std::sin(theta) * dt;
  }
  bool onMessage(const Message<float>& msg) {
    if (msg.origin != MessageOrigin::Vehicle &&
        msg.origin != MessageOrigin::Motor)
      return false;
    //  update speed and steering angle from messages
    switch (msg.content) {
      case MessageContent::SteeringAngle:
        if (msg.unit != Unit::AngleRadian) return false;
        steeringAngle = Angle(msg.value, AngleUnit::RAD);
        return true;
      case MessageContent::MotorSpeed:
        if (msg.unit != Unit::Percent) return false;
        // differential motor with left = 0 and right = 1
        switch (msg.origin_id) {
          case 0:
            p_speedSource->setThrottlePercent(msg.value, 0);
            leftSpeed = p_speedSource->getSpeed(0);
            break;
          case 1:
            p_speedSource->setThrottlePercent(msg.value, 1);
            rightSpeed = p_speedSource->getSpeed(1);
            if (callback) {
              callback(userData);
            }
            break;
          default:
            // ignore other motors
            break;
        }
        return true;
      default:
        return false;  // Unhandled message content
    }
  }

  void setCallback(void (*callback)(void*), void* userData) {
    this->callback = callback;
    this->userData = userData;
  }

  void setSpeedSource(ISpeedSource& speedSource) {
    p_speedSource = &speedSource;
  }

  Speed getSpeed() const override { return (leftSpeed + rightSpeed) / 2; }
  Angle getSteeringAngle() const override { return steeringAngle; }

 private:
  Distance wheelBase;
  ISpeedSource* p_speedSource = nullptr;
  Speed leftSpeed;
  Speed rightSpeed;
  Angle steeringAngle;
  void (*callback)(void*) = nullptr;
  void* userData = nullptr;
};

}  // namespace tinyrobotics

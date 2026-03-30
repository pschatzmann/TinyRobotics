#pragma once
#include <cmath>
#include <vector>

#include "Arduino.h"  // for millis()
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/units/Units.h"

namespace tinyrobotics {

struct Delta2D {
  float dx;
  float dy;
  float dtheta;
};

/**
 * @class Odometry2D
 * @brief Tracks 2D position and orientation of a robot using velocity and
 * steering data.
 *
 * This class provides simple 2D odometry for mobile robots, such as
 * differential drive or Ackermann vehicles. It integrates velocity and steering
 * angle over time to estimate the robot's position (x, y) in meters.
 *
 * ## Supported Kinematics
 * - Differential drive (default model)
 * - Ackermann steering (if steering angle is provided)
 *
 * ## Inputs
 * - Speed (as a Speed object, e.g., meters per second)
 * - Steering angle (as an Angle object, e.g., radians)
 * - Time delta (in milliseconds, or uses millis() if not provided)
 *
 * ## Outputs
 * - 2D position (x, y) in meters
 *
 * ## Coordinate Frame
 * - All positions are in the robot's local/world frame, with x forward and y
 * left/right.
 *
 * ## Update Method
 * - Call update() with new speed and steering angle at each control loop
 * iteration.
 * - Optionally provide delta time, or let the class compute it using millis().
 *
 * ## Integration Method
 * - Uses simple Euler integration for position updates.
 *
 * ## Limitations
 * - Assumes no wheel slip or drift.
 * - Not suitable for holonomic or omnidirectional robots.
 * - Orientation (theta) is not explicitly tracked in this class.
 *
 * ## Example Usage
 * @code
 *   Odometry2D odom;
 *   odom.begin(Coordinate<DistanceM>(0, 0));
 *   // In your control loop:
 *   odom.update(currentSpeed, currentSteeringAngle);
 *   auto pos = odom.getPosition();
 *   Serial.printf("x=%.2f, y=%.2f\n", pos.x, pos.y);
 * @endcode
 *
 * @author TinyRobotics contributors
 * @date 2026-03-30
 */

class Odometry2D {
 public:
  Odometry2D() = default;

  bool begin(Coordinate<DistanceM> initialPosition, float initialTheta = 0.0f,
             Distance wheelBase = Distance()) {
    position = initialPosition;
    theta = initialTheta;
    totalDistance = 0.0f;
    lastUpdateTimeMs = 0;
    this->wheelBase = wheelBase;
    return true;
  }

  void update(Speed speed, Angle steeringAngle, float deltaTimeMs) {
    this->steeringAngle = steeringAngle;
    this->speed = speed;
    float speedMps = speed.getSpeed(SpeedUnit::MPS);
    float steeringAngleRad = steeringAngle.getAngle(AngleUnit::RAD);
    float deltaTheta = 0.0f;
    // Use Ackermann if wheelBase is set, else differential drive
    if (wheelBase.getDistance(DistanceUnit::M) > 0.0f) {
      float wb = wheelBase.getDistance(DistanceUnit::M);
      float omega = (wb > 0.0f) ? speedMps * tan(steeringAngleRad) / wb : 0.0f;
      deltaTheta = omega * deltaTimeMs / 1000.0f;
    } else {
      // Differential drive: steeringAngleRad is angular velocity
      deltaTheta = steeringAngleRad * deltaTimeMs / 1000.0f;
    }
    theta += deltaTheta;
    float deltaX = speedMps * cos(theta) * deltaTimeMs / 1000.0f;
    float deltaY = speedMps * sin(theta) * deltaTimeMs / 1000.0f;
    position.x += deltaX;
    position.y += deltaY;
    lastDelta = {deltaX, deltaY, deltaTheta};
    totalDistance += sqrt(deltaX * deltaX + deltaY * deltaY);
  }

  void update(Speed speed, Angle steeringAngle) {
    auto now = millis();
    float deltaTimeMs = now - lastUpdateTimeMs;
    if (lastUpdateTimeMs > 0) update(speed, steeringAngle, deltaTimeMs);
    lastUpdateTimeMs = now;
  }

  Coordinate<DistanceM> getPosition() const { return position; }
  float getTheta() const { return theta; }
  Angle getSteeringAngle() const { return steeringAngle; }
  Speed getSpeed() const { return speed; }
  float getLinearVelocity() const { return speed.getSpeed(SpeedUnit::MPS); }
  float getAngularVelocity() const {
    return steeringAngle.getAngle(AngleUnit::RAD);
  }
  float getTotalDistance() const { return totalDistance; }
  Delta2D getLastDelta() const { return lastDelta; }
  void setState(Coordinate<DistanceM> pos, float th) {
    position = pos;
    theta = th;
  }

 protected:
  Coordinate<float> position;
  float theta = 0.0f;
  float totalDistance = 0.0f;
  Delta2D lastDelta = {0.0f, 0.0f, 0.0f};  // (dx, dy, dtheta)
  uint32_t lastUpdateTimeMs = 0;
  Angle steeringAngle;
  Speed speed;
  Distance wheelBase;
};

}  // namespace tinyrobotics
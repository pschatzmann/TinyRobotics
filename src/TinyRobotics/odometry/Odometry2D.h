#pragma once
#include <cmath>
#include <vector>

#include "Arduino.h"  // for millis()
#include "TinyRobotics/control/MotionState2D.h"
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/units/Units.h"

namespace tinyrobotics {

/**
 * @class Odometry2D
 * @ingroup odometry
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

class Odometry2D : public IMotionState2D {
 public:
  Odometry2D() = default;
  Odometry2D(Distance wheelBase) { setWheelBase(wheelBase); }

  void setWheelBase(Distance wheelBase) { this->wheelBase = wheelBase; }

  /**
   * @brief Initialize the odometry state.
   *
   * @param initialPosition The starting position of the robot (x, y) in meters.
   * @param initialTheta The starting orientation (heading/yaw) in radians.
   * Default is 0.0.
   * @param wheelBase The wheelbase (distance between axles for Ackermann, or
   * length to rudder for boats) in meters. Default is 0 (differential drive).
   * @return true on success
   *
   * If wheelBase is set (>0), Ackermann or boat kinematics are used for heading
   * updates. If wheelBase is zero, differential drive kinematics are used.
   */
  bool begin(Coordinate<DistanceM> initialPosition, float initialTheta = 0.0f,
             Distance wheelBase = Distance()) {
    position = initialPosition;
    theta = initialTheta;
    totalDistance = 0.0f;
    lastUpdateTimeMs = 0;
    this->wheelBase = wheelBase;
    return true;
  }

  bool begin(Transform2D transform) {
    return begin(transform.pos, transform.getHeading(AngleUnit::RAD));
  }

  /**
   * @brief Initialize the odometry state from a Frame2D.
   *
   * @param frame The Frame2D containing the initial position and orientation.
   * @param wheelBase The wheelbase (optional, default 0).
   * @return true on success
   */
  bool begin(const Frame2D& frame) {
    // Extract position and orientation from the frame's transform
    auto tf = frame.getTransform();
    // tf.translation is a Coordinate<float>, convert to Coordinate<DistanceM>
    return begin(tf.pos, tf.getHeading(AngleUnit::RAD), wheelBase);
  }

  void end() {}

  /**
   * @brief Update the odometry state with new speed and steering angle.
   *
   * @param speed The current speed of the robot (with units).
   * @param steeringAngle The current steering angle (radians for
   * Ackermann/rudder, or angular velocity for differential drive).
   * @param deltaTimeMs Time since last update in milliseconds.
   *
   * If wheelBase is set (>0), uses Ackermann/boat kinematics for heading
   * update: omega = v * tan(steeringAngle) / wheelBase If wheelBase is zero,
   * uses differential drive kinematics: deltaTheta = steeringAngle (angular
   * velocity) * deltaTime
   */
  void update(Speed speed, Angle steeringAngle, float deltaTimeMs) {
    this->steeringAngle = steeringAngle;
    this->speed = speed;
    float speedMps = speed.getValue(SpeedUnit::MPS);
    float steeringAngleRad = steeringAngle.getValue(AngleUnit::RAD);
    float deltaTheta = 0.0f;
    // Use Ackermann if wheelBase is set, else differential drive
    if (wheelBase.getValue(DistanceUnit::M) > 0.0f) {
      float wb = wheelBase.getValue(DistanceUnit::M);
      float omega =
          (wb > 0.0f) ? speedMps * std::tan(steeringAngleRad) / wb : 0.0f;
      deltaTheta = omega * deltaTimeMs / 1000.0f;
    } else {
      // Differential drive: steeringAngleRad is angular velocity
      deltaTheta = steeringAngleRad * deltaTimeMs / 1000.0f;
    }
    theta += deltaTheta;
    // Normalize theta to [-pi, pi)
    theta = normalizeAngleRad(theta);
    float deltaX = speedMps * std::cos(theta) * deltaTimeMs / 1000.0f;
    float deltaY = speedMps * std::sin(theta) * deltaTimeMs / 1000.0f;
    position.x += deltaX;
    position.y += deltaY;
    lastDelta = {deltaX, deltaY, deltaTheta};
    totalDistance += std::sqrt(deltaX * deltaX + deltaY * deltaY);

    publish();
  }

  /**
   * @brief Update the odometry state with new speed and steering angle, using
   * automatic time delta.
   *
   * @param speed The current speed of the robot (with units).
   * @param steeringAngle The current steering angle (radians for
   * Ackermann/rudder, or angular velocity for differential drive).
   *
   * Uses millis() to compute the time since the last update.
   */
  void update(Speed speed, Angle steeringAngle) {
    auto now = millis();
    float deltaTimeMs = now - lastUpdateTimeMs;
    if (lastUpdateTimeMs > 0) update(speed, steeringAngle, deltaTimeMs);
    lastUpdateTimeMs = now;
  }

  /// @brief Get the current 2D position (meters)
  Coordinate<DistanceM> getPosition() const { return position; }
  /// @brief Get the current steering angle (radians or angular velocity)
  Angle getSteeringAngle() const { return steeringAngle; }
  /// @brief Get the current heading as an Angle (radians)
  Angle getHeading() const { return Angle(theta, AngleUnit::RAD); }
  /// @brief Get the current speed (meters/second)
  Speed getSpeed() const { return speed; }
  /// @brief Get the current orientation (radians)
  float getTheta() const { return theta; }
  /// @brief Get the current linear velocity (meters/second)
  float getLinearVelocity() const { return speed.getValue(SpeedUnit::MPS); }
  /// @brief Get the current angular velocity (radians/second)
  float getAngularVelocity() const {
    return steeringAngle.getValue(AngleUnit::RAD);
  }
  /// @brief Get the total distance traveled
  Distance getTotalDistance() const {
    return Distance(totalDistance, DistanceUnit::M);
  }
  /// @brief Get the last delta update (dx, dy, dtheta)
  Delta2D getLastDelta() const { return lastDelta; }

  /// @brief Set the odometry state (position and orientation)
  void setState(Coordinate<DistanceM> pos, float th) {
    position = pos;
    theta = th;
  }

  Transform2D getTransform() const {
    return Transform2D(position, getHeading().getValue(AngleUnit::DEG));
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

  void publish() {
    // Publish position as message
    Message<Coordinate<float>> msgPos{MessageContent::Position,
                                      Coordinate<float>(position.x, position.y),
                                      Unit::Meters};
    msgPos.origin = MessageOrigin::System;
    sendMessage(msgPos);

    // Publish heading as float (radians)
    Message<float> msgHeading{MessageContent::Heading, theta,
                              Unit::AngleRadian};
    msgHeading.origin = MessageOrigin::System;
    sendMessage(msgHeading);

    // Publish speed as float (meters/second)
    Message<float> msgSpeed{MessageContent::Speed,
                            speed.getValue(SpeedUnit::MPS),
                            Unit::MetersPerSecond};
    msgSpeed.origin = MessageOrigin::System;
    sendMessage(msgSpeed);
  }
};

}  // namespace tinyrobotics
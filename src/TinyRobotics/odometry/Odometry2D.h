#pragma once
#include <cmath>
#include <vector>

#include "Arduino.h"  // for millis()
#include "TinyRobotics/communication/MessageHandler.h"
#include "TinyRobotics/communication/MessageSource.h"
#include "TinyRobotics/control/MotionState2D.h"
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/odometry/OdometryModel2D.h"
#include "TinyRobotics/units/Units.h"

namespace tinyrobotics {

/**
 * @class Odometry2D
 * @ingroup odometry
 * @brief Tracks 2D position and orientation of a robot using velocity and
 * steering data from external sources.
 *
 * This class provides 2D odometry for mobile robots, such as differential drive
 * or Ackermann vehicles. It integrates velocity and steering angle over time to
 * estimate the robot's position (x, y) in meters.
 *
 * ## Supported Kinematics
 * - Differential drive (default model, via steering angle is provided, via
 * IOdometryModel2D)
 *
 * ## Inputs
 * - Speed (from an ISpeedSource, e.g., WheelEncoder)
 * - Steering angle (from messages or sensors)
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
 * - Call update() in your control loop to update odometry using the latest
 * speed and steering angle.
 * - Optionally provide delta time, or let the class compute it using millis().
 *
 * ## Integration Method
 * - Uses simple Euler integration for position updates.
 *
 * ## Construction
 * - Construct with references to an IMessageSource (vehicle), an ISpeedSource
 * (e.g., WheelEncoder), and an IOdometryModel2D (kinematics model).
 * - The vehicle is subscribed to receive messages for speed/throttle/steering
 * updates.
 *
 * ## Limitations
 * - Assumes no wheel slip or drift unless compensated by the speed source.
 * - Not suitable for holonomic or omnidirectional robots.
 * - Orientation (theta) is not explicitly tracked in this class.
 *
 * ## Example Usage
 * @code
 *   WheelEncoder encoder(...);
 *   OdometryModelAckerman model(...);
 *   Odometry2D odom(vehicle, encoder, model);
 *   odom.begin(Coordinate<DistanceM>(0, 0));
 *   // In your control loop:
 *   odom.update();
 *   auto pos = odom.getPosition();
 *   Serial.printf("x=%.2f, y=%.2f\n", pos.x, pos.y);
 * @endcode
 *
 * @author TinyRobotics contributors
 * @date 2026-03-30
 */

class Odometry2D : public IMotionState2D {
 public:
  Odometry2D(MessageSource& vehicle, ISpeedSource& speedSource,
             IOdometryModel2D& model)
      : vehicle(vehicle), speedSource(speedSource), model(model) {
    model.setSpeedSource(speedSource);
    vehicle.subscribe(model);
    model.registerCallback(
        [](void* userData) {
          Odometry2D* odometry = static_cast<Odometry2D*>(userData);
          odometry->update();
        },
        this);
  }

  /**
   * @brief Initialize the odometry state.
   *
   * @param initialPosition The starting position of the robot (x, y) in meters.
   * @param initialTheta The starting orientation (heading/yaw) in radians.
   * Default is 0.0.
   * @return true on success
   */
  bool begin(Coordinate<DistanceM> initialPosition, float initialTheta = 0.0f) {
    position = initialPosition;
    theta = initialTheta;
    totalDistance = 0.0f;
    lastUpdateTimeMs = 0;
    return true;
  }

  bool begin(Transform2D transform) {
    return begin(transform.pos, transform.getHeading(AngleUnit::RAD));
  }

  /**
   * @brief Initialize the odometry state from a Frame2D.
   *
   * @param frame The Frame2D containing the initial position and orientation.
   * @return true on success
   */
  bool begin(const Frame2D& frame) {
    // Extract position and orientation from the frame's transform
    auto tf = frame.getTransform();
    // tf.translation is a Coordinate<float>, convert to Coordinate<DistanceM>
    return begin(tf.pos, tf.getHeading(AngleUnit::RAD));
  }

  void end() {}

  /**
   * @brief Update the odometry state with new speed and steering angle
   * Call this method in your control loop
   */
  void update() {
    auto now = millis();
    float deltaTimeMs = now - lastUpdateTimeMs;
    if (lastUpdateTimeMs > 0) update(deltaTimeMs);
    lastUpdateTimeMs = now;
  }

  /// @brief Get the current 2D position (meters)
  Coordinate<DistanceM> getPosition() const { return position; }
  /// @brief Get the current steering angle (radians or angular velocity)
  Angle getSteeringAngle() const { return model.getSteeringAngle(); }
  /// @brief Get the current heading as an Angle (radians)
  Angle getHeading() const { return Angle(theta, AngleUnit::RAD); }
  /// @brief Get the current speed (meters/second)
  Speed getSpeed() const { return model.getSpeed(); }
  /// @brief Get the current orientation (radians)
  float getTheta() const { return theta; }
  /// @brief Get the current linear velocity (meters/second)
  float getLinearVelocity() const {
    return getSpeed().getValue(SpeedUnit::MPS);
  }
  /// @brief Get the current angular velocity (radians/second)
  float getAngularVelocity() const {
    return getSteeringAngle().getValue(AngleUnit::RAD);
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
  IOdometryModel2D& model;
  ISpeedSource& speedSource;
  MessageSource& vehicle;
  bool is_differential = false;

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
    Message<float> msgSpeed{MessageContent::Speed, getSpeed().getValue(SpeedUnit::MPS),
                            Unit::MetersPerSecond};
    msgSpeed.origin = MessageOrigin::System;
    sendMessage(msgSpeed);
  }

  /**
   * @brief Update the odometry state with new speed and steering angle.
   *
   * @param speed The current speed of the robot (with units).
   * @param steeringAngle The current steering angle (radians for
   * Ackermann/rudder, or angular velocity for differential drive).
   * @param deltaTimeMs Time since last update in milliseconds.
   *
   */
  void update(uint32_t deltaTimeMs) {
    // Update speed from the speed source (if it has inertia)
    model.updateSpeed(deltaTimeMs);
    float steeringAngleRad = model.getSteeringAngle().getValue(AngleUnit::RAD);
    float deltaTheta = 0.0f;
    deltaTheta = model.computeDeltaTheta(deltaTimeMs);
    theta += deltaTheta;
    // Normalize theta to [-pi, pi)
    theta = normalizeAngleRad(theta);
    float deltaX = 0.0f, deltaY = 0.0f;
    model.computeDeltaXY(theta, deltaTimeMs, deltaX, deltaY);
    position.x += deltaX;
    position.y += deltaY;
    lastDelta = {deltaX, deltaY, deltaTheta};
    totalDistance += std::sqrt(deltaX * deltaX + deltaY * deltaY);

    publish();
  }
};

}  // namespace tinyrobotics
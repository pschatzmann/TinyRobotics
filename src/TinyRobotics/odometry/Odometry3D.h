#pragma once
#include <cmath>
#include <vector>

#include "Arduino.h"  // for millis()
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/coordinates/Orientation3D.h"
#include "TinyRobotics/units/Units.h"

namespace tinyrobotics {

/**
 * @class Odometry3D
 * @ingroup odometry
 * @brief Tracks 3D position and orientation of a robot using velocity and
 * angular rates.
 *
 * This class provides simple 3D odometry for mobile robots, such as drones or
 * underwater vehicles. It integrates linear velocity and angular velocity over
 * time to estimate the robot's position (x, y, z) and orientation (yaw, pitch,
 * roll) in meters and radians.
 *
 * ## Inputs
 * - Linear velocity (vx, vy, vz) in m/s (robot frame)
 * - Angular velocity (wx, wy, wz) in rad/s (robot frame)
 * - Time delta (in milliseconds, or uses millis() if not provided)
 *
 * ## Outputs
 * - 3D position (x, y, z) in meters
 * - 3D orientation (yaw, pitch, roll) in radians
 *
 * ## Limitations
 * - Assumes no slip or drift
 * - Subject to integration drift over time
 *
 * @author TinyRobotics contributors
 * @date 2026-03-31
 */
class Odometry3D {
 public:
  Odometry3D() = default;

  /**
   * @brief Initialize the odometry state.
   * @param initialPosition The starting position (x, y, z) in meters.
   * @param initialOrientation The starting orientation (yaw, pitch, roll) in
   * radians.
   * @return true on success
   */
  bool begin(Coordinate<float> initialPosition = {0, 0, 0},
             Orientation3D initialOrientation = Orientation3D()) {
    position = initialPosition;
    orientation = initialOrientation;
    totalDistance = 0.0f;
    lastUpdateTimeMs = 0;
    return true;
  }

  /**
   * @brief Update the odometry state with new velocities and angular rates.
   * @param vx Linear velocity in x (m/s, robot frame)
   * @param vy Linear velocity in y (m/s, robot frame)
   * @param vz Linear velocity in z (m/s, robot frame)
   * @param wx Angular velocity in x (rad/s, robot frame)
   * @param wy Angular velocity in y (rad/s, robot frame)
   * @param wz Angular velocity in z (rad/s, robot frame)
   * @param deltaTimeMs Time since last update in milliseconds
   */
  void update(float vx, float vy, float vz, float wx, float wy, float wz,
              uint32_t deltaTimeMs) {
    float dt = static_cast<float>(deltaTimeMs) / 1000.0f;
    // Integrate orientation (Euler angles, simple integration)
    orientation.roll += wx * dt;
    orientation.pitch += wy * dt;
    orientation.yaw += wz * dt;
    orientation.wrap();
    // Rotate velocity to world frame (using current orientation)
    float cr = std::cos(orientation.roll), sr = std::sin(orientation.roll);
    float cp = std::cos(orientation.pitch), sp = std::sin(orientation.pitch);
    float cy = std::cos(orientation.yaw), sy = std::sin(orientation.yaw);
    // Rotation matrix (ZYX convention)
    float vx_world = cy * cp * vx + (cy * sp * sr - sy * cr) * vy +
                     (cy * sp * cr + sy * sr) * vz;
    float vy_world = sy * cp * vx + (sy * sp * sr + cy * cr) * vy +
                     (sy * sp * cr - cy * sr) * vz;
    float vz_world = -sp * vx + cp * sr * vy + cp * cr * vz;
    // Integrate velocity for position
    float dx = vx_world * dt;
    float dy = vy_world * dt;
    float dz = vz_world * dt;
    position.x += dx;
    position.y += dy;
    position.z += dz;
    totalDistance += std::sqrt(dx * dx + dy * dy + dz * dz);
    lastDelta = Distance3D(dx, dy, dz, DistanceUnit::M);
  }

  void update(Velocity3D velocity, AngularVelocity3D angularVelocity,
              uint32_t deltaTimeMs) {
    update(velocity.getX(SpeedUnit::MPS), velocity.getY(SpeedUnit::MPS),
           velocity.getZ(SpeedUnit::MPS),
           angularVelocity.getX(AngularVelocityUnit::RadPerSec),
           angularVelocity.getY(AngularVelocityUnit::RadPerSec),
           angularVelocity.getZ(AngularVelocityUnit::RadPerSec), deltaTimeMs);
  }

  /**
   * @brief Update with automatic time delta (uses millis()).
   */
  void update(float vx, float vy, float vz, float wx, float wy, float wz) {
    uint32_t now = millis();
    uint32_t deltaTimeMs = now - lastUpdateTimeMs;
    if (lastUpdateTimeMs > 0) update(vx, vy, vz, wx, wy, wz, deltaTimeMs);
    lastUpdateTimeMs = now;
  }

  void update(Velocity3D velocity, AngularVelocity3D angularVelocity) {
    update(velocity.getX(SpeedUnit::MPS), velocity.getY(SpeedUnit::MPS),
           velocity.getZ(SpeedUnit::MPS),
           angularVelocity.getX(AngularVelocityUnit::RadPerSec),
           angularVelocity.getY(AngularVelocityUnit::RadPerSec),
           angularVelocity.getZ(AngularVelocityUnit::RadPerSec));
  }

  /// @brief Get the current 3D position (meters)
  Coordinate<float> getPosition() const { return position; }
  /// @brief Get the current orientation as Orientation3D (yaw, pitch, roll in
  /// radians)
  Orientation3D getOrientation() const { return orientation; }
  /// @brief Get the total distance traveled
  Distance getTotalDistance() const {
    return Distance(totalDistance, DistanceUnit::M);
  }
  /// @brief Get the last delta update (dx, dy, dz)
  Distance3D getLastDelta() const { return lastDelta; }

 protected:
  Coordinate<float> position;
  Orientation3D orientation;
  float totalDistance = 0.0f;
  Distance3D lastDelta = Distance3D(0.0f, 0.0f, 0.0f, DistanceUnit::M);
  uint32_t lastUpdateTimeMs = 0;
};

}  // namespace tinyrobotics

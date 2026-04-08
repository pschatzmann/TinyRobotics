#pragma once
#include <cmath>
#include <vector>

#include "Arduino.h"  // for millis()
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/coordinates/Orientation3D.h"
#include "TinyRobotics/units/Units.h"
#include "TinyRobotics/odometry/IOdometryModel3D.h"

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
  Odometry3D(MessageSource& vehicle, IOdometryModel3D& model)
    : vehicle(vehicle), model(model) {
      vehicle.subscribe(model);  // Subscribe to model messages if needed
      model.registerCallback([](void* userData) {
        Odometry3D* odometry = static_cast<Odometry3D*>(userData);
        odometry->update();
      }, this);
    }

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
   */
  void update() {
    uint32_t now = millis();
    uint32_t deltaTimeMs = now - lastUpdateTimeMs;
    if (lastUpdateTimeMs > 0) {
      float vx, vy, vz, wx, wy, wz;
      model.getLinearVelocity(vx, vy, vz);
      model.getAngularVelocity(wx, wy, wz);
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
    lastUpdateTimeMs = now;
  }


  // Remove all parameterized update methods for clarity and consistency

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
  MessageSource& vehicle;
  IOdometryModel3D& model;
  Coordinate<float> position;
  Orientation3D orientation;
  float totalDistance = 0.0f;
  Distance3D lastDelta = Distance3D(0.0f, 0.0f, 0.0f, DistanceUnit::M);
  uint32_t lastUpdateTimeMs = 0;
};

}  // namespace tinyrobotics

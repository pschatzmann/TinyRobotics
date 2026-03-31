#pragma once
#include <math.h>
#include <stdint.h>

#include "TinyRobotics/communication/Message.h"
#include "TinyRobotics/communication/MessageHandler.h"
#include "TinyRobotics/communication/MessageSource.h"
#include "TinyRobotics/control/MotionState2D.h"
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/coordinates/Orientation3D.h"
#include "TinyRobotics/units/Speed.h"
#include "TinyRobotics/units/Distance.h"
#include "TinyRobotics/units/AngularVelocity.h"

namespace tinyrobotics {

/**
 * @brief Basic 3D IMU dead-reckoning class.
 *
 * This class estimates 3D position, velocity, and orientation (yaw, pitch,
 * roll) for robotics applications by integrating gyroscope and accelerometer
 * data over time.
 *
 * - Integrates gyro (roll, pitch, yaw rates) to estimate orientation (radians)
 * - Integrates accelerometer (X, Y, Z) in the robot frame, rotated to world
 * frame, to estimate velocity and position
 * - All units SI (meters, seconds, radians)
 *
 * Limitations:
 * - No sensor fusion with magnetometer or GPS
 * - No Kalman filter or bias estimation
 * - Subject to drift over time (no correction)
 *
 * @tparam T Numeric type (float or double recommended)
 */
template <typename T = float>
class IMU3D : public MessageSource {
 public:
  IMU3D() = default;

  bool begin(const Coordinate<T>& initialPosition = {0, 0, 0},
             Orientation3D initialOrientation = Orientation3D()) {
    position = initialPosition;
    orientation.yaw = initialOrientation.yaw;
    orientation.pitch = initialOrientation.pitch;
    orientation.roll = initialOrientation.roll;
  speed = Speed3D(0.0f, 0.0f, 0.0f, SpeedUnit::MPS);
    lastUpdateMillis = 0;
    is_active = true;
    return true;
  }

  void end() { is_active = false; }

  /// Update with accelerometer and gyro measurements
  /// accelX, accelY, accelZ in sensor frame (m/s^2)
  /// gyroX, gyroY, gyroZ in rad/s (roll, pitch, yaw rates)
  void update(T accelX, T accelY, T accelZ, T gyroX, T gyroY, T gyroZ,
              unsigned long nowMillis) {
    if (!is_active) return;
    if (lastUpdateMillis == 0) {
      lastUpdateMillis = nowMillis;
      return;
    }
    T dt = (nowMillis - lastUpdateMillis) / (T)1000;
    lastUpdateMillis = nowMillis;
    if (dt <= 0) return;

    // Integrate gyro for orientation (Euler angles, simple integration)
    orientation.roll += gyroX * dt;
    orientation.pitch += gyroY * dt;
    orientation.yaw += gyroZ * dt;
    orientation.wrap();

    // Rotate acceleration to world frame (using current orientation)
    // Rotation order: roll (X), pitch (Y), yaw (Z)
    T cr = cos(orientation.roll), sr = sin(orientation.roll);
    T cp = cos(orientation.pitch), sp = sin(orientation.pitch);
    T cy = cos(orientation.yaw), sy = sin(orientation.yaw);
    // Rotation matrix (ZYX convention)
    T ax_world = cy * cp * accelX + (cy * sp * sr - sy * cr) * accelY +
                 (cy * sp * cr + sy * sr) * accelZ;
    T ay_world = sy * cp * accelX + (sy * sp * sr + cy * cr) * accelY +
                 (sy * sp * cr - cy * sr) * accelZ;
    T az_world = -sp * accelX + cp * sr * accelY + cp * cr * accelZ;

    // Integrate acceleration for velocity
    speed = Speed3D(
      speed.getX(SpeedUnit::MPS) + ax_world * dt,
      speed.getY(SpeedUnit::MPS) + ay_world * dt,
      speed.getZ(SpeedUnit::MPS) + az_world * dt,
      SpeedUnit::MPS);

    // Integrate velocity for position
  float dx = speed.getX(SpeedUnit::MPS) * dt;
  float dy = speed.getY(SpeedUnit::MPS) * dt;
  float dz = speed.getZ(SpeedUnit::MPS) * dt;
  position.x += dx;
  position.y += dy;
  position.z += dz;

  lastDelta = Distance3D(dx, dy, dz, DistanceUnit::M);
  lastAngularVelocity = AngularVelocity3D(static_cast<float>(gyroX), static_cast<float>(gyroY), static_cast<float>(gyroZ), AngularVelocityUnit::RadPerSecond);
    publish();
  }

  /// @brief Get the last delta update (dx, dy, dz)
  Distance3D getLastDelta() const { return lastDelta; }
  /// @brief Get the last angular velocity (rad/s)
  AngularVelocity3D getLastAngularVelocity() const {
    return lastAngularVelocity;
  }
  /// @brief Get the current linear velocity (m/s)
  Speed3D getLinearVelocity() const { return speed; }
  /// @brief Get the current orientation as Orientation3D (yaw, pitch, roll in
  /// radians)
  Orientation3D getOrientation() const { return orientation; }
  /// @brief Get the current position
  Coordinate<T> getPosition() const { return position; }

 protected:
  bool is_active = false;
  Orientation3D orientation;
  Speed3D speed = Speed3D(0.0f, 0.0f, 0.0f, SpeedUnit::MPS);
  Coordinate<T> position;
  unsigned long lastUpdateMillis = 0;
  Distance3D lastDelta = Distance3D(0.0f, 0.0f, 0.0f, DistanceUnit::M);
  AngularVelocity3D lastAngularVelocity = AngularVelocity3D(0.0f, 0.0f, 0.0f, AngularVelocityUnit::RadPerSecond);

  void publish() {
    // Publish position as message
    Message<Coordinate<T>> msgPos{MessageContent::Position, position,
                                  Unit::Meters};
    msgPos.source = MessageOrigin::IMU;
    sendMessage(msgPos);
    // Publish orientation as float[3] (yaw, pitch, roll)
    // (You may want to define a custom message type for 3D orientation)
    // Publish velocity as message
    // (You may want to define a custom message type for 3D velocity)
  }
};

}  // namespace tinyrobotics

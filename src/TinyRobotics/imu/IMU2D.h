#include <TinyRobotics/arduino/Arduino.h>
using ::millis;
#pragma once
#include <math.h>
#include <stdint.h>

#include "TinyRobotics/communication/Message.h"
#include "TinyRobotics/communication/MessageHandler.h"
#include "TinyRobotics/communication/MessageSource.h"
#include "TinyRobotics/control/MotionState2D.h"
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/coordinates/GPSCoordinate.h"

namespace tinyrobotics {

/**
 * @class IMU2D
 * @ingroup imu
 * @brief Basic 2D IMU dead-reckoning class.
 *
 * This class estimates 2D position, velocity, and heading (yaw angle) for
 * robotics applications by integrating gyroscope and accelerometer data over
 * time.
 *
 * - Integrates gyro Z (yaw rate) to estimate heading (radians)
 * - Integrates accelerometer (X, Y) in the robot frame, rotated to world frame,
 * to estimate velocity and position
 * - All units SI (meters, seconds, radians)
 *
 * Limitations:
 * - No sensor fusion with magnetometer or GPS
 * - No Kalman filter or bias estimation
 * - Subject to drift over time (no correction)
 *
 * Example usage:
 * @code
 *   IMU2D<float> imu;
 *   imu.begin(initialAngle, initialPosition);
 *   imu.update(accelX, accelY, gyroZ, millis());
 *   auto pos = imu.getPosition();
 *   auto heading = imu.getHeading();
 *   auto speed = imu.getSpeed();
 * @endcode
 *
 * @tparam T Numeric type (float or double recommended)
 */

template <typename T = float>
class IMU2D : public MessageSource, public IMotionState2D {
 public:
  IMU2D() = default;

  bool begin(T initialAngleDeg, Coordinate<DistanceM> initialPosition) {
    position = initialPosition;
    headingRad = initialAngleDeg * M_PI / 180.0;
    speedMPS = 0;
    lastUpdateMillis = 0;
    is_active = true;
    return true;
  }

  void end() { is_active = false; }

  /// Update with accelerometer and gyro measurements (accelX, accelY in sensor
  /// frame, gyroZ in rad/s)
  void update(T accelX, T accelY, T gyroZ_in, unsigned long nowMillis) {
    if (!is_active) return;
    if (lastUpdateMillis == 0) {
      lastUpdateMillis = nowMillis;
      return;
    }

    T dt = (nowMillis - lastUpdateMillis) / (T)1000;
    lastUpdateMillis = nowMillis;
    if (dt <= 0) return;

    // Integrate gyro for heading
    headingRad += gyroZ_in * dt;
    // Angle wrap
    while (headingRad > M_PI) headingRad -= 2 * M_PI;
    while (headingRad < -M_PI) headingRad += 2 * M_PI;

    // Rotate acceleration to world frame
    T ax_world = accelX * cos(headingRad) - accelY * sin(headingRad);
    T ay_world = accelX * sin(headingRad) + accelY * cos(headingRad);

    // Integrate acceleration for velocity (simple dead-reckoning)
    static T vx = 0, vy = 0;
    vx += ax_world * dt;
    vy += ay_world * dt;
    speedMPS = sqrt(vx * vx + vy * vy);

    // Integrate velocity for position
    T dx = vx * dt;
    T dy = vy * dt;

    position.x += dx;
    position.y += dy;
    totalDistanceM += sqrt(dx * dx + dy * dy);

    lastDelta = {static_cast<float>(dx), static_cast<float>(dy),
                 static_cast<float>(gyroZ_in * dt)};
    lastAngularVelocity = gyroZ_in;
    timestamp = millis();
    publish();
  }

  /// @brief Get the last delta update (dx, dy, dtheta)
  Delta2D getLastDelta() const { return lastDelta; }
  /// @brief Get the current angular velocity (radians/second)
  float getAngularVelocity() const { return lastAngularVelocity; }
  /// @brief Get the current linear velocity (meters/second)
  float getLinearVelocity() const { return speedMPS; }
  /// @brief Get the current orientation (radians)
  float getTheta() const { return headingRad; }

  /// get position
  Coordinate<DistanceM> getPosition() const { return position; }
  /// get heading as Angle
  Angle getHeading() const { return Angle(headingRad, AngleUnit::RAD); }
  /// get speed as Speed
  Speed getSpeed() const { return Speed(speedMPS, SpeedUnit::MPS); }
  ///  Get the total distance traveled
  Distance getTotalDistance() const { return Distance(totalDistanceM,DistanceUnit::M); }
  /// Get time of last update
  uint32_t getTimestamp() const { return timestamp; }

 public:
  bool is_active = false;
  float headingRad = 0;
  float speedMPS = 0;
  Coordinate<DistanceM> position;
  unsigned long lastUpdateMillis = 0;
  Delta2D lastDelta = {0.0f, 0.0f, 0.0f};
  float lastAngularVelocity = 0.0f;
  float totalDistanceM = 0.0f;
  uint32_t timestamp = 0;

  void publish() {
    // Publish position as message
    Message<Coordinate<DistanceM>> msgPos{MessageContent::Position, position,
                                          Unit::Meters};
    msgPos.source = MessageOrigin::IMU;
    sendMessage(msgPos);

    // Publish heading as float (radians)
    Message<float> msgHeading{MessageContent::Heading, headingRad,
                              Unit::AngleRadian};
    msgHeading.source = MessageOrigin::IMU;
    sendMessage(msgHeading);

    // Publish speed as float (m/s)
    Message<float> msgSpeed{MessageContent::Speed, speedMPS,
                            Unit::MetersPerSecond};
    msgSpeed.source = MessageOrigin::IMU;
    sendMessage(msgSpeed);
  }
};

}  // namespace tinyrobotics
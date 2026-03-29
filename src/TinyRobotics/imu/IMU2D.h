#pragma once
#include <math.h>
#include <stdint.h>

#include "TinyRobotics/communication/Message.h"
#include "TinyRobotics/communication/MessageHandler.h"
#include "TinyRobotics/communication/MessageSource.h"
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/coordinates/GPSCoordinate.h"
#include "TinyRobotics/control/KalmanFilter.h"
#undef F

namespace tinyrobotics {

/**
 * @brief 2D IMU sensor fusion class using an Extended Kalman Filter (EKF).
 *
 * This class fuses gyroscope, accelerometer, magnetometer, and GPS data to
 * estimate 2D position, velocity, and heading (yaw angle) for robotics
 * applications. It uses an EKF with a 6D state vector:
 *
 *   [x, y, vx, vy, angle, gyro_bias]
 *
 * - x, y: position (meters)
 * - vx, vy: velocity (m/s)
 * - angle: heading (radians)
 * - gyro_bias: estimated gyro Z bias (rad/s)
 *
 * The process model integrates acceleration and gyro rate, while the
 * measurement model incorporates magnetometer and GPS updates. Angle
 * integration is handled by the EKF state.
 *
 * Features:
 * - Robust sensor fusion for position, velocity, and heading
 * - Online gyro bias estimation
 * - Angle wrapping for heading
 * - All units SI (meters, seconds, radians)
 *
 * Example usage:
 * @code
 *   tinyrobotics::IMU2D<float> imu;
 *   imu.begin(initialAngle, initialPosition);
 *   imu.update(accelX, accelY, gyroZ, millis());
 *   imu.updateMagnetometer(magX, magY);
 *   imu.updateGPS(gpsCoord, millis());
 *   // publish and/or get estimates
 *   imu.publish();
 *   auto pos = imu.getPosition();
 *   auto vel = imu.getVelocity();
 *   float heading = imu.getHeadingAngleRad();
 * @endcode
 *
 * @tparam T Numeric type (float or double recommended)
 */

template <typename T = float>
class IMU2D : public MessageSource {
  struct Vec2 {
    T x = 0, y = 0;
  };

 public:
  IMU2D() = default;

  /**
   * @brief Calibrate the gyroscope Z-axis bias (zero-rate offset).
   *
   * Gyroscopes often have a small offset (bias) even when stationary, which
   * causes the integrated angle to drift over time. This method should be
   * called while the device is stationary to record the current gyro Z reading
   * as the bias. The bias will then be subtracted from all future gyro readings
   * in the update() method, reducing drift and improving long-term angle
   * accuracy.
   *
   * Typical usage:
   * @code
   *   imu.calibrateGyro(currentGyroZ); // Call when device is stationary
   * @endcode
   *
   * @param g The measured gyro Z value (rad/s) when stationary.
   *
   * @note Call this method before normal operation, ideally after power-on or
   * reset, and ensure the device is not rotating during calibration.
   */
  void calibrateGyro(T g) { gyroBias = g; }

  /**
   * @brief Initialize the IMU2D EKF state with a known heading and position.
   *
   * This method sets the initial orientation (heading, in radians) and position
   * (in meters) of the IMU. It should be called at startup, after sensor
   * calibration, to ensure all internal state variables are referenced to a
   * common global frame (e.g., north and world coordinates).
   *
   * The EKF state vector is initialized as:
   *   [x, y, vx, vy, angle, bias]
   *
   * @param initialAngle Initial heading in Degrees (relative to north, e.g.,
   * from magnetometer)
   * @param initialPosition Initial position in world or base frame coordinates
   * (meters)
   * @return true if initialization was successful
   *
   * @note Call this before the first update() to ensure consistent sensor
   * fusion.
   *
   * Example usage:
   * @code
   *   imu.begin(initialHeading, initialPosition);
   * @endcode
   */
  bool begin(T initialAngleDeg, Coordinate<DistanceM> initialPosition) {
    // State: [x, y, vx, vy, angle, bias]
    ekf.x(0, 0) = initialPosition.x;
    ekf.x(1, 0) = initialPosition.y;
    ekf.x(2, 0) = 0;
    ekf.x(3, 0) = 0;
    ekf.x(4, 0) = initialAngleDeg * M_PI / 180;
    ekf.x(5, 0) = 0;
    is_active = true;
    return true;
  }

  void end() {
    is_active = false;
  }

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

    T& x = ekf.x(0, 0);
    T& y = ekf.x(1, 0);
    T& vx = ekf.x(2, 0);
    T& vy = ekf.x(3, 0);
    T& angle = ekf.x(4, 0);
    T& bias = ekf.x(5, 0);

    T gyro = gyroZ_in - gyroBias;

    // Predict angle
    angle += dt * (gyro - bias);

    // Gravity compensation
    const T g = (T)9.80665;
    T gx = g * cos(angle);
    T gy_ = g * sin(angle);

    T linAx = accelX - gx;
    T linAy = accelY - gy_;

    // Rotate to world frame
    T cosA = cos(angle);
    T sinA = sin(angle);
    T ax_w = linAx * cosA - linAy * sinA;
    T ay_w = linAx * sinA + linAy * cosA;

    // State prediction
    x += vx * dt + 0.5 * ax_w * dt * dt;
    y += vy * dt + 0.5 * ay_w * dt * dt;
    vx += ax_w * dt;
    vy += ay_w * dt;

    // Jacobian F
    auto& F = ekf.Fmat;
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++) F(i, j) = (i == j);
    F(0, 2) = dt;
    F(1, 3) = dt;
    F(4, 5) = -dt;

    ekf.P = F * ekf.P * transpose(F) + ekf.Q;
  }

  /// Update with magnetometer measurement (magX, magY in sensor frame)
  void updateMagnetometer(T magX, T magY) {
    if (!is_active) return;
    T meas = atan2(magY, magX);

    T stateAngle = ekf.x(4, 0);
    T innovation = wrapAngle(meas - stateAngle);

    tinyrobotics::Matrix<2, 1> z;
    z(0, 0) = stateAngle + innovation;
    z(1, 0) = 0;  // dummy for 2x1 measurement

    tinyrobotics::Matrix<2, 6> H = {};
    H(0, 4) = 1;                              // angle affects state[4]
    for (int j = 0; j < 6; j++) H(1, j) = 0;  // set second row to zero

    tinyrobotics::Matrix<2, 2> R = {};
    R(0, 0) = R_mag;
    R(1, 1) = 1e6;  // effectively disables second measurement

    ekf.update(z);  // uses standard 2x2 NZ=2 update
  }

  /// Provide actual GPS coordinate
  void updateGPS(const GPSCoordinate& gps, unsigned long nowMillis) {
    if (!is_active) return;
    if (hasPrevGPS) {
      T dtGPS = (nowMillis - lastGPSUpdateMillis) / (T)1000;
      if (dtGPS > 0) {
        T dist = gps.distance(prevGPS);
        T ang = gps.bearing(prevGPS);

        tinyrobotics::Matrix<2, 1> z;
        z(0, 0) = (dist / dtGPS) * cos(ang);
        z(1, 0) = (dist / dtGPS) * sin(ang);

        tinyrobotics::Matrix<2, 6> H = {};
        H(0, 2) = 1;
        H(1, 3) = 1;

        ekf.update(z);  // uses standard NZ=2 update
      }
    }

    prevGPS = gps;
    lastGPSUpdateMillis = nowMillis;
    hasPrevGPS = true;
  }

  /// Publish current state as messages
  void publish() {
    if (!is_active) return;
    Coordinate<DistanceM> pos = getPosition();
    T angle = getHeadingAngleRad();
    T vel = getVelocity();

    // Publish as message (could be extended to include velocity, etc.)
    Message<float> msg{MessageContent::Speed, vel, Unit::MetersPerSecond};
    msg.source = MessageOrigin::IMU;
    sendMessage(msg);

    Message<float> msgAngle{MessageContent::Heading, angle, Unit::AngleRadian};
    msg.source = MessageOrigin::IMU;
    sendMessage(msgAngle);

    Message<Coordinate<DistanceM>> msgPos{MessageContent::Position, pos,
                                          Unit::Meters};
    msgPos.source = MessageOrigin::IMU;
    sendMessage(msgPos);

    Message<GPSCoordinate> msgGPS{MessageContent::PositionGPS, prevGPS,
                                  Unit::AngleDegree};
    msgPos.source = MessageOrigin::IMU;
    sendMessage(msgPos);
  }

  /// get position
  Coordinate<DistanceM> getPosition() const {
    return {ekf.x(0, 0), ekf.x(1, 0)};
  }
  /// get velocity as 2D vector and magnitude in m/s
  Vec2 getVelocity2D() const { return {ekf.x(2, 0), ekf.x(3, 0)}; }
  /// get velocity magnitude in m/s
  T getVelocity() const {
    Vec2 v = getVelocity2D();
    return sqrt(v.x * v.x + v.y * v.y);
  }
  /// get heading angle (radians)
  T getHeadingAngleRad() const { return ekf.x(4, 0); }

  /// get heading angle with unit conversion
  T getHeadingAngle(AngleUnit unit) const {
    T angleRad = getHeadingAngleRad();
    Angle result(angleRad, AngleUnit::RAD);
    return result.getAngle(unit);
  }

 private:
  KalmanFilter<6, 2> ekf;
  GPSCoordinate prevGPS;
  bool hasPrevGPS = false;
  bool is_active = false;

  unsigned long lastUpdateMillis = 0;
  unsigned long lastGPSUpdateMillis = 0;
  T gyroBias = 0;
  T R_mag = 0.05;

  static T wrapAngle(T a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
  }
};

}  // namespace tinyrobotics
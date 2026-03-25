#pragma once
#include <math.h>
#include <stdint.h>

#include "coordinates/GPSCoordinate.h"
#include "utils/KalmanFilter.h"

namespace tinyrobotics {

/**
 * @brief 2D IMU sensor fusion class for robotics applications.
 *
 * This class fuses data from gyroscope, accelerometer, magnetometer, and GPS to
 * estimate heading (angle), velocity, and position in a 2D plane. It uses
 * Kalman filter fusion for both heading and velocity, providing robust,
 * low-noise state estimates suitable for robotics.
 *
 * - **Heading fusion:** Combines gyro, accelerometer, magnetometer, and GPS
 * angles using a Kalman filter.
 * - **Velocity fusion:** Combines acceleration integration and GPS velocity
 * using a Kalman filter.
 * - **Position estimation:** Integrates velocity to estimate 2D position.
 *
 * The class is designed for extensibility and embedded use, with clear
 * separation of sensor update, fusion, and state query methods. All units are
 * SI (meters, seconds, radians).
 *
 * Example usage:
 * @code
 *   tinyrobotics::IMU2D<float> imu;
 *   imu.begin(initialHeading, initialPosition);
 *   // In your update loop:
 *   imu.update(accelX, accelY, gyroZ, millis());
 *   imu.updateMagnetometer(magX, magY, millis());
 *   imu.updateGPS(gpsCoord, millis());
 *   imu.calculate();
 *   float heading = imu.getAngleBlended();
 *   float velocity = imu.getVelocityBlended();
 *   auto pos = imu.getPosition();
 * @endcode
 *
 * @tparam T Numeric type (float or double recommended)
 */

template <typename T = float>
class IMU2D {
  /**
   * @brief 2D vector struct for position and velocity.
   */
  struct Vec2 {
    T x = 0;
    T y = 0;
  };

 public:
  /**
   * @brief Default constructor.
   */
  IMU2D() = default;

  /**
   * @brief Calibrate the gyroscope bias (zero-rate offset).
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
   * @param gyroZ The measured gyro Z value (rad/s) when stationary.
   *
   * @note Call this method before normal operation, ideally after power-on or
   * reset, and ensure the device is not rotating during calibration.
   */
  void calibrateGyro(T gyroZ) { gyroBias = gyroZ; }

  /**
   * @brief Initialize the IMU2D state with a known heading and position.
   *
   * This method sets the initial orientation (relative to north) and position
   * of the IMU. It should be called at startup, after sensor calibration, to
   * ensure all internal angles and position are referenced to a common global
   * frame (e.g., north and world coordinates).
   *
   * @param initialAngle Initial heading in radians (relative to north, e.g.,
   * from magnetometer)
   * @param initialPosition Initial position in world or base frame coordinates
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
  bool begin(T initialAngle, Coordinate<DistanceM> initialPosition) {
    gyroAngle = initialAngle;
    accelAngle = initialAngle;
    position = initialPosition;
    return true;
  }

  /**
   * @brief Update IMU state with new accelerometer and gyroscope data.
   * @param accelX Acceleration in X (m/s^2)
   * @param accelY Acceleration in Y (m/s^2)
   * @param gyroZ Angular velocity around Z (rad/s)
   * @param nowMillis Current time in milliseconds
   */
  void update(T accelX, T accelY, T gyroZ_in, unsigned long nowMillis) {
    T dt_in = 0;
    if (lastUpdateMillis != 0) {
      dt_in =
          (nowMillis - lastUpdateMillis) / 1000.0f;  // convert ms to seconds
    }
    lastUpdateMillis = nowMillis;
    if (dt_in <= 0) return;  // skip integration on first call or invalid dt

    // Remove bias
    T gyroZ = gyroZ_in - gyroBias;
    this->gyroZ = gyroZ;
    this->dt = dt_in;

    // --- Gyro integration (angle)
    gyroAngle += gyroZ * dt_in;

    // --- Accel angle (tilt in 2D plane)
    accelAngle = atan2(accelY, accelX);

    // --- Rotate acceleration into world frame (using blended angle for motion)
    T blendAngle = 0.8f * gyroAngle + 0.2 * accelAngle;  // simple blending
    T cosA = cos(blendAngle);
    T sinA = sin(blendAngle);

    Vec2 accelWorld;
    accelWorld.x = accelX * cosA - accelY * sinA;
    accelWorld.y = accelX * sinA + accelY * cosA;

    // --- Integrate velocity
    velocity.x += accelWorld.x * dt_in;
    velocity.y += accelWorld.y * dt_in;
    accelVelocity2D = velocity;
    accelVelocity = sqrt(velocity.x * velocity.x + velocity.y * velocity.y);

    // --- Integrate position
    position.x += velocity.x * dt_in;
    position.y += velocity.y * dt_in;
  }

  /**
   * @brief Update heading estimate with magnetometer data.
   * @param magX Magnetometer X axis
   * @param magY Magnetometer Y axis
   * @param nowMillis Current time in milliseconds
   */
  void updateMagnetometer(T magX, T magY, unsigned long nowMillis) {
    // --- Magnetometer angle (heading)
    magAngle = atan2(magY, magX);
    lastMagUpdateMillis = nowMillis;
    hasMagInfo = true;
  }

  /**
   * @brief Update position and velocity estimate with GPS data.
   * @param gps Latest GPS coordinate
   * @param nowMillis Current time in milliseconds
   */
  void updateGPS(const GPSCoordinate& gps, unsigned long nowMillis) {
    // compute GPS velocity and angle if we have a previous GPS position
    if (gpsPosition) {
      gpsAngle = gps.bearing(gpsPosition);
      gpsDistanceM = gps.distance(gpsPosition);
      float dt = (nowMillis - lastGPSUpdateMillis) / 1000.0f;  // seconds
      if (dt > 0) {
        gpsVelocity = gpsDistanceM / dt;
        // Calculate velocity components from angle and speed
        gpsVelocity2D.x = gpsVelocity * cos(gpsAngle);
        gpsVelocity2D.y = gpsVelocity * sin(gpsAngle);
        hasGPSInfo = true;
      }
    }
    // Save GPS position
    gpsPosition = gps;
    lastGPSUpdateMillis = nowMillis;
  }

  /**
   * @brief Run Kalman filter fusion for heading (angle) and velocity
   * (magnitude).
   *
   * Heading: fuses gyro, accel, mag, GPS angles.
   * Velocity: fuses acceleration integration and GPS velocity.
   *
   * @param R_accel Measurement noise for accel angle (default: 0.1)
   * @param R_mag Measurement noise for mag angle (default: 0.02)
   * @param R_gps Measurement noise for GPS angle (default: 0.2)
   * @param R_accelVel Measurement noise for accel velocity (default: 0.2)
   * @param R_gpsVel Measurement noise for GPS velocity (default: 0.5)
   */
  void calculate(float R_accel = 0.1f, float R_mag = 0.02f, float R_gps = 0.2f,
                 float R_accelVel = 0.2f, float R_gpsVel = 0.5f) {
    calculateAngle(R_accel, R_mag, R_gps);
    calculateVelocity(R_accelVel, R_gpsVel);
  }

  /**
   * @brief Reset all state variables to initial values.
   */
  void reset() {
    // Reset all class variables to initial state
    position = {0, 0};
    gyroAngle = 0;
    accelAngle = 0;
    accelVelocity = 0;
    magAngle = 0;
    gpsAngle = 0;
    gpsVelocity = 0;
    gpsDistanceM = 0;
    lastUpdateMillis = 0;
    lastMagUpdateMillis = 0;
    lastGPSUpdateMillis = 0;
    hasGPSInfo = false;
    hasMagInfo = false;
    gpsPosition = GPSCoordinate();
    accelVelocity2D = {0, 0};
    gpsVelocity2D = {0, 0};
    velocity = {0 ,0};
    blendedAngle = 0;
    // Reset Kalman filter for angle
    kf.x(0, 0) = 0;
    kf.Fmat(0, 0) = 1.0f;
    kf.P = kf.Fmat * kf.P * transpose(kf.Fmat) + kf.Q;
    kf.H(0, 0) = 1.0f;
    kf.R(0, 0) = 0.1f;
    // Reset Kalman filter for velocity
    velocityKF.x(0, 0) = 0;
    velocityKF.Fmat(0, 0) = 1.0f;
    // No process model (could add acceleration*dt if desired)
    velocityKF.x(0, 0) += 0;
    velocityKF.P = velocityKF.Fmat * velocityKF.P * transpose(velocityKF.Fmat) +
                   velocityKF.Q;
    blendedVelocity = 0;
    // do not reset gyroBias
    // gyroBias = 0;
  }

  // Getters
  /**
   * @brief Get the current gyro-integrated angle (rad).
   */
  T getGyroAngle() const { return gyroAngle; }

  /**
   * @brief Get the current accelerometer-derived angle (rad).
   */
  T getAccelAngle() const { return accelAngle; }

  /**
   * @brief Get the current magnetometer-derived angle (rad).
   */
  T getMagAngle() const { return magAngle; }

  /**
   * @brief Get the current GPS-derived angle (rad).
   */
  T getGPSAngle() const { return gpsAngle; }

  /**
   * @brief Get the current velocity magnitude from accelerometer integration
   * (m/s).
   */
  T getAccelVelocity() const { return accelVelocity; }

  /**
   * @brief Get the current velocity magnitude from GPS (m/s).
   */
  T getGPSVelocity() const { return gpsVelocity; }

  /**
   * @brief Get the blended velocity magnitude from IMU and GPS.
   * @param imuWeight Weight for IMU velocity (0-1)
   * @return Blended velocity (m/s)
   */
  T getVelocityBlended() const { return blendedVelocity; }

  /**
   * @brief Get the blended angle in radians.
   * @return T
   */
  T getAngleBlended() { return blendedAngle; }

  /**
   * @brief Get the current estimated position in the 2D plane.
   * @return Position vector (meters)
   */
  Coordinate<DistanceM> getPosition() const { return position; }

  /**
   * @brief Get the most recent GPS position.
   * @return GPSCoordinate object
   */
  GPSCoordinate getGPSPosition() const { return gpsPosition; }

 private:
  unsigned long lastUpdateMillis = 0;
  unsigned long lastMagUpdateMillis = 0;
  unsigned long lastGPSUpdateMillis = 0;
  T accelWeight = 0;
  T gyroAngle = 0;
  T accelAngle = 0;
  T accelVelocity = 0;
  T magAngle = 0;
  T gyroBias = 0;
  T gpsVelocity = 0;
  T gpsAngle = 0;
  T gpsDistanceM = 0;
  bool hasGPSInfo = false;
  bool hasMagInfo = false;

  Vec2 accelVelocity2D;
  Vec2 gpsVelocity2D;
  Vec2 velocity;

  // Store latest gyroZ and dt for Kalman prediction
  T gyroZ = 0;
  T dt = 0;
  KalmanFilter<1, 1> kf;
  T blendedAngle = 0;
  KalmanFilter<1, 1> velocityKF;
  T blendedVelocity = 0;

  Coordinate<DistanceM> position;
  GPSCoordinate gpsPosition;  // For GPS fusion

  void calculateAngle(float R_accel = 0.1f, float R_mag = 0.02f,
                      float R_gps = 0.2f) {
    // --- Prediction (state: angle)
    // State transition: x = x + gyroZ * dt
    kf.Fmat(0, 0) = 1.0f;
    kf.x(0, 0) += gyroZ * dt;
    kf.P = kf.Fmat * kf.P * transpose(kf.Fmat) + kf.Q;

    // --- Updates (as available)
    // Accelerometer angle update
    {
      tinyrobotics::Matrix<1, 1> z;
      z(0, 0) = accelAngle;
      kf.H(0, 0) = 1.0f;
      kf.R(0, 0) = R_accel;
      kf.update(z);
    }
    // Magnetometer angle update
    if (hasMagInfo) {
      tinyrobotics::Matrix<1, 1> z;
      z(0, 0) = magAngle;
      kf.H(0, 0) = 1.0f;
      kf.R(0, 0) = R_mag;
      kf.update(z);
    }
    // GPS angle update (if available)
    if (hasGPSInfo) {
      tinyrobotics::Matrix<1, 1> z;
      z(0, 0) = gpsAngle;
      kf.H(0, 0) = 1.0f;
      kf.R(0, 0) = R_gps;
      kf.update(z);
    }
    // --- Result
    blendedAngle = kf.x(0, 0);
  }

  /**
   * @brief Run Kalman filter fusion for velocity (magnitude) using accel and
   * GPS.
   *
   * @param R_accelVel Measurement noise for accel velocity
   * @param R_gpsVel Measurement noise for GPS velocity
   */
  void calculateVelocity(float R_accelVel = 0.2f, float R_gpsVel = 0.5f) {
    // Prediction: x = x (constant velocity model)
    velocityKF.Fmat(0, 0) = 1.0f;
    // No process model (could add acceleration*dt if desired)
    velocityKF.x(0, 0) += 0;
    velocityKF.P = velocityKF.Fmat * velocityKF.P * transpose(velocityKF.Fmat) +
                   velocityKF.Q;

    // Accelerometer velocity update (from integration)
    {
      tinyrobotics::Matrix<1, 1> z;
      z(0, 0) = accelVelocity;
      velocityKF.H(0, 0) = 1.0f;
      velocityKF.R(0, 0) = R_accelVel;
      velocityKF.update(z);
    }
    // GPS velocity update (if available)
    if (hasGPSInfo) {
      tinyrobotics::Matrix<1, 1> z;
      z(0, 0) = gpsVelocity;
      velocityKF.H(0, 0) = 1.0f;
      velocityKF.R(0, 0) = R_gpsVel;
      velocityKF.update(z);
    }
    blendedVelocity = velocityKF.x(0, 0);
  }
};

}  // namespace tinyrobotics
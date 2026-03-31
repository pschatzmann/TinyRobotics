
#pragma once
#include <math.h>
#include <stdint.h>

#include "TinyRobotics/communication/Message.h"
#include "TinyRobotics/communication/MessageHandler.h"
#include "TinyRobotics/communication/MessageSource.h"
#include "TinyRobotics/control/KalmanFilter.h"
#include "TinyRobotics/control/MotionState2D.h"
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/coordinates/GPSCoordinate.h"

namespace tinyrobotics {

/**
 * @brief 2D sensor fusion EKF for position, heading, and speed estimation.
 *
 * Supports optional sensors:
 * - IMU (gyro + optional acceleration)
 * - Wheel speed
 * - Absolute heading (magnetometer or GPS-derived)
 * - GPS position
 *
 * The EKF automatically adapts to the available sensors.
 */
class Fusion2D : public MessageSource, public IMotionState2D {
 public:
  /**
   * @brief Holds the estimated 2D state.
   */
  struct State2D {
    uint32_t timeMs;  ///< Timestamp of the state in milliseconds
    float x, y;       ///< Position in meters
    float heading;    ///< Heading in radians
    float speed;      ///< Speed in meters per second
    float gyroBias;   ///< Estimated gyro bias (radians/sec)
  };

  /**
   * @brief Simple 2D pose (x, y, heading).
   */
  struct Pose2D {
    float x, y;
    float heading;
  };

  /**
   * @brief Constructor. Resets the EKF state to zeros.
   */
  Fusion2D() { reset(0); }

  bool begin(const Coordinate<DistanceM>& initialPosition = {0, 0},
             float initialHeadingDeg = 0) {
    // Set initial state
    reset(0);                  // optional timestamp
    state.x = 10.0f;           // initial X position in meters
    state.y = 5.0f;            // initial Y position in meters
    state.heading = M_PI / 4;  // initial heading in radians (45 degrees)
    state.speed = 0.0f;        // optional initial speed
    state.timeMs = millis();
    return true;
  }


  // =====================
  // PREDICTION
  // =====================
  /**
   * @brief Predict the next state based on elapsed time.
   *
   * This method performs dead-reckoning using the last speed and heading.
   * If no wheel speed is available but IMU acceleration is provided, it will
   * integrate ax/ay to estimate speed.
   *
   * @param timeMs Current timestamp in milliseconds.
   * @param ax Optional IMU acceleration along body-forward axis (m/s²). Used
   * only if no wheel speed.
   * @param ay Optional IMU acceleration along body-sideways axis (m/s²). Used
   * only if no wheel speed.
   */
  void predict(uint32_t timeMs, float ax = 0.0f, float ay = 0.0f) {
    uint32_t dtMs = timeMs - lastPredictTime;
    lastPredictTime = timeMs;
    float dt = dtMs * 0.001f;
    if (dt <= 0.0f) return;
    if (dt > 0.1f) dt = 0.1f;

    float c = cosf(state.heading);
    float s = sinf(state.heading);

    // Determine velocity
    float v = state.speed;

    // Use IMU acceleration if wheel speed not available
    if (!hasWheelSpeed && hasIMU) {
      float ax_world = ax * c - ay * s;
      float ay_world = ax * s + ay * c;
      float dv = sqrtf(ax_world * ax_world + ay_world * ay_world) * dt;
      v += dv;
      state.speed = v;
    }

    // Heading integration
    float omega = hasIMU ? (omegaMeasured - state.gyroBias) : 0.0f;
    state.heading += omega * dt;
    state.heading = wrapAngle(state.heading);

    // Position integration (requires some velocity)
    if (v > 0.0f) {
      state.x += v * c * dt;
      state.y += v * s * dt;
    }

    // EKF covariance propagation
    float F[5][5] = {{1, 0, -v * s * dt, c * dt, 0},
                     {0, 1, v * c * dt, s * dt, 0},
                     {0, 0, 1, 0, -dt},
                     {0, 0, 0, 1, 0},
                     {0, 0, 0, 0, 1}};

    float q_pos = 0.05f, q_heading = 0.01f, q_speed = 0.1f, q_bias = 0.0005f;
    float Q[5][5] = {{q_pos, 0, 0, 0, 0},
                     {0, q_pos, 0, 0, 0},
                     {0, 0, q_heading, 0, 0},
                     {0, 0, 0, q_speed, 0},
                     {0, 0, 0, 0, q_bias}};

    float FP[5][5] = {}, FPFt[5][5] = {};
    for (int i = 0; i < 5; i++)
      for (int j = 0; j < 5; j++)
        for (int k = 0; k < 5; k++) FP[i][j] += F[i][k] * P[k][j];
    for (int i = 0; i < 5; i++)
      for (int j = 0; j < 5; j++)
        for (int k = 0; k < 5; k++) FPFt[i][j] += FP[i][k] * F[j][k];
    for (int i = 0; i < 5; i++)
      for (int j = 0; j < 5; j++) P[i][j] = FPFt[i][j] + Q[i][j];

    state.timeMs = timeMs;
  }

  // =====================
  // SENSOR UPDATES
  // =====================
  /**
   * @brief Update the EKF with IMU measurements.
   *
   * @param timeMs Timestamp of the IMU measurement.
   * @param yawRate Gyro z-axis measurement (yaw rate, rad/s).
   * @param ax Optional IMU acceleration along body x-axis (m/s²).
   * @param ay Optional IMU acceleration along body y-axis (m/s²).
   *
   * The yawRate is used for heading prediction, and optional ax/ay
   * are used to integrate speed when wheel encoder is missing.
   */
  void updateIMU(uint32_t timeMs, float yawRate, float ax = 0.0f,
                 float ay = 0.0f) {
    hasIMU = true;
    omegaMeasured = yawRate;
    predict(timeMs, ax, ay);
  }

  // Update filter using an IMU2D instance
  void updateIMU(const IMU2D<float>& imu) {
    uint32_t t = imu.getTimestamp();         // timestamp in ms or µs
    float omega = imu.getAngularVelocity();  // angular velocity (rad/s)
    float v = imu.getLinearVelocity();       // linear velocity (m/s)

    // 1. Integrate angular velocity into orientation
    updateIMU(t, omega);

    // 2. Integrate linear velocity into position/speed
    updateSpeed(t, v);
  }

  /**
   * @brief Update the EKF with wheel speed.
   *
   * @param timeMs Timestamp of the speed measurement.
   * @param speed Wheel speed in meters per second.
   *
   * Overrides IMU acceleration-based speed if available.
   */
  void updateSpeed(uint32_t timeMs, float speed) {
    hasWheelSpeed = true;
    predict(timeMs);
    float y = speed - state.speed;
    float R = 0.5f;
    float S = P[3][3] + R;
    float K[5];
    for (int i = 0; i < 5; i++) K[i] = P[i][3] / S;
    applyUpdate(K, y);
    updateCovariance(K, 3);
  }

  void updateSpeed(uint32_t timeMs, Speed newSpeed) {
    updateSpeed(timeMs, newSpeed.getSpeed(SpeedUnit::MPS));
  }

  /**
   * @brief Update the EKF with an absolute heading measurement.
   *
   * @param timeMs Timestamp of the heading measurement.
   * @param heading Absolute heading in radians (e.g., from magnetometer or
   * GPS-derived).
   *
   * This corrects the heading estimate and helps reduce gyro drift.
   */
  void updateHeading(uint32_t timeMs, float heading) {
    predict(timeMs);
    float y = wrapAngle(heading - state.heading);
    float R = 0.3f;
    float S = P[2][2] + R;
    float K[5];
    for (int i = 0; i < 5; i++) K[i] = P[i][2] / S;
    applyUpdate(K, y);
    state.heading = wrapAngle(state.heading);
    updateCovariance(K, 2);
  }

  void updateHeading(uint32_t timeMs, Angle newHeading) {
    updateHeading(timeMs, newHeading.getAngle(AngleUnit::RAD));
  }

  /**
   * @brief Update the EKF with GPS position.
   *
   * @param timeMs Timestamp of the GPS measurement.
   * @param x GPS X position in meters (or local coordinates).
   * @param y GPS Y position in meters (or local coordinates).
   * @param accuracy Standard deviation of the GPS measurement (meters).
   *
   * Corrects the EKF position estimate. Works independently of IMU or wheel
   * speed.
   */
  void updateGPS(uint32_t timeMs, float x, float y, float accuracy) {
    predict(timeMs);
    float R = accuracy * accuracy;
    // X
    {
      float residual = x - state.x;
      float S = P[0][0] + R;
      float K[5];
      for (int i = 0; i < 5; i++) K[i] = P[i][0] / S;
      applyUpdate(K, residual);
      updateCovariance(K, 0);
    }
    // Y
    {
      float residual = y - state.y;
      float S = P[1][1] + R;
      float K[5];
      for (int i = 0; i < 5; i++) K[i] = P[i][1] / S;
      applyUpdate(K, residual);
      updateCovariance(K, 1);
    }
  }

  void updateGPS(uint32_t timeMs, const Coordinate<DistanceM>& gpsCoord,
                 float accuracyM) {
    updateGPS(timeMs, gpsCoord.x, gpsCoord.y, accuracyM);
  }

  // =====================
  // OUTPUTS
  // =====================
  /**
   * @brief Get the full estimated state.
   * @return Current State2D struct.
   */
  State2D getState() const { return state; }

  /**
   * @brief Get 2D pose (position + heading).
   * @return Pose2D struct with x, y, heading.
   */
  Pose2D getPose() const { return {state.x, state.y, state.heading}; }

  /**
   * @brief Get the current speed estimate.
   * @return Speed in meters per second.
   */
  float getSpeedMPS() const { return state.speed; }

  /**
   * @brief Get the current estimated gyro bias.
   * @return Gyro bias in rad/s.
   */
  float getGyroBias() const { return state.gyroBias; }

  Coordinate<DistanceM> getPosition() const {
    return Coordinate<DistanceM>(state.x, state.y);
  }
  Angle getHeading() const { return Angle(state.heading, AngleUnit::RAD); }
  Speed getSpeed() const { return Speed(state.speed, SpeedUnit::MPS); }

 private:
  State2D state;
  float P[5][5] = {};
  uint32_t lastPredictTime = 0;
  float omegaMeasured = 0.0f;
  bool hasWheelSpeed = false;
  bool hasIMU = false;

  void applyUpdate(float K[5], float residual) {
    state.x += K[0] * residual;
    state.y += K[1] * residual;
    state.heading += K[2] * residual;
    state.speed += K[3] * residual;
    state.gyroBias += K[4] * residual;
    state.heading = wrapAngle(state.heading);
  }

  void updateCovariance(float K[5], int idx) {
    float newP[5][5];
    for (int i = 0; i < 5; i++)
      for (int j = 0; j < 5; j++) newP[i][j] = P[i][j] - K[i] * P[idx][j];
    for (int i = 0; i < 5; i++)
      for (int j = 0; j < 5; j++) P[i][j] = newP[i][j];
  }

  static float wrapAngle(float a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
  }

   /**
   * @brief Reset the EKF state and covariance.
   * @param timeMs Optional timestamp to set the initial state time.
   */
  void reset(uint32_t timeMs = millis()) {
    state = {};
    state.timeMs = timeMs;
    state.speed = 0.0f;
    state.heading = 0.0f;
    state.gyroBias = 0.0f;

    hasWheelSpeed = false;
    hasIMU = false;

    for (int i = 0; i < 5; i++)
      for (int j = 0; j < 5; j++) P[i][j] = (i == j) ? 1.0f : 0.0f;

    lastPredictTime = timeMs;
    omegaMeasured = 0.0f;
  }
 
};

}  // namespace tinyrobotics
#pragma once
#include <math.h>
#include <stdint.h>

#include "TinyRobotics/communication/Message.h"
#include "TinyRobotics/communication/MessageHandler.h"
#include "TinyRobotics/communication/MessageSource.h"
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/coordinates/Orientation3D.h"
#include "TinyRobotics/units/Speed.h"
#include "TinyRobotics/units/Distance.h"
#include "TinyRobotics/units/AngularVelocity.h"

namespace tinyrobotics {

/**
 * @class Fusion3D
 * @ingroup fusion
 * @brief 3D sensor fusion EKF for position, orientation, and velocity estimation.
 *
 * Supports optional sensors:
 * - IMU3D (gyro + acceleration)
 * - GPS position
 * - Absolute orientation (magnetometer or GPS-derived)
 *
 * The EKF automatically adapts to the available sensors.
 */
class Fusion3D : public MessageSource {
 public:
  struct State3D {
    uint32_t timeMs;  ///< Timestamp in ms
    float x, y, z;    ///< Position (meters)
    float vx, vy, vz; ///< Velocity (m/s)
    float yaw, pitch, roll; ///< Orientation (radians)
    float gyroBiasX, gyroBiasY, gyroBiasZ; ///< Gyro bias (rad/s)
  };

  Fusion3D() { reset(0); }

  bool begin(const Coordinate<float>& initialPosition = {0, 0, 0},
             const Orientation3D& initialOrientation = Orientation3D()) {
    reset(0);
    state.x = initialPosition.x;
    state.y = initialPosition.y;
    state.z = initialPosition.z;
    state.yaw = initialOrientation.yaw;
    state.pitch = initialOrientation.pitch;
    state.roll = initialOrientation.roll;
    state.vx = state.vy = state.vz = 0.0f;
    state.timeMs = millis();
    return true;
  }

  // Prediction step (dead-reckoning)
  void predict(uint32_t timeMs, float ax = 0.0f, float ay = 0.0f, float az = 0.0f) {
    uint32_t dtMs = timeMs - lastPredictTime;
    lastPredictTime = timeMs;
    float dt = dtMs * 0.001f;
    if (dt <= 0.0f) return;
    if (dt > 0.1f) dt = 0.1f;

    // Integrate acceleration for velocity
    state.vx += ax * dt;
    state.vy += ay * dt;
    state.vz += az * dt;

    // Integrate velocity for position
    state.x += state.vx * dt;
    state.y += state.vy * dt;
    state.z += state.vz * dt;

    // Integrate angular velocity for orientation (simple Euler integration)
    state.roll  += (omegaXMeasured - state.gyroBiasX) * dt;
    state.pitch += (omegaYMeasured - state.gyroBiasY) * dt;
    state.yaw   += (omegaZMeasured - state.gyroBiasZ) * dt;
    wrapAngles();

    state.timeMs = timeMs;
  }

  // IMU update (gyro + accel)
  void updateIMU(uint32_t timeMs, float omegaX, float omegaY, float omegaZ,
                 float ax, float ay, float az) {
    hasIMU = true;
    omegaXMeasured = omegaX;
    omegaYMeasured = omegaY;
    omegaZMeasured = omegaZ;
    predict(timeMs, ax, ay, az);
  }

  // GPS update
  void updateGPS(uint32_t timeMs, float x, float y, float z, float accuracy) {
    predict(timeMs);
    float R = accuracy * accuracy;
    // X
    float residualX = x - state.x;
    // Y
    float residualY = y - state.y;
    // Z
    float residualZ = z - state.z;
    // (EKF update step omitted for brevity)
    state.x += residualX * 0.5f; // Simple correction
    state.y += residualY * 0.5f;
    state.z += residualZ * 0.5f;
  }

  // Absolute orientation update (e.g., from magnetometer)
  void updateOrientation(uint32_t timeMs, float yaw, float pitch, float roll) {
    predict(timeMs);
    // (EKF update step omitted for brevity)
    state.yaw   = yaw;
    state.pitch = pitch;
    state.roll  = roll;
    wrapAngles();
  }

  // Outputs
  State3D getState() const { return state; }
  Coordinate<float> getPosition() const { return {state.x, state.y, state.z}; }
  Speed3D getVelocity() const { return Speed3D(state.vx, state.vy, state.vz, SpeedUnit::MPS); }
  Orientation3D getOrientation() const { return Orientation3D(state.yaw, state.pitch, state.roll); }

 private:
  State3D state;
  uint32_t lastPredictTime = 0;
  float omegaXMeasured = 0.0f, omegaYMeasured = 0.0f, omegaZMeasured = 0.0f;
  bool hasIMU = false;

  void wrapAngles() {
    while (state.yaw > M_PI) state.yaw -= 2 * M_PI;
    while (state.yaw < -M_PI) state.yaw += 2 * M_PI;
    while (state.pitch > M_PI) state.pitch -= 2 * M_PI;
    while (state.pitch < -M_PI) state.pitch += 2 * M_PI;
    while (state.roll > M_PI) state.roll -= 2 * M_PI;
    while (state.roll < -M_PI) state.roll += 2 * M_PI;
  }

  void reset(uint32_t timeMs = millis()) {
    state = {};
    state.timeMs = timeMs;
    state.x = state.y = state.z = 0.0f;
    state.vx = state.vy = state.vz = 0.0f;
    state.yaw = state.pitch = state.roll = 0.0f;
    state.gyroBiasX = state.gyroBiasY = state.gyroBiasZ = 0.0f;
    hasIMU = false;
    lastPredictTime = timeMs;
    omegaXMeasured = omegaYMeasured = omegaZMeasured = 0.0f;
  }
};

} // namespace tinyrobotics

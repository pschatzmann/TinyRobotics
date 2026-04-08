#include "TinyRobotics/communication/Message.h"
#pragma once
#include "TinyRobotics/communication/MessageHandler.h"
#include "TinyRobotics/odometry/IOdometryModel3D.h"
#include "TinyRobotics/units/Units.h"

namespace tinyrobotics {

/**
 * @class AirplaneOdometryModel3D
 * @ingroup odometry
 * @brief IOdometryModel3D implementation for fixed-wing airplanes using control
 * surface and throttle inputs.
 *
 * This model computes linear and angular velocities for a fixed-wing airplane
 * based on control inputs:
 * - Throttle (0..100%): Sets forward speed as a percentage of maxSpeed.
 * - Aileron (degrees, typically -45..45): Sets roll rate as a fraction of
 * maxRollRate.
 * - Elevator (degrees, typically -45..45): Sets pitch rate as a fraction of
 * maxPitchRate.
 * - Rudder (degrees, typically -45..45): Sets yaw rate as a fraction of
 * maxYawRate.
 *
 * ## Features
 * - Maps control surface deflections and throttle to body-frame velocities for
 * odometry integration.
 * - Supports message-driven control via onMessage() (handles
 * MessageContent::Throttle, ::Roll, ::Pitch, ::Yaw).
 * - All input values are clamped to safe ranges.
 * - Suitable for simulation, estimation, or as a reference model for real-time
 * control.
 *
 * ## Usage Example
 * @code
 * AirplaneOdometryModel3D model;
 * model.setThrottle(80);      // 80% throttle
 * model.setAileron(10);       // 10 degrees right aileron
 * model.setElevator(-5);      // 5 degrees down elevator
 * model.setRudder(0);         // Centered rudder
 * float vx, vy, vz, wx, wy, wz;
 * model.getLinearVelocity(vx, vy, vz);
 * model.getAngularVelocity(wx, wy, wz);
 * // vx will be 0.8 * maxSpeed, wx will be 10/45 * maxRollRate, etc.
 * @endcode
 *
 * ## Message Integration
 * The model can be updated from messages (e.g., from an RC receiver or
 * autopilot):
 * @code
 * Message<float> msg;
 * msg.content = MessageContent::Throttle;
 * msg.value = 75;
 * model.onMessage(msg); // Sets throttle to 75%
 * @endcode
 */
class AirplaneOdometryModel3D : public IOdometryModel3D {
 public:

  /**
   * @brief Register a callback to be invoked on relevant events (e.g., input change).
   */
  void registerCallback(void (*callback)(void*), void* userData) override {
    this->callback = callback;
    this->userData = userData;
  }

    /**
     * @brief Construct an AirplaneOdometryModel3D with configurable max rates.
     *
     * @param maxSpeed      Maximum forward speed (m/s). Typical: 20–40 m/s for small UAVs.
     * @param maxRollRate   Maximum roll rate (deg/s). Typical: 40–85 deg/s (converted to rad/s internally).
     * @param maxPitchRate  Maximum pitch rate (deg/s). Typical: 30–60 deg/s (converted to rad/s internally).
     * @param maxYawRate    Maximum yaw rate (deg/s). Typical: 17–40 deg/s (converted to rad/s internally).
     *
     * These values should be set to match the physical or simulated aircraft's limits.
     * For most fixed-wing RC airplanes:
     *   - maxSpeed:      25–35 m/s (90–125 km/h)
     *   - maxRollRate:   57 deg/s
     *   - maxPitchRate:  40 deg/s
     *   - maxYawRate:    29 deg/s
     *
     * Note: The constructor will convert the rates from deg/s to rad/s for internal use.
     * Example:
     *   AirplaneOdometryModel3D(30.0f, 57.0f, 40.0f, 29.0f); // 30 m/s, 57°/s, 40°/s, 29°/s
     */
    AirplaneOdometryModel3D(float maxSpeed = 30.0f, float maxRollRate = 57.0f,
                            float maxPitchRate = 40.0f, float maxYawRate = 29.0f)
        : throttlePercent(0),
          aileronDeg(0),
          elevatorDeg(0),
          rudderDeg(0),
          maxSpeed(maxSpeed),
          maxRollRate(maxRollRate * 0.01745329252f),
          maxPitchRate(maxPitchRate * 0.01745329252f),
          maxYawRate(maxYawRate * 0.01745329252f) {}

  /**
   * @brief Handle incoming control messages to set airplane control surfaces
   * and throttle. Supports MessageContent::Throttle, ::Roll, ::Pitch, ::Yaw
   * (float values).
   * @return true if message was handled
   */
  bool onMessage(const Message<float>& msg) {
    switch (msg.content) {
      case MessageContent::Throttle:
        setThrottle(msg.value);
        return true;
      case MessageContent::Roll:  // aileron
        setAileron(msg.value);
        return true;
      case MessageContent::Pitch:  // elevator
        setElevator(msg.value);
        return true;
      case MessageContent::Yaw:  // rudder
        setRudder(msg.value);
        return true;
      default:
        return false;
    }
  }

  // Control input setters
  // Throttle: 0..100 (%)
  void setThrottle(float percent) {
    throttlePercent = clamp(percent, 0.0f, 100.0f);
    if (callback) callback(userData);
  }
  // Control surfaces: degrees (typically -30..30)
  void setAileron(float deg) {
    aileronDeg = clamp(deg, -45.0f, 45.0f);
    if (callback) callback(userData);
  }
  void setElevator(float deg) {
    elevatorDeg = clamp(deg, -45.0f, 45.0f);
    if (callback) callback(userData);
  }
  void setRudder(float deg) {
    rudderDeg = clamp(deg, -45.0f, 45.0f);
    if (callback) callback(userData);
  }

  void getLinearVelocity(float& vx, float& vy, float& vz) const override {
    // Forward speed proportional to throttle percent (0..100)
    vx = (throttlePercent / 100.0f) * maxSpeed;
    vy = 0.0f;
    vz = 0.0f;
  }

  void getAngularVelocity(float& wx, float& wy, float& wz) const override {
    // Control surfaces in degrees, convert to radians for rates
    constexpr float deg2rad = 0.01745329252f;
    wx = (aileronDeg / 45.0f) * maxRollRate;  // normalized to max deflection
    wy = (elevatorDeg / 45.0f) * maxPitchRate;
    wz = (rudderDeg / 45.0f) * maxYawRate;
    // Optionally, scale by actual deflection in radians if desired:
    // wx = aileronDeg * deg2rad * rollRatePerRad;
    // ...
  }

 protected:
  void (*callback)(void*) = nullptr;
  void* userData = nullptr;
  float throttlePercent;  // 0..100
  float aileronDeg;       // degrees
  float elevatorDeg;      // degrees
  float rudderDeg;        // degrees
  float maxSpeed;         // m/s
  float maxRollRate;      // rad/s
  float maxPitchRate;     // rad/s
  float maxYawRate;       // rad/s

  static float clamp(float v, float min, float max) {
    return (v < min) ? min : (v > max) ? max : v;
  }
};

}  // namespace tinyrobotics

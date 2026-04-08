
#pragma once
#include <array>

#include "TinyRobotics/communication/Message.h"
#include "TinyRobotics/odometry/IOdometryModel3D.h"
#include "TinyRobotics/units/Units.h"

namespace tinyrobotics {

/**
 * @class DroneOdometryModel3D
 * @ingroup odometry
 * @brief IOdometryModel3D implementation for quadcopters/drones using motor
 * percentages.
 *
 * This model computes linear and angular velocities for a quadcopter based on
 * the percentage output (0-100%) of each motor. It assumes a standard
 * X-configuration quadcopter with four motors:
 *   - Motor 0: Front Left
 *   - Motor 1: Front Right
 *   - Motor 2: Rear Right
 *   - Motor 3: Rear Left
 *
 * ## Features
 * - Maps motor percentages to body-frame velocities for odometry integration.
 * - Supports message-driven control via onMessage() (handles
 * MessageContent::MotorSpeed with origin_id 0..3).
 * - All input values are clamped to 0..100%.
 * - Suitable for simulation, estimation, or as a reference model for real-time
 * control.
 *
 * ## Notes
 * - This model is designed for extensibility and realism, supporting
 * multi-motor drones and robust unit-safe arithmetic.
 * - The mapping logic is as follows:
 *   - Vertical velocity (vz) is proportional to the average of all motor
 * percentages and scaled by maxVz.
 *   - Roll and pitch rates are proportional to left/right and front/back motor
 * differences, scaled by maxRollRate and maxPitchRate.
 *   - Yaw rate is proportional to diagonal (CW vs CCW) motor differences,
 * scaled by maxYawRate.
 * - All control input values are clamped to the range 0..100% for safety and
 * realism.
 * - The model is message-driven: it integrates with the communication framework
 * and updates motor percentages via MessageContent::MotorSpeed messages
 * (origin_id 0..3).
 * - Default parameters (maxVz, maxRollRate, maxPitchRate, maxYawRate) are
 * chosen to be realistic for typical quadcopters, but can be configured for
 * other drone types.
 * - See the README and documentation for integration and usage examples.
 *
 * ## Usage Example
 * @code
 * DroneOdometryModel3D model;
 * model.setMotorPercent(0, 60); // Front Left
 * model.setMotorPercent(1, 60); // Front Right
 * model.setMotorPercent(2, 60); // Rear Right
 * model.setMotorPercent(3, 60); // Rear Left
 * float vx, vy, vz, wx, wy, wz;
 * model.getLinearVelocity(vx, vy, vz);
 * model.getAngularVelocity(wx, wy, wz);
 * @endcode
 */
class DroneOdometryModel3D : public IOdometryModel3D {
 public:
  static constexpr int NUM_MOTORS = 4;

  /**
   * @brief Register a callback to be invoked on relevant events (e.g., input
   * change).
   */
  void registerCallback(void (*callback)(void*), void* userData) override {
    this->callback = callback;
    this->userData = userData;
  }

  /**
   * @brief Construct a DroneOdometryModel3D with configurable max rates.
   *
   * @param maxVz      Maximum vertical speed (m/s) at 100% throttle. Typical: 5
   * m/s.
   * @param maxRollRate   Maximum roll rate (deg/s, converted to rad/s).
   * Typical: 200 deg/s.
   * @param maxPitchRate  Maximum pitch rate (deg/s, converted to rad/s).
   * Typical: 200 deg/s.
   * @param maxYawRate    Maximum yaw rate (deg/s, converted to rad/s). Typical:
   * 100 deg/s.
   */
  DroneOdometryModel3D(float maxVz = 5.0f, float maxRollRate = 200.0f,
                       float maxPitchRate = 200.0f, float maxYawRate = 100.0f)
      : maxVz(maxVz),
        maxRollRate(maxRollRate * 0.01745329252f),
        maxPitchRate(maxPitchRate * 0.01745329252f),
        maxYawRate(maxYawRate * 0.01745329252f) {
    motorPercent.fill(0.0f);
  }

  /**
   * @brief Set the percentage (0..100) for a given motor (0..3).
   */
  void setMotorPercent(int motor, float percent) {
    if (motor < 0 || motor >= NUM_MOTORS) return;
    motorPercent[motor] = clamp(percent, 0.0f, 100.0f);
    // update on last motor update to avoid multiple callbacks during batch
    // updates
    if (callback && motor == (NUM_MOTORS - 1)) callback(userData);
  }

 protected:
  void (*callback)(void*) = nullptr;
  void* userData = nullptr;

  /**
   * @brief Handle incoming motor speed messages (MessageContent::MotorSpeed,
   * origin_id = motor index).
   */
  bool onMessage(const Message<float>& msg) {
    if (msg.content == MessageContent::MotorSpeed &&
        msg.origin_id < NUM_MOTORS) {
      setMotorPercent(msg.origin_id, msg.value);
      return true;
    }
    return false;
  }

  /**
   * @brief Compute linear velocity (vx, vy, vz) in m/s (body frame).
   * Only vz is modeled (vertical climb/descent), vx and vy are zero.
   */
  void getLinearVelocity(float& vx, float& vy, float& vz) const override {
    // Average all motors for vertical thrust
    float avg = 0.0f;
    for (float p : motorPercent) avg += p;
    avg /= NUM_MOTORS;
    vx = 0.0f;
    vy = 0.0f;
    vz = (avg / 100.0f) * maxVz;
  }

  /**
   * @brief Compute angular velocity (wx, wy, wz) in rad/s (body frame).
   * Roll and pitch rates are proportional to left/right and front/back motor
   * differences. Yaw rate is proportional to diagonal differences (simplified
   * torque model).
   */
  void getAngularVelocity(float& wx, float& wy, float& wz) const override {
    // Roll: left (0+3) vs right (1+2)
    float rollInput = ((motorPercent[0] + motorPercent[3]) -
                       (motorPercent[1] + motorPercent[2])) /
                      200.0f;
    // Pitch: front (0+1) vs rear (2+3)
    float pitchInput = ((motorPercent[0] + motorPercent[1]) -
                        (motorPercent[2] + motorPercent[3])) /
                       200.0f;
    // Yaw: (CW - CCW) torque, assume 0/2 are CW, 1/3 are CCW
    float yawInput = ((motorPercent[0] + motorPercent[2]) -
                      (motorPercent[1] + motorPercent[3])) /
                     200.0f;
    wx = rollInput * maxRollRate;
    wy = pitchInput * maxPitchRate;
    wz = yawInput * maxYawRate;
  }

 protected:
  std::array<float, NUM_MOTORS> motorPercent;  // 0..100 for each motor
  float maxVz;                                 // m/s
  float maxRollRate;                           // rad/s
  float maxPitchRate;                          // rad/s
  float maxYawRate;                            // rad/s

  static float clamp(float v, float min, float max) {
    return (v < min) ? min : (v > max) ? max : v;
  }
};

}  // namespace tinyrobotics

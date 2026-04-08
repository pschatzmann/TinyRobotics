
#pragma once
#include <Arduino.h>

#include "SpeedFromThrottle.h"

namespace tinyrobotics {

/**
 * @class SpeedFromThrottleWithInertia
 * @ingroup odometry
 * @brief Estimates speed from throttle with acceleration/inertia modeling (for
 * boats, etc).
 *
 * This class extends SpeedFromThrottle by simulating acceleration/inertia:
 * - Speed ramps toward the target value at a configurable acceleration rate
 * (m/s^2).
 * - Call updateSpeed(deltaTimeSec) in your main loop to advance the speed
 * estimate.
 * - Useful for boats or vehicles with slow acceleration.
 *
 * Example usage:
 * @code
 * SpeedFromThrottleWithInertia speedMap(2.0f, 0.2f); // 2 m/s max, 0.2 m/s^2
 * // In your loop:
 * speedMap.updateSpeed(dt); // dt in seconds
 * float speed = speedMap.getSpeed();
 * @endcode
 */
class SpeedFromThrottleWithInertia : public SpeedFromThrottle {
 public:
  SpeedFromThrottleWithInertia(float maxSpeedMps, float accelerationMps2, uint8_t numMotors = 1)
      : SpeedFromThrottle(maxSpeedMps, numMotors), acceleration(accelerationMps2) {
    throttlePercent.resize(numMotors, 0.0f);
    lastUpdateMs.resize(numMotors, 0);
  }
  SpeedFromThrottleWithInertia(Speed maxSpeed, float accelerationMps2, uint8_t numMotors = 1)
      : SpeedFromThrottle(maxSpeed, numMotors), acceleration(accelerationMps2) {
    throttlePercent.resize(numMotors, 0.0f);
    lastUpdateMs.resize(numMotors, 0);
  }

  void setAcceleration(float a) { acceleration = a; }
  float getAcceleration() const { return acceleration; }

  // Override to store throttle for inertia simulation
  void setThrottlePercent(float throttle, uint8_t motor = 0) override {
    if (motor >= numMotors) return;
    throttlePercent[motor] = throttle;
    lastUpdateMs[motor] = millis();
    // Do not ramp speed here; updateSpeed() will be called in the main loop
  }

  // Optionally, allow instant set for testing
  void setSpeedInstant(float speed, uint8_t motor = 0) {
    if (motor >= numMotors) return;
    speedMps[motor] = speed;
    sendSpeedMessage(motor);
  }
  // ISpeedSource interface implementation
  Speed getSpeed(uint8_t motor = 0) const override {
    if (motor >= numMotors) return Speed(0.0f, SpeedUnit::MPS);
    return Speed(speedMps[motor], SpeedUnit::MPS);
  }

  // Call this in your main loop with the elapsed time (in milliseconds)
  Speed updateSpeed(uint32_t deltaTimeMs, uint8_t motor = 0) override {
    if (motor >= numMotors) return Speed(0.0f, SpeedUnit::MPS);
    float target = getSpeedMPS(throttlePercent[motor]);
    float delta = target - speedMps[motor];
    float maxDelta = acceleration * deltaTimeMs / 1000.0f; // Convert ms to seconds for maxDelta
    if (fabs(delta) <= maxDelta) {
      speedMps[motor] = target;
    } else {
      speedMps[motor] += (delta > 0 ? maxDelta : -maxDelta);
    }
    sendSpeedMessage(motor);
    lastUpdateMs[motor] = millis();
    return Speed(speedMps[motor], SpeedUnit::MPS);
  }

 protected:
  float acceleration;  // m/s^2
  std::vector<float> throttlePercent;
  std::vector<unsigned long> lastUpdateMs;

};

}  // namespace tinyrobotics

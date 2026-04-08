#pragma once

#include <vector>

#include "TinyRobotics/communication/MessageHandler.h"
#include "TinyRobotics/odometry/ISpeedSource.h"
#include "TinyRobotics/units/Speed.h"

namespace tinyrobotics {

/**
 * @class SpeedFromThrottle
 * @ingroup odometry
 * @brief Estimates vehicle speed from throttle percentage using calibration
 * data.
 *
 * This class provides a flexible, piecewise linear mapping from throttle
 * percentage (e.g., -100% to 100%) to speed (in meters per second), based on
 * user-provided calibration points. It is useful for systems where direct speed
 * measurement is unavailable or unreliable, and a throttle-to-speed
 * relationship can be established empirically. The mapping can be linear or
 * non-linear, depending on the calibration data.
 *
 * **Features:**
 * - By default, 0% throttle maps to 0 m/s, 100% throttle maps to maxSpeedMps,
 * and -100% throttle maps to -maxSpeedMps.
 * - Additional calibration points can be added with
 * addSpeedCalibration(throttle, speed), allowing for non-linear mappings.
 * - If a calibration point for a throttle value already exists, it is replaced;
 * otherwise, it is inserted in order.
 * - Supports both direct speed queries (getSpeed, getSpeedMPS) and integration
 * with the TinyRobotics message system.
 * - When used as a MessageHandler, it listens for Throttle messages and emits
 * Speed messages via MessageSource.
 * - Calibration data can be cleared and redefined at runtime.
 *
 * **Typical usage:**
 * @code
 *   SpeedFromThrottle speedMap(2.0f); // 2 m/s at 100% throttle
 *   speedMap.addSpeedCalibration(50.0f, 1.0f); // 1 m/s at 50%
 *   float speed = speedMap.getSpeedMPS(75.0f); // Interpolated speed
 *   // As a message handler/source:
 *   bus.subscribe(speedMap); // Receives Throttle messages, emits Speed
 * messages
 * @endcode
 *
 * **Integration:**
 * - Use as a standalone utility for throttle-to-speed conversion.
 * - Or, connect to a MessageBus to automatically convert Throttle messages to
 * Speed messages.
 *
 * @see addSpeedCalibration, getSpeed, getSpeedMPS, onMessage, clearCalibration
 *
 * @author Phil Schatzmann
 */

class SpeedFromThrottle : public MessageSource,
                          public MessageHandler,
                          public ISpeedSource {
 public:
  SpeedFromThrottle(float maxSpeedMps, uint8_t numMotors = 1) {
    this->numMotors = numMotors;
    this->maxSpeedMps = maxSpeedMps;
    speedMps.resize(numMotors, 0.0f);
    init();
  }

  SpeedFromThrottle(Speed maxSpeed, uint8_t numMotors = 1)
      : SpeedFromThrottle(maxSpeed.getValue(SpeedUnit::MPS), numMotors) {}

  void setMaxSpeed(Speed speed) {
    this->maxSpeedMps = speed.getValue(SpeedUnit::MPS);
    init();
  }

  /// Handle Throttle messages to update speed
  bool onMessage(const Message<float>& msg) override {
    if (msg.content == MessageContent::Throttle) {
      setThrottlePercent(msg.value);
      return true;
    }
    return false;  // Not handled
  }

  /// Add or update a calibration point (throttlePercent, speedMps)
  void addSpeedCalibration(float throttlePercent, float speedMps) {
    auto it = calibrationData.begin();
    while (it != calibrationData.end() && it->first < throttlePercent) {
      ++it;
    }
    if (it != calibrationData.end() && it->first == throttlePercent) {
      it->second = speedMps;  // Replace existing
    } else {
      calibrationData.insert(
          it, std::pair<float, float>(throttlePercent, speedMps));
    }
  }

  /// Clear all calibration data
  void clearCalibration() { calibrationData.clear(); }

  /// Define the actual throttle value
  void setThrottlePercent(float throttlePercent, uint8_t motor = 0) override {
    assert(motor < numMotors);
    speedMps[motor] = getSpeedMPS(throttlePercent);
    sendSpeedMessage(motor);
  }

  /// Get the actual speed based on the last throttle value
  Speed getSpeed(uint8_t motor = 0) const override {
    assert(motor < numMotors);
    return Speed(speedMps[motor], SpeedUnit::MPS);
  }

  Speed updateSpeed(uint32_t deltaTimeMs, uint8_t motor = 0) override {
    assert(motor < numMotors);
    return Speed(speedMps[motor], SpeedUnit::MPS);
  }

  size_t getMotorCount() const override { return numMotors; }

  protected:
  std::vector<std::pair<float, float>> calibrationData;
  std::vector<float> speedMps;
  uint8_t numMotors = 1;
  float maxSpeedMps = 0.0f;

  // Overload for multi-motor begin
  bool init() {
    calibrationData.clear();
    addSpeedCalibration(0.0f, 0.0f);           // 0% throttle = 0 m/s
    addSpeedCalibration(100.0f, maxSpeedMps);  // 100% throttle = max speed
    addSpeedCalibration(-100.0f,
                        maxSpeedMps);  // -100% throttle = max reverse speed
    return true;
  }
  void sendSpeedMessage(uint8_t motor = 0) {
    assert(motor < numMotors);
    Message<float> msg(MessageContent::Speed, speedMps[motor],
                       Unit::MetersPerSecond, MessageOrigin::System);
    msg.origin_id = motor;
    MessageSource::sendMessage(msg);
  }

  /// Get speed in meters per second by interpolating calibration data
  float getSpeedMPS(float throttlePercent) const {
    if (calibrationData.empty()) return 0.0f;
    // Clamp below
    if (throttlePercent <= calibrationData.front().first)
      return calibrationData.front().second;
    // Clamp above
    if (throttlePercent >= calibrationData.back().first)
      return calibrationData.back().second;
    // Find interval
    for (size_t i = 1; i < calibrationData.size(); ++i) {
      float t0 = calibrationData[i - 1].first;
      float t1 = calibrationData[i].first;
      if (throttlePercent >= t0 && throttlePercent <= t1) {
        float s0 = calibrationData[i - 1].second;
        float s1 = calibrationData[i].second;
        float alpha = (throttlePercent - t0) / (t1 - t0);
        return s0 + alpha * (s1 - s0);
      }
    }
    // Should not reach here
    return 0.0f;
  }
};

}  // namespace tinyrobotics
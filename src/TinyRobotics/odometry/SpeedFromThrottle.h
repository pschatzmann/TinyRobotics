#pragma once
#include <vector>

#include "TinyRobotics/units/Speed.h"

namespace tinyrobotics {


/**
 * @class SpeedFromThrottle
 * @ingroup odometry
 * @brief Estimates vehicle speed from throttle percentage using calibration data.
 *
 * This class provides a flexible, piecewise linear mapping from throttle percentage
 * (e.g., -100% to 100%) to speed (in meters per second), based on user-provided calibration points.
 * It is useful for systems where direct speed measurement is unavailable or unreliable, and a throttle-to-speed
 * relationship can be established empirically.
 *
 * - By default, 0% throttle maps to 0 m/s, 100% throttle maps to maxSpeedMps, and -100% throttle maps to -maxSpeedMps.
 * - Additional calibration points can be added with addSpeedCalibration(throttle, speed), allowing for non-linear mappings.
 * - If a calibration point for a throttle value already exists, it is replaced; otherwise, it is inserted in order.
 * - The class supports both direct speed queries (getSpeed, getSpeedMPS) and integration with the TinyRobotics message system.
 * - When used as a MessageHandler, it listens for Throttle messages and emits Speed messages via MessageSource.
 *
 * Typical usage:
 * @code
 *   SpeedFromThrottle speedMap(2.0f); // 2 m/s at 100% throttle
 *   speedMap.addSpeedCalibration(50.0f, 1.0f); // 1 m/s at 50%
 *   float speed = speedMap.getSpeedMPS(75.0f); // Interpolated speed
 *   // As a message handler/source:
 *   bus.subscribe(speedMap); // Receives Throttle messages, emits Speed messages
 * @endcode
 *
 * @see addSpeedCalibration, getSpeed, getSpeedMPS, onMessage
 *
 * @author Phil Schatzmann
 */

class SpeedFromThrottle : public MessageSource, public MessageHandler {
 public:
  SpeedFromThrottle(float maxSpeedMps) { begin(maxSpeedMps); }

  bool begin(float maxSpeedMps) {
    addSpeedCalibration(0.0f, 0.0f);           // 0% throttle = 0 m/s
    addSpeedCalibration(100.0f, maxSpeedMps);  // 100% throttle = max speed
    addSpeedCalibration(-100.0f,
                        maxSpeedMps);  // -100% throttle = max reverse speed
    return true;
  }

  //// Get speed as a Speed object (with units)
  Speed getSpeed(float throttlePercent) const {
    float speedMps = getSpeedMPS(throttlePercent);
    Speed speed(speedMps, SpeedUnit::MPS);
    return speed;
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

  /// Handle Throttle messages to update speed
  bool onMessage(const Message<float>& msg) override {
    if (msg.content == MessageContent::Throttle) {
      setThrottle(msg.value);
      return true;
    }
    return false;  // Not handled
  }

  /// Add or update a calibration point (throttlePercent, speedMps)
  void addSpeedCalibration(float throttlePercent, float speedMps) {
    // Insert in order by throttlePercent, replace if exists
    auto it = calibrationData.begin();
    while (it != calibrationData.end() && it->first < throttlePercent) {
      ++it;
    }
    if (it != calibrationData.end() && it->first == throttlePercent) {
      it->second = speedMps;  // Replace existing
    } else {
      calibrationData.insert(it, std::make_pair(throttlePercent, speedMps));
    }
  }

  /// Clear all calibration data
  void clearCalibration() { calibrationData.clear(); }

 protected:
  std::vector<std::pair<float, float>> calibrationData;

  void setThrottle(float throttlePercent) {
    float speedMps = getSpeedMPS(throttlePercent);
    Message<float> msg(MessageContent::Speed, speedMps, Unit::MetersPerSecond,
                       MessageOrigin::System);
    MessageSource::sendMessage(msg);
  }
};

}  // namespace tinyrobotics
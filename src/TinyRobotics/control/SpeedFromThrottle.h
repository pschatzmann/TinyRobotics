#pragma once
#include <vector>

#include "TinyRobotics/units/Speed.h"

namespace tinyrobotics {

/**
 * @brief Maps throttle percentage to speed using calibration data.
 *
 * This class provides a simple way to estimate the speed
 * of a vehicle based on the throttle percentage, using a piecewise linear
 * calibration curve. Calibration points can be added with
 * addSpeedCalibration(), and the speed for any throttle value is interpolated
 * between the nearest calibration points.
 *
 * - By default, 0% throttle maps to 0 m/s and 100% throttle maps to
 * maxSpeedMps.
 * - If a calibration point for a throttle value already exists, it is replaced.
 * - The calibration data is always kept ordered by throttle percent.
 *
 * Example usage:
 * @code
 *   SpeedFromThrottle speedMap(2.0f); // 2 m/s at 100% throttle
 *   speedMap.addSpeedCalibration(50.0f, 1.0f); // 1 m/s at 50%
 *   auto speed = speedMap.getSpeed(75.0f); // Interpolated speed
 * @endcode
 */

class SpeedFromThrottle {
 public:
  SpeedFromThrottle(float maxSpeedMps) { begin(maxSpeedMps); }

  bool begin(float maxSpeedMps) {
    addSpeedCalibration(0.0f, 0.0f);           // 0% throttle = 0 m/s
    addSpeedCalibration(100.0f, maxSpeedMps);  // 100% throttle = max speed
    addSpeedCalibration(-100.0f, maxSpeedMps);  // -100% throttle = max reverse speed
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
  std::vector<std::pair<float, float>>
      calibrationData;  // Optional: throttle vs speed pairs for calibration
};

}  // namespace tinyrobotics
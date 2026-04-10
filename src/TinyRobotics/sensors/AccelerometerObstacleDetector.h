#pragma once
#include <cmath>
#include <functional>

#include "TinyRobotics/communication/Message.h"
#include "TinyRobotics/communication/MessageSource.h"
#include "TinyRobotics/units/Distance.h"

namespace tinyrobotics {

/**
 * @class AccelerometerObstacleDetector
 * @ingroup sensors
 * @brief Simple obstacle detector using accelerometer data.
 *
 * Detects sudden deceleration (collision/obstacle) events by monitoring the
 * X/Y/Z acceleration. When a deceleration above a threshold is detected, a
 * callback (with user data) or message is triggered.
 *
 * ## Usage Example
 * @code
 * AccelerometerObstacleDetector detector(3.0f); // 3 m/s^2 threshold
 * detected!"); }, nullptr); void loop() { detector.update(ax, ay, az); }
 * @endcode
 */
class AccelerometerObstacleDetector : public MessageSource {
 public:
  /**
   * @brief Construct with a deceleration threshold.
   * @param threshold Deceleration threshold (m/s^2)
   */
  AccelerometerObstacleDetector(float threshold = 3.0f)
      : threshold(threshold) {}


  /**
   * @brief Update the detector with new acceleration values. Checks for sudden
   * deceleration.
   * @param ax Acceleration in X (m/s^2)
   * @param ay Acceleration in Y (m/s^2)
   * @param az Acceleration in Z (m/s^2)
   */
  void update(float ax, float ay, float az) {
    float magnitude = std::sqrt(ax * ax + ay * ay + az * az);
    float delta = lastMagnitude - magnitude;
    if (delta > threshold) {
      Message<float> msg(MessageContent::Obstacle, 0.0f, Unit::Meters);
      msg.origin = MessageOrigin::IMU;
      sendMessage(msg);
    }
    lastMagnitude = magnitude;
  }

  /**
   * @brief Set the deceleration threshold (m/s^2).
   */
  void setThreshold(float t) { threshold = t; }

 protected:
  // No accelerometer reference needed
  float threshold;
  float lastMagnitude = 0.0f;
};

}  // namespace tinyrobotics

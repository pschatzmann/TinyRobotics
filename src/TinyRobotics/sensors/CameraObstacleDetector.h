
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "TinyRobotics/communication/Message.h"
#include "TinyRobotics/communication/MessageSource.h"

namespace tinyrobotics {

/**
 * @class CameraObstacleDetector
 * @ingroup sensors
 * @brief Simple obstacle detection algorithm for grayscale camera images.
 *
 * This class scans a central region of a grayscale image (e.g., from a camera
 * or vision sensor) to detect the presence of dark obstacles based on pixel
 * intensity and density. It is useful for basic obstacle avoidance in robotics.
 *
 * Features:
 *   - Scans the center region of the image for dark pixels
 *   - Triggers detection if the density of dark pixels exceeds a threshold
 *   - Returns detection status and density (fraction of dark pixels)
 *   - Configurable pixel threshold and trigger density
 *   - Lightweight, suitable for embedded/Arduino use
 *
 * @note Expected format:
 * - The input image should be a grayscale buffer (uint8_t*)
 * - The pixel intensity is expected to be in the range 0 (black) to 255
 * (white).
 * - The layout of the image buffer should be row-major (i.e., pixel at (x,y) is
 * at index y*width + x).
 * - on ESP32 e.g. PIXFORMAT_GRAYSCALE, FRAMESIZE_QVGA (320 x 240)
 *
 * Usage Example:
 * @code
 * CameraObstacleDetector detector(60, 0.2f); // threshold=60, trigger=20%
 * auto result = detector.process(image, width, height);
 * if (result.detected) {
 *   // Obstacle detected, take action
 * }
 * @endcode
 */

class CameraObstacleDetector : public MessageSource {
 public:
  struct Result {
    bool detected;
    float density;  // 0.0 → 1.0
  };

  CameraObstacleDetector(
      uint8_t threshold = 60,  // pixel threshold (0–255)
      float trigger = 0.15f    // % of pixels to trigger detection
      )
      : _threshold(threshold), _trigger(trigger) {}

  // image: grayscale buffer (width * height)
  Result process(const uint8_t* image, size_t width, size_t height) {
    if (!image || width == 0 || height == 0) {
      return {false, 0.0f};
    }

    // Focus on center region (ignore edges)
    size_t x_start = width / 4;
    size_t x_end = 3 * width / 4;
    size_t y_start = height / 3;
    size_t y_end = height;

    size_t total = 0;
    size_t hits = 0;

    for (size_t y = y_start; y < y_end; y++) {
      for (size_t x = x_start; x < x_end; x++) {
        uint8_t pixel = image[y * width + x];

        // Detect dark object
        if (pixel < _threshold) {
          hits++;
        }
        total++;
      }
    }

    float density = (total > 0) ? (float)hits / total : 0.0f;

    // publish angle to direction of movement
    Message<float> msgError(MessageContent::Density, density * 100.0f, Unit::Percent);
    msgError.source = MessageOrigin::Camera;
    sendMessage(msgError);

    return {density > _trigger, density};
  }

 protected:
  uint8_t _threshold;
  float _trigger;
};

}  // namespace tinyrobotics
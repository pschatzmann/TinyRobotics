
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "TinyRobotics/communication/Message.h"
#include "TinyRobotics/communication/MessageSource.h"

namespace tinyrobotics {

/**
 * @brief Simple edge-following algorithm for grayscale camera images.
 *
 * This class scans a horizontal row of a grayscale image (e.g., from a line or
 * edge sensor) to detect the strongest edge (sharpest intensity change) using a
 * threshold. It is useful for basic line-following or edge-detection robots.
 *
 * Features:
 *   - Scans a row near the bottom of the image for the most prominent edge
 *   - Returns the x position of the detected edge and error from image center
 *   - Configurable edge threshold for noise rejection
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
 * CameraEdgeFollower follower(30); // threshold = 30
 * auto result = follower.process(image, width, height);
 * if (result.found) {
 *   // Use result.position or result.error for steering
 * }
 * @endcode
 */

class CameraEdgeFollower : public MessageSource {
 public:
  struct Result {
    bool found;
    int position;  // x position of edge
    int error;     // deviation from center
  };

  CameraEdgeFollower(uint8_t threshold = 20) : _threshold(threshold) {}

  Result process(const uint8_t* image, size_t width, size_t height) {
    if (!image || width < 2 || height == 0) {
      return {false, 0, 0};
    }

    // Scan one row near bottom (where obstacles are closer)
    size_t y = (height * 3) / 4;

    int best_x = -1;
    int max_edge = 0;

    for (size_t x = 0; x < width - 1; x++) {
      uint8_t p1 = image[y * width + x];
      uint8_t p2 = image[y * width + x + 1];

      int diff = abs((int)p1 - (int)p2);

      if (diff > _threshold && diff > max_edge) {
        max_edge = diff;
        best_x = x;
      }
    }

    if (best_x < 0) {
      return {false, 0, 0};
    }

    int center = width / 2;
    int error = best_x - center;

    // publish angle to direction of movement
    Message<float> msgError(MessageContent::Error, error, Unit::Pixel);
    msgError.source = MessageOrigin::Camera;
    sendMessage(msgError);

    return {true, best_x, error};
  }

 protected:
  uint8_t _threshold;
};

}  // namespace tinyrobotics

#pragma once
#include <stddef.h>
#include <stdint.h>

namespace tinyrobotics {

/**
 * @brief Simple line-following algorithm for grayscale camera images.
 *
 * This class scans a horizontal row of a grayscale image (e.g., from a camera or line sensor)
 * to detect the center of the widest dark line (such as a tape or path on the floor).
 * It is useful for basic line-following robots.
 *
 * Features:
 *   - Scans a row near the bottom of the image for the widest dark segment
 *   - Returns the x position of the detected line center and error from image center
 *   - Configurable pixel threshold and minimum line width
 *   - Remembers last seen line position for robustness
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
 * CameraLineFollower follower(80, 5); // threshold=80, minWidth=5
 * auto result = follower.process(image, width, height);
 * if (result.found) {
 *   // Use result.position or result.error for steering
 * }
 * @endcode
 */

class CameraLineFollower {
 public:
  struct Result {
    bool found;
    int position;  // x coordinate of line center
    int error;     // deviation from image center
  };

  CameraLineFollower(uint8_t threshold = 80,  // pixel threshold
                     int minWidth = 3         // minimum line width (pixels)
                     )
      : _threshold(threshold), _minWidth(minWidth), _lastPosition(-1) {}

  Result process(const uint8_t* image, size_t width, size_t height) {
    if (!image || width == 0 || height == 0) {
      return {false, 0, 0};
    }

    // Scan near bottom (closer to robot)
    size_t y = (height * 3) / 4;

    int bestStart = -1;
    int bestEnd = -1;
    int currentStart = -1;

    // Find longest dark segment
    for (size_t x = 0; x < width; x++) {
      uint8_t pixel = image[y * width + x];

      bool isLine = (pixel < _threshold);  // dark line

      if (isLine) {
        if (currentStart < 0) {
          currentStart = x;
        }
      } else {
        if (currentStart >= 0) {
          int segmentWidth = x - currentStart;

          if (segmentWidth >= _minWidth) {
            if (segmentWidth > (bestEnd - bestStart)) {
              bestStart = currentStart;
              bestEnd = x;
            }
          }
          currentStart = -1;
        }
      }
    }

    // Handle line reaching end of row
    if (currentStart >= 0) {
      int segmentWidth = width - currentStart;
      if (segmentWidth >= _minWidth) {
        bestStart = currentStart;
        bestEnd = width;
      }
    }

    if (bestStart < 0) {
      // fallback to last known position
      if (_lastPosition >= 0) {
        int error = _lastPosition - (int)(width / 2);
        return {false, _lastPosition, error};
      }
      return {false, 0, 0};
    }

    int position = (bestStart + bestEnd) / 2;

    // simple smoothing
    if (_lastPosition >= 0) {
      position = (_lastPosition * 3 + position) / 4;
    }

    _lastPosition = position;

    int center = width / 2;
    int error = position - center;

    return {true, position, error};
  }

 protected:
  uint8_t _threshold;
  int _minWidth;
  int _lastPosition;
};

}  // namespace tinyrobotics
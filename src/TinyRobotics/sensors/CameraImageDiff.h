

#pragma once
#include <cmath>
#include <cstdint>
#include <vector>

#include "TinyRobotics/utils/AllocatorPSRAM.h"

namespace tinyrobotics {

/**
 * @class CameraImageDiff
 * @ingroup sensors
 * @brief Computes pixel-wise differences between consecutive grayscale camera
 * images.
 *
 * This class is designed for simple motion or change detection in a sequence of
 * grayscale images. It calculates the absolute difference between the current
 * and previous image, applies a threshold, and provides methods to count the
 * number of changed pixels globally or in image regions.
 *
 * Features:
 *   - Computes per-pixel absolute difference between frames
 *   - Thresholds the difference to create a binary mask of changed pixels
 *   - Counts changed pixels globally, by vertical thirds (left/center/right),
 * or by horizontal thirds (top/center/bottom)
 *   - Lightweight and suitable for embedded/Arduino use
 *
 * Usage Example:
 * @code
 * CameraImageDiff diff;
 * diff.process(image, width, height);
 * diff.threshold(30);
 * uint32_t changed = diff.countChanges();
 * auto [left, center, right] = diff.countChangesSplitVertical();
 * auto [top, center, bottom] = diff.countChangesSplitHorizontal();
 * @endcode
 */

class CameraImageDiff {
 public:
  CameraImageDiff() = default;

  void process(const uint8_t* image, size_t width, size_t height) {
    this->width_ = width;
    this->height_ = height;
    diff_.resize(width * height);
    prevImage_.resize(width * height);
    for (uint32_t i = 0; i < width_ * height_; ++i) {
      diff_[i] = (uint8_t)std::abs(int(image[i]) - int(prevImage_[i]));
    }
    memcpy(prevImage_.data(), image, width * height);
  }

  /// Simple thresholding: convert difference to binary mask
  void threshold(uint8_t thresh) {
    mask_.resize(width_ * height_);
    for (uint32_t i = 0; i < width_ * height_; ++i) {
      mask_[i] = (diff_[i] > thresh) ? 255 : 0;
    }
  }

  /// Count number of pixels above threshold
  uint32_t countChanges() {
    uint32_t cnt = 0;
    for (uint32_t i = 0; i < width_ * height_; ++i)
      if (mask_[i] != 0) ++cnt;
    return cnt;
  }

  /**
   * Count number of changed pixels in left, center, and right thirds of the
   * image.
   * @return std::tuple<uint32_t, uint32_t, uint32_t> (left, center, right)
   */
  std::tuple<uint32_t, uint32_t, uint32_t> countChangesSplitVertical() {
    uint32_t left = 0, center = 0, right = 0;
    int leftEnd = width_ / 3;
    int centerEnd = 2 * width_ / 3;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        int idx = y * width_ + x;
        if (mask_[idx] != 0) {
          if (x < leftEnd)
            ++left;
          else if (x < centerEnd)
            ++center;
          else
            ++right;
        }
      }
    }
    return std::make_tuple(left, center, right);
  }

  /**
   * Count number of changed pixels in top, center, and bottom thirds of the
   * image.
   * @return std::tuple<uint32_t, uint32_t, uint32_t> (top, center, bottom)
   */
  std::tuple<uint32_t, uint32_t, uint32_t> countChangesSplitHorizontal() {
    uint32_t top = 0, center = 0, bottom = 0;
    int topEnd = height_ / 3;
    int centerEnd = 2 * height_ / 3;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        int idx = y * width_ + x;
        if (mask_[idx] != 0) {
          if (y < topEnd)
            ++top;
          else if (y < centerEnd)
            ++center;
          else
            ++bottom;
        }
      }
    }
    return std::make_tuple(top, center, bottom);
  }

  void end() {
    prevImage_.resize(0);
    diff_.resize(0);
    mask_.resize(0);
  }
  int getWidth() const { return width_; }
  int getHeight() const { return height_; }
  int getPixelCount() const { return width_ * height_; }
  /// Provides the raw difference buffer (grayscale values 0–255) for external
  /// use or debugging.
  uint8_t* getDiff() { return diff_.data(); }

 protected:
  int width_ = 0;
  int height_ = 0;
  std::vector<uint8_t, AllocatorPSRAM<uint8_t>> prevImage_;
  std::vector<uint8_t, AllocatorPSRAM<uint8_t>> diff_;
  std::vector<uint8_t, AllocatorPSRAM<uint8_t>> mask_;
};

}  // namespace tinyrobotics
# TinyRobotics Sensors Module

This directory contains sensor processing classes for robotics applications. These classes are designed for embedded systems (such as Arduino/ESP32) and provide lightweight, efficient algorithms for interpreting sensor data from cameras and range sensors.

There are plenty of Arduino device libraries, so there is no need to provide this functionality here: We just provide high level robotics classes that can process the output from those libraries.

## Available Sensor Classes

### CameraEdgeFollower

Detects the strongest edge in a grayscale camera image (e.g., for line or edge following robots).

- Scans a row near the bottom of the image for the sharpest intensity change.
- Returns the x position of the detected edge and error from image center.
- Configurable edge threshold for noise rejection.

### CameraLineFollower

Detects the center of the widest dark line in a grayscale camera image (e.g., for tape or path following).

- Scans a row near the bottom of the image for the widest dark segment.
- Returns the x position of the detected line center and error from image center.
- Configurable pixel threshold and minimum line width.

### CameraObstacleDetector

Detects obstacles in a grayscale camera image by analyzing the density of dark pixels in the center region.

- Scans the center region of the image for dark pixels.
- Triggers detection if the density of dark pixels exceeds a threshold.
- Returns detection status and density (fraction of dark pixels).
- Configurable pixel threshold and trigger density.

### CameraImageDiff

Detects motion or changes between consecutive grayscale camera images.

- Computes per-pixel absolute difference between frames.
- Thresholds the difference to create a binary mask of changed pixels.
- Counts changed pixels globally, by vertical thirds (left/center/right), or by horizontal thirds (top/center/bottom).

### RangeSensor

Abstracts a generic range sensor (e.g., ultrasonic, infrared, or LIDAR).

- Provides distance measurement and validity checking.
- Supports callback for new data.

## Image Format

All camera-based classes expect grayscale (8-bit, single channel) images as a contiguous array of `uint8_t` values, in row-major order, with dimensions specified by `width` and `height`.

## Example Usage

```cpp
#include "sensors/CameraLineFollower.h"

CameraLineFollower follower(80, 5); // threshold=80, minWidth=5
uint8_t image[WIDTH * HEIGHT];
// ... fill image ...
auto result = follower.process(image, WIDTH, HEIGHT);
if (result.found) {
  // Use result.position or result.error for steering
}
```

## Notes

- These classes are designed for efficiency and minimal memory usage.
- They are suitable for real-time robotics and embedded systems.
- For more details, see the documentation in each header file.

# TinyRobotics Sensors Module

This directory contains sensor processing classes for robotics applications. These classes are designed for embedded systems (such as Arduino/ESP32) and provide lightweight, efficient algorithms for interpreting sensor data from cameras and range sensors.

There are plenty of Arduino device libraries, so there is no need to provide this functionality here: We just provide high level robotics classes that can process the output from those libraries.

## Available Sensor Classes

### RangeSensor

Abstracts a generic range sensor (e.g., ultrasonic, infrared, or LIDAR).

- Provides distance measurement and validity checking.
- Supports callback for new data.

### SpeedFromThrottle

Estimates vehicle speed from throttle input using a piecewise linear mapping.

- Supports direct speed queries and integration with the TinyRobotics message system.
- Useful for simple robots without encoders or for simulation.

### WheelEncoder

Tracks wheel rotation and distance using encoder pulses.

- Provides position, speed, and distance traveled.
- Integrates with the TinyRobotics message system for reporting.


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

## See Examples

- [Camera Line Follower](../../../examples/sensors/CameraLineFollower/CameraLineFollower.ino)
- [Range Sensor](../../../examples/sensors/RangeSensor/RangeSensor.ino)
- [Wheel Encoder](../../../examples/sensors/WheelEncoder/WheelEncoder.ino)

## Class Documentation

- [sensors](https://pschatzmann.github.io/TinyRobotics/group__sensors.html)

## Notes

- These classes are designed for efficiency and minimal memory usage.
- They are suitable for real-time robotics and embedded systems.
- All camera-based classes expect grayscale (8-bit, single channel) images as a contiguous array of `uint8_t` values, in row-major order, with dimensions specified by `width` and `height`.

- For more details, see the documentation in each header file.

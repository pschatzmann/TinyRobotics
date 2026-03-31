
# TinyRobotics IMU Module

## Overview

This module provides dead-reckoning integration for 2D Inertial Measurement Units (IMUs) in robotics applications. It estimates position, velocity, and heading by integrating gyroscope and accelerometer data. No sensor fusion, Kalman filtering, magnetometer, or GPS support is provided; the IMU2D class is intended for simple heading and velocity estimation in embedded and Arduino environments.

## Features

- Dead-reckoning 2D IMU integration (no sensor fusion)
- Position, velocity, and heading estimation
- Designed for embedded and Arduino environments

## Key Classes

- `IMU2D`: 2D IMU dead-reckoning integration

## Example Usage

```cpp
#include <TinyRobotics.h> // Main library include for all modules

// Create IMU2D instance (float precision)
IMU2D imu;

void setup() {
  // Initialize with known heading (radians) and position (Coordinate<float>)
  imu.begin(initialHeadingRad, initialPosition);
}

void loop() {
    // Update with new sensor data (accelerometer X/Y, gyro Z, timestamp)
    imu.update(accelX, accelY, gyroZ, millis());
    // Publish results to subscribers
    imu.publish();
  // Query state
  auto pos = imu.getPosition();
  auto vel = imu.getLinearVelocity();
  float heading = imu.getTheta();
}
```

## API Highlights

- `begin(float headingRad, Coordinate<float> position)`: Initialize state
- `update(float accelX, float accelY, float gyroZ, uint32_t timestampMs)`: Update with new sensor data
- `publish()`: Publish result to subscribers
- `getPosition()`: Get current position (Coordinate<float>)
- `getLinearVelocity()`: Get current linear velocity (float)
- `getTheta()`: Get current heading (float, radians)

## See Examples

- [IMU 2D Example](../../../examples/imu/imu2d/imu2d.ino)

## Notes

- All units are SI (meters, seconds, radians)
- See source code for detailed Doxygen documentation

---
For more details, see the source files and Doxygen comments.


# TinyRobotics IMU Module

## Overview


This module provides dead-reckoning integration for 2D and 3D Inertial Measurement Units (IMUs) in robotics applications. It estimates position, velocity, and orientation by integrating gyroscope and accelerometer data. No sensor fusion, Kalman filtering, magnetometer, or GPS support is provided. The IMU2D and IMU3D classes are intended for simple heading, velocity, and pose estimation in embedded and Arduino environments.

## Features

- Dead-reckoning 2D and 3D IMU integration (no sensor fusion)
- Position, velocity, and heading/orientation estimation
- 3D support: orientation (yaw, pitch, roll), 3D position, 3D velocity
- Designed for embedded and Arduino environments

## Class Documentation

- [imu](https://pschatzmann.github.io/TinyRobotics/group__imu.html)


## Example Usage

### 2D IMU
```cpp
#include <TinyRobotics.h>
IMU2D imu;
void setup() {
  imu.begin(initialHeadingRad, initialPosition);
}
void loop() {
  imu.update(accelX, accelY, gyroZ, millis());
  imu.publish();
  auto pos = imu.getPosition();
  auto vel = imu.getLinearVelocity();
  float heading = imu.getTheta();
}
```

### 3D IMU
```cpp
#include <TinyRobotics.h>
IMU3D<> imu3d;
void setup() {
  imu3d.begin(/* initialPosition */ {0,0,0}, /* initialOrientation */ Orientation3D());
}
void loop() {
  // Update with new sensor data (accelX, accelY, accelZ, gyroX, gyroY, gyroZ, timestamp)
  imu3d.update(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, millis());
  imu3d.publish();
  auto pos = imu3d.getPosition();
  auto vel = imu3d.getLinearVelocity();
  auto orientation = imu3d.getOrientation();
  auto lastDelta = imu3d.getLastDelta();
  auto lastAngularVel = imu3d.getLastAngularVelocity();
}
```


## API Highlights

### IMU2D
- `begin(float headingRad, Coordinate<float> position)`: Initialize state
- `update(float accelX, float accelY, float gyroZ, uint32_t timestampMs)`: Update with new sensor data
- `publish()`: Publish result to subscribers
- `getPosition()`: Get current position (Coordinate<float>)
- `getLinearVelocity()`: Get current linear velocity (float)
- `getTheta()`: Get current heading (float, radians)

### IMU3D
- `begin(Coordinate<T> position, Orientation3D orientation)`: Initialize state
- `update(T accelX, T accelY, T accelZ, T gyroX, T gyroY, T gyroZ, uint32_t timestampMs)`: Update with new sensor data
- `publish()`: Publish result to subscribers
- `getPosition()`: Get current position (Coordinate<T>)
- `getLinearVelocity()`: Get current linear velocity (Speed3D)
- `getOrientation()`: Get current orientation (Orientation3D)
- `getLastDelta()`: Get last position delta (Distance3D)
- `getLastAngularVelocity()`: Get last angular velocity (AngularVelocity3D)


## See Examples

- [IMU 2D Example](../../../examples/imu/imu2d/imu2d.ino)

## Notes

- All units are SI (meters, seconds, radians)
- See source code for detailed Doxygen documentation

---
For more details, see the source files and Doxygen comments.

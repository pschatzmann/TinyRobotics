# TinyRobotics IMU Module

## Overview
This module provides sensor fusion for 2D and 3D Inertial Measurement Units (IMUs) in robotics applications. It supports fusing data from gyroscopes, accelerometers, magnetometers, and GPS to estimate position, velocity, and heading using advanced filtering techniques such as Kalman and Extended Kalman Filters (EKF).

## Features
- Sensor fusion for 2D and 3D IMUs
- Kalman and Extended Kalman Filter implementations
- Gyroscope bias calibration
- Magnetometer and GPS integration
- Robust position, velocity, and heading estimation
- Designed for embedded and Arduino environments

## Key Classes
- `IMU2D`: 2D IMU sensor fusion using EKF

## Example Usage
```cpp
#include <TinyRobotics.h>

// Create IMU2D instance
IMU2D<float> imu;

void setup() {
  // Calibrate gyro bias (call when stationary)
  imu.calibrateGyro(gyroZ_stationary);
  // Initialize with known heading and position
  imu.begin(initialHeadingDeg, initialPosition);
}

void loop() {
  imu.update(accelX, accelY, gyroZ, millis());
  imu.updateMagnetometer(magX, magY);
  imu.updateGPS(gpsCoord, millis());
  // Publish and/or read the result
  imu.publish();
  auto pos = imu.getPosition();
  auto vel = imu.getVelocity();
  float heading = imu.getAngle();
}
```

## API Highlights
- `calibrateGyro(float gyroZ)`: Calibrate gyro bias
- `begin(float heading, Coordinate position)`: Initialize state
- `update(...)`: Update with new sensor data
- `updateMagnetometer(...)`: Update with magnetometer data
- `updateGPS(...)`: Update with GPS data
- `publish()` : Publish result to subscibers
- `getPosition()`, `getVelocity()`, `getAngle()`: Query state


## See Examples

- [IMU 2D Example](../../../examples/imu/imu2d/imu2d.ino)

## Notes
- All units are SI (meters, seconds, radians)
- See source code for detailed Doxygen documentation

---
For more details, see the source files and Doxygen comments.

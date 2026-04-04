# TinyRobotics

[![Arduino Library](https://img.shields.io/badge/Arduino-Library-blue.svg)](https://www.arduino.cc/reference/en/libraries/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

TinyRobotics is a modular, header-only C++ library for Arduino and embedded systems, providing essential building blocks for robotics, navigation, mapping, and sensor integration. The library is designed for flexibility and extensibility, supporting a variety of coordinate systems, path planning algorithms, and sensor types.

## [Features](src/TinyRobotics)

The following __base functionality__ is automatically available:

- [**Coordinate Systems**](src/TinyRobotics/coordinates/README.md): 2D/3D coordinates, GPS coordinates, NMEA sentence parsing, local cartesian frames, and hierarchical frame management (SE(2)/SE(3)).
- [**Path Planning**](src/TinyRobotics/planning/README.md): Generic A* and Dijkstra algorithm, path representation, and planning utilities for navigation and obstacle avoidance.
- [**Mapping**](src/TinyRobotics/maps/README.md): Grid maps (occupancy/value), path maps (graph-based), and 3D point clouds for environment modeling and SLAM.
- [**Control**](src/TinyRobotics/control/README.md): Core control algorithms including PID controllers, Kalman filters, moving averages, schedulers, and the MotionController for path following and vehicle control.
- [**Units**](src/TinyRobotics/units/README.md): Strongly-typed units for distance, time, speed, and angle to ensure type safety and clarity.
- [**Sensors**](src/TinyRobotics/sensors/README.md): RangeSensor abstraction, Camera scenarios and WheelEncoder
- [**Odometry**](src/TinyRobotics/odometry/README.md): 2D and 3D odometry modules for incremental position, orientation, and distance estimation. Provides robust pose tracking for mobile robots and vehicles.
- [**IMU**](src/TinyRobotics/imu/README.md): 2D and 3D Inertial Measurement Unit (IMU) abstraction for embedded robotics. Provides real-time access to orientation, acceleration, and angular velocity. Integrates seamlessly with control, navigation, and SLAM modules for feedback and state estimation.
- [**Fusion**](src/TinyRobotics/fusion/README.md): Sensor fusion algorithms for combining IMU, odometry, and other sensor data to estimate robot pose, velocity, and orientation in 2D and 3D.
- [**SLAM**](src/TinyRobotics/localization/README.md): Simultaneous Localization and Mapping (SLAM) for real-time robot pose estimation and map building. Includes frame transforms, exploration, and modular integration with sensors and vehicles.
- [**Motors**](src/TinyRobotics/motors/README.md): Drivers and abstractions for stepper, DC, and RC servo motors. .
- [**Vehicles**](src/TinyRobotics/vehicles/README.md): High-level abstractions for controlling cars (4WD, Ackerman), quadrotors, airplanes, and boats.
- [**Utilities**](src/TinyRobotics/utils/README.md): Logger, Streams, Buffers, serialization and memory allocators for embedded systems.


The following __extended functionality__ can be included optionally:

- [**Communication**](src/TinyRobotics/communication/README.md): Message-based framework supporting UDP, ESPNow, Serial, LoRa, IEEE802_15_4, and more for telemetry, remote control, and inter-robot messaging.
- [**Concurrency**](src/TinyRobotics/concurrency/README.md): Thread-safe queues, buffers, mutexes, and RTOS integration for multitasking and synchronization.

## Documentation

- [Class Documentaion](https://pschatzmann.github.io/TinyRobotics/modules.html)
- [Examples](/examples)

## Installation

For Arduino, you can download the library as zip and call include Library -> zip library. Or you can git clone this project into the Arduino libraries folder e.g. with

```
cd  ~/Documents/Arduino/libraries
git clone https://github.com/pschatzmann/TinyRobotics.git
```

## Dependencies

You also need to install the follwing Arduino libraries:
- FastAccelStepper
- Servo
- ESP32Servo

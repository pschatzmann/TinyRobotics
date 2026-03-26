# TinyRobotics

[![Arduino Library](https://img.shields.io/badge/Arduino-Library-blue.svg)](https://www.arduino.cc/reference/en/libraries/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

TinyRobotics is a modular, header-only C++ library for Arduino and embedded systems, providing essential building blocks for robotics, navigation, mapping, and sensor integration. The library is designed for flexibility and extensibility, supporting a variety of coordinate systems, path planning algorithms, and sensor types.

## Features


The following __base functionality__ is automatically available:

- **Coordinate Systems**: 2D/3D coordinates, GPS coordinates, NMEA sentence parsing, local cartesian frames, and hierarchical frame management (SE(2)/SE(3)).
- **Frame Management**: Manage trees of coordinate frames in 2D and 3D, with support for GPS/geodetic integration and conversion between local and global frames.
- **Path Planning**: Generic A* and Dijkstra algorithm, path representation, and planning utilities for navigation and obstacle avoidance.
- **Mapping**: Grid maps (occupancy/value), path maps (graph-based), and 3D point clouds for environment modeling and SLAM.
- **Control**: Core control algorithms including PID controllers, Kalman filters, moving averages, and schedulers for feedback, filtering, and state estimation.
- **Units**: Strongly-typed units for distance, time, speed, and angle to ensure type safety and clarity.
- **Sensors**: Range sensor abstraction, sensor fusion, and scheduling utilities for periodic sensor tasks.

The following __extended functionality__ can be included optionally:

- **Communication**: Message-based framework supporting UDP, ESPNow, Serial, LoRa, IEEE802_15_4, and more for telemetry, remote control, and inter-robot messaging.
- **Concurrency**: Thread-safe queues, buffers, mutexes, and RTOS integration for multitasking and synchronization.

## Documentation

- Each class is documented in its header file with detailed descriptions and usage examples.
- See the `examples/` directory for practical demonstrations of key features.
- For module overviews, see the `README.md` files in each subdirectory (e.g., `src/TinyRobotics/coordinates/README.md`).


## Modules Overview

- **Coordinates**: 2D/3D coordinates, GPS, frame management, and geodetic conversion. See [Coordinates Module](src/TinyRobotics/coordinates/README.md)
- **Maps**: Grid maps, path maps (graphs), and 3D point clouds for environment modeling and navigation. See [Maps Module](src/TinyRobotics/maps/README.md)
- **Planning**: Path representation, A* and Dijkstra algorithms for navigation and motion planning. See [Planning Module](src/TinyRobotics/planning/README.md)
- **Control**: Core control algorithms including PID controllers, Kalman filters, moving averages, and schedulers for feedback and state estimation. See [Control Module](src/TinyRobotics/control/README.md)
- **Sensors**: Camera-based edge/line/obstacle detection, image differencing, and range sensor abstraction. See [Sensors Module](src/TinyRobotics/sensors/README.md)
- **Motors**: H-Bridge and Servo motor drivers for DC and RC servo control. See [Motors Module](src/TinyRobotics/motors/README.md)
- **Vehicles**: High-level abstractions for cars (4WD, Ackerman), quadrotors, airplanes, and boats. See [Vehicles Module](src/TinyRobotics/vehicles/README.md)
- **Communication**: Message-based communication framework supporting multiple protocols (UDP, ESPNow, Serial, LoRa, IEEE802_15_4, and more) for telemetry, remote control, and inter-robot messaging. See [Communication Module](src/TinyRobotics/communication/README.md)
- **Concurrency**: Thread-safe queues, buffers, mutexes, and RTOS integration for multitasking and synchronization. See [Concurrency Module](src/TinyRobotics/concurrency/README.md)
- **Units**: Strongly-typed units for distance, time, speed, and angle. See [Units Module](src/TinyRobotics/units/README.md)
- **Utilities**: Logger, scheduler, serialization, and memory allocators for embedded systems. See [Utilities Module](src/TinyRobotics/utils/README.md)


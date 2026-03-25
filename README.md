# TinyRobotics

TinyRobotics is a modular, header-only C++ library for Arduino and embedded systems, providing essential building blocks for robotics, navigation, mapping, and sensor integration. The library is designed for flexibility and extensibility, supporting a variety of coordinate systems, path planning algorithms, and sensor types.

## Features

- **Coordinate Systems**: 2D/3D coordinates, GPS coordinates, NMEA sentence parsing, local cartesian frames, and hierarchical frame management (SE(2)/SE(3)).
- **Frame Management**: Manage trees of coordinate frames in 2D and 3D, with support for GPS/geodetic integration and conversion between local and global frames.
- **Path Planning**: Generic A* and Dijkstra algorithm, path representation, and planning utilities for navigation and obstacle avoidance.
- **Mapping**: Grid maps (occupancy/value), path maps (graph-based), and 3D point clouds for environment modeling and SLAM.
- **Communication**: Message-based framework supporting UDP, ESPNow, Serial, LoRa, IEEE802_15_4, and more for telemetry, remote control, and inter-robot messaging.
- **Concurrency**: Thread-safe queues, buffers, mutexes, and RTOS integration for multitasking and synchronization.
- **Units**: Strongly-typed units for distance, time, speed, and angle to ensure type safety and clarity.
- **Sensors**: Range sensor abstraction, sensor fusion, and scheduling utilities for periodic sensor tasks.
- **Utilities**: Logger, scheduler, serialization, and fixed-capacity vector for embedded-friendly development.

## Documentation

- Each class is documented in its header file with detailed descriptions and usage examples.
- See the `examples/` directory for practical demonstrations of key features.
- For module overviews, see the `README.md` files in each subdirectory (e.g., `src/coordinates/README.md`).


## Modules Overview

- **Coordinates**: 2D/3D coordinates, GPS, frame management, and geodetic conversion. See `src/coordinates/README.md` for details.
- **Maps**: Grid maps, path maps (graphs), and 3D point clouds for environment modeling and navigation. See `src/maps/README.md`.
- **Planning**: Path representation, A* and Dijkstra algorithms for navigation and motion planning. See `src/planning/README.md`.
- **Sensors**: Camera-based edge/line/obstacle detection, image differencing, and range sensor abstraction. See `src/sensors/README.md`.
- **Motors**: H-Bridge and Servo motor drivers for DC and RC servo control. See `src/motors/README.md`.
- **Vehicles**: High-level abstractions for cars (4WD, Ackerman), quadrotors, airplanes, and boats. See `src/vehicles/README.md`.
- **Communication**: Message-based communication framework supporting multiple protocols (UDP, ESPNow, Serial, LoRa, IEEE802_15_4, and more) for telemetry, remote control, and inter-robot messaging. See `src/communication/README.md`.
- **Concurrency**: Thread-safe queues, buffers, mutexes, and RTOS integration for multitasking and synchronization. See `src/concurrency/README.md`.
- **Units**: Strongly-typed units for distance, time, speed, and angle.
- **Utilities**: Logger, scheduler, serialization, and memory allocators for embedded systems.



## Quick Links to Module READMEs

- [Coordinates Module](src/coordinates/README.md)
- [Maps Module](src/maps/README.md)
- [Planning Module](src/planning/README.md)
- [Sensors Module](src/sensors/README.md)
- [Motors Module](src/motors/README.md)
- [Vehicles Module](src/vehicles/README.md)
- [Communication Module](src/communication/README.md)
- [Concurrency Module](src/concurrency/README.md)

---

## License

MIT License

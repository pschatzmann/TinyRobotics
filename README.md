# TinyRobotics

TinyRobotics is a modular, header-only C++ library for Arduino and embedded systems, providing essential building blocks for robotics, navigation, mapping, and sensor integration. The library is designed for flexibility and extensibility, supporting a variety of coordinate systems, path planning algorithms, and sensor types.

## Features

- **Coordinate Systems**: 2D/3D coordinates, GPS coordinates, NMEA sentence parsing, local cartesian frames, and hierarchical frame management (SE(2)/SE(3)).
- **Frame Management**: Manage trees of coordinate frames in 2D and 3D, with support for GPS/geodetic integration and conversion between local and global frames.
- **Path Planning**: Generic A* algorithm, path representation, and planning utilities for navigation and obstacle avoidance.
- **Mapping**: Grid maps (occupancy/value), path maps (graph-based), and 3D point clouds for environment modeling and SLAM.
- **Units**: Strongly-typed units for distance, time, speed, and angle to ensure type safety and clarity.
- **Sensors**: Range sensor abstraction, sensor fusion, and scheduling utilities for periodic sensor tasks.
- **Utilities**: Logger, scheduler, serialization, and fixed-capacity vector for embedded-friendly development.

## Directory Structure

```
src/
 TinyRobotics.h         # Main include file
 coordinates/           # Coordinate, GPS, and frame types (2D/3D)
 maps/                  # Map, path, and point cloud types
 planning/              # Path planning algorithms (A*, Kalman, etc.)
 sensors/               # Sensor abstractions
 units/                 # Units for distance, time, speed, angle
 utils/                 # Utility classes (Logger, Scheduler, VectorFromArray, etc.)
examples/
 record-gps/            # Example: record GPS data to a path
 frame2d/               # Example: 2D frame management
 frame3d/               # Example: 3D frame management
```

## Documentation

- Each class is documented in its header file with detailed descriptions and usage examples.
- See the `examples/` directory for practical demonstrations of key features.
- For module overviews, see the `README.md` files in each subdirectory (e.g., `src/coordinates/README.md`).

## License

MIT License

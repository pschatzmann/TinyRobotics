# TinyRobotics Coordinates Module

This directory contains classes and utilities for representing and manipulating coordinates and frames in 2D and 3D space, as well as geodetic (GPS) coordinates. These are foundational for robotics, mapping, navigation, and sensor fusion.

## Contents

- **Coordinate.h**  
  Generic 3D coordinate class (`Coordinate<T>`) for representing points in local or global Cartesian frames.  
  Features: distance, bearing, elevation, navigation, arithmetic, and serialization.

- **GPSCoordinate.h**  
  Represents a geodetic coordinate (latitude, longitude, altitude) using WGS84.  
  Features: geodesic distance, bearing, elevation, navigation, proximity checks, and serialization.

- **FrameMgr2D.h**  
  Manages a hierarchy of 2D coordinate frames (e.g., world, robot base, sensors) and enables SE(2) transforms and GPS conversion.  
  Features: frame tree management, transform computation, and conversion to GPS coordinates.

- **FrameMgr3D.h**  
  Manages a hierarchy of 3D coordinate frames (e.g., world, robot base, sensors) and enables SE(3) transforms and GPS conversion.  
  Features: frame tree management, quaternion-based transform computation, and conversion to GPS coordinates in 3D.

## Typical Usage

- Represent robot, sensor, or world positions in 2D/3D space.
- Convert between local/world frames and GPS coordinates.
- Compute distances, bearings, and elevation angles for navigation and mapping.
- Manage hierarchical coordinate frames for complex robots or sensor setups.

## Example

```cpp
#include <TinyRobotics.h>

Coordinate<float> local(1.0, 2.0, 0.5);
GPSCoordinate gps(48.8584, 2.2945, 35); // Eiffel Tower

float dist = local.distance(Coordinate<float>(2.0, 3.0, 1.0));
float bearing = local.bearing(Coordinate<float>(2.0, 3.0, 1.0));

// Define a simple frame hierarchy: world -> base -> lidar
FrameMgr2D mgr;
Frame2D world{FrameType::WORLD, 0};
Frame2D base{FrameType::BASE, 0, world, Transform2D(10, 20, 90)};
Frame2D lidar{FrameType::LIDAR, 0, base, Transform2D(0.2, -0.1, 0)};
mgr.setGPS(world, GPSCoordinate(46.2097, 7.2572, 503));

// Convert a local frame coordinate to GPS:
auto gps_result = mgr.toGPS(base);
```

## See Examples

- [2D Frame Example](../../../examples/coordinates&frames/frame2d/frame2d.ino)
- [3D Frame Example](../../../examples/coordinates&frames/frame3d/frame3d.ino)
- [Record GPS Example](../../../examples/coordinates&frames/record-gps/record-gps.ino)


## See Also

- [units/](../units/) for strongly-typed units (distance, angle, etc.)
- [maps/](../maps/) for map and path representations
- [sensors/](../sensors/) for sensor abstractions

---

This module is suitable for embedded and desktop robotics, mapping, and navigation applications.

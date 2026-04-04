
# TinyRobotics SLAM Module

This module provides robust 2D Simultaneous Localization and Mapping (SLAM) for robotics applications using the TinyRobotics library. It fuses IMU and range sensor data, maintains an efficient occupancy grid map, and supports flexible frame transforms for accurate world modeling and autonomous exploration.

## Features

- **Occupancy Grid Mapping:** Efficient 2D grid map with `GridBitMap` for storing free, occupied, and unknown cells.
- **IMU/Odometry Integration:** Use `IMU2D` or any `IMotionState2D` implementation for pose estimation.
- **Range Sensor Fusion:** Integrate LIDAR or other range sensors with `RangeSensor` for obstacle detection and map updates.
- **Frame Management:** Use `Frame2D` and `FrameMgr2D` for coordinate transforms between world, base, and sensor frames.
- **Frontier-Based Exploration:** Autonomous navigation with `FrontierExplorer` and the `IFrontierExplorer` interface.
- **Message Bus:** Modular sensor fusion and communication via the message bus architecture.

## Example Usage

```cpp
#include <TinyRobotics.h>

// Frame definitions
Frame2D world, base, lidar;
// Map: 10x10 meters, 10cm resolution
GridBitMap<float> map(100, 100, 0.1f);
// IMU
IMU2D<float> imu;
// Frontier-based exploration
FrontierExplorer<float> explorer(map);
// SLAM system
LocalizationAndMapping2D<float> slam(map, explorer, world, base, lidar, imu);

// Subscribe to sensor messages
slam.subscribe(handler);

// Add a Lidar measurement (distance in meters, angle in degrees, cell state)
slam.addRangeMeasurement(distance, angle, CellState::OCCUPIED);

// Access the map
auto& mymap = slam.getMap();

// Get the next exploration frontier
Coordinate<float> next;
if (slam.getNextFrontier(next)) {
	// Use 'next' as the next goal
}
```

## Main Classes

- `LocalizationAndMapping2D<T>`: Main SLAM class (modular, template-based)
- `GridBitMap<T>`: 2D occupancy grid map
- `IMU2D<T>`: 2D IMU/odometry sensor fusion
- `RangeSensor<T>`: Range/obstacle sensor abstraction
- `Frame2D`, `FrameMgr2D`: Frame transforms
- `FrontierExplorer<T>`, `IFrontierExplorer<T>`: Autonomous exploration

## See Examples

- [Pointcloud A* Example](../../../examples/maps&planning/pointcloud-astar/pointcloud-astar.ino)
- [Other Example Sketches](../../../examples/)

## See Also

- [TinyRobotics Documentation](https://github.com/pschatzmann/TinyRobotics)
- Example sketches in the `examples/` directory

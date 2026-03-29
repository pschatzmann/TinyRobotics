## TinyRobotics SLAM Module

This module provides 2D Simultaneous Localization and Mapping (SLAM) capabilities for robotics applications using the TinyRobotics library. It integrates IMU and range sensor data, maintains an occupancy grid map, and supports frame transforms for accurate world modeling.

### Features
- Occupancy grid mapping with `GridMap<CellState>`
- IMU2D-based pose estimation
- RangeSensor-based obstacle detection
- Frame2D and FrameMgr2D for coordinate transforms
- Efficient map updates (free/occupied cells)
- Frontier-based exploration support
- Message bus integration for modular sensor fusion

### Example Usage
```cpp
#include <TinyRobotics.h>

Frame2D world, base, lidar;
FrameMgr2D tf;
SLAM2D slam(10.0, 10.0, 0.1, world, base, lidar, tf); // 10x10m map, 10cm resolution

// Subscribe to messages
slam.subscribe(handler);

// IMU update
slam.update(accelX, accelY, gyroZ, nowMillis);

// Lidar update
slam.addRangeMeasurement(distance, angle);

// Publish IMU state and update base frame
slam.publish();

// Access the map
const auto& map = slam.getMap();

// Get next exploration frontier
auto next = slam.getNextFrontier();
```

### Classes
- `SLAM2D`: Main class for 2D SLAM with frame transforms and message integration
- `GridMap<CellState>`: Occupancy grid map
- `IMU2D`: 2D IMU sensor fusion
- `RangeSensor`: Range/obstacle sensor abstraction
- `Frame2D`, `FrameMgr2D`: Frame transforms

### See Also
- [TinyRobotics Documentation](https://github.com/pschatzmann/TinyRobotics)
- Example sketches in the `examples/` directory

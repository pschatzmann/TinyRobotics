# TinyRobotics Maps Module

This directory contains classes and utilities for representing spatial maps and navigation graphs, which are essential for robotics, SLAM, path planning, and environment modeling.

## Contents

- **GridMap.h**  
  2D occupancy or value grid for representing environments as rasterized cells.  
  Features: cell state management, world-to-grid conversion, neighbor queries, and probabilistic updates.

- **PathMap.h**  
  Graph structure for pathfinding, where nodes are coordinates and edges are path segments.  
  Features: directed/undirected segments, segment queries, and support for algorithms like A* and Dijkstra.

- **PointCloud.h**  
  Efficient 3D point cloud for mapping and obstacle detection.  
  Features: point addition, bounding box tracking, voxel grid downsampling, and occupancy queries.

## Typical Usage

- Model environments as occupancy grids for mapping and localization.
- Represent road networks or navigation graphs for path planning.
- Store and process 3D point clouds from sensors like LIDAR.
- Perform fast spatial queries for collision checking and navigation.

## Example

```cpp
#include "GridMap.h"
#include "PathMap.h"
#include "PointCloud.h"

// Create a 2D grid map
GridMap<> grid(100, 100, 0.1f); // 100x100 cells, 10cm resolution

// Add a path segment to a path map
PathMap<Coordinate<float>> pathMap;
pathMap.addSegment({a, b, false}); // Add undirected segment from a to b

// Add a point to a point cloud
PointCloud cloud;
cloud.add(1.0, 2.0, 0.5);
cloud.buildVoxelGrid(0.1f); // 10cm voxels
```

## See Also

- [coordinates/](../coordinates/) for coordinate and frame representations
- [planning/](../planning/) for path planning algorithms
- [sensors/](../sensors/) for sensor abstractions

---

This module is designed for embedded and desktop robotics, mapping, and navigation applications.
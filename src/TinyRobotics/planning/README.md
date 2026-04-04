# TinyRobotics Planning Module

This directory contains algorithms and utilities for path planning, motion planning, and trajectory generation—core components for autonomous navigation and robotics.

## Contents

- **Path.h**  
  Path representation as a sequence of waypoints or segments.  
  Features: path construction, reversal, length calculation, and interpolation.

- **AStar.h**  
  Flexible A* pathfinding algorithm for any map implementing IMap<T> and using Coordinate<T> nodes.  
  Features: interface-based design, customizable cost and validity callbacks, and support for arbitrary map types.

- **Dijkstra.h**  
  Flexible Dijkstra shortest path algorithm for any map implementing IMap<T> and using Coordinate<T> nodes.  
  Features: callback-based cost and validity, interface-based map integration, and optimal path computation without heuristics.

- **FrontierExplorer.h**  
  Generic frontier-based exploration utility for autonomous mapping and SLAM.  
  Features: supports multiple selection strategies (random, nearest, farthest, first, last, custom), works with any grid/occupancy map, and provides flexible integration for exploration tasks.


## Typical Usage

- Find the shortest or optimal path between two points in a grid, map, or graph.
- Represent and manipulate robot paths for following, smoothing, or analysis.
- Integrate planning algorithms with mapping and localization modules for autonomous navigation.
- Perform frontier-based exploration for mapping unknown environments or enhancing SLAM performance.

## Example

```cpp
#include <TinyRobotics.h>

// Define your map type (must implement IMap<T>)
GridMap grid(50, 50, 0.1f);
auto start = grid.worldToGrid(0.0f, 0.0f);
auto goal = grid.worldToGrid(4.0f, 4.0f);

AStar astar;
auto path = astar.findPath(grid, start, goal);

if (!path.empty()) {
    for (const auto& node : path) {
        // Process or visualize the path
    }
}
```

## See Examples

- [Gridmap A*](../../../examples/maps&planning/gridmap-astar/gridmap-astar.ino)
- [Gridmap Dijkstra](../../../examples/maps&planning/gridmap-dijkstra/gridmap-dijkstra.ino)
- [Pathmap A*](../../../examples/maps&planning/pathmap-astar/pathmap-astar.ino)
- [Planned Path](../../../examples/maps&planning/plannedpath/plannedpath.ino)
- [Pointcloud A*](../../../examples/maps&planning/pointcloud-astar/pointcloud-astar.ino)

## See Also

- [coordinates/](../coordinates/) for coordinate and frame representations
- [maps/](../maps/) for map and graph data structures
- [sensors/](../sensors/) for sensor integration

---

This module is designed for embedded and desktop robotics, autonomous vehicles, and any application requiring robust path or motion planning.
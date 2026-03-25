# TinyRobotics Planning Module

This directory contains algorithms and utilities for path planning, motion planning, and trajectory generation—core components for autonomous navigation and robotics.

## Contents

- **Path.h**  
  Path representation as a sequence of waypoints or segments.  
  Features: path construction, reversal, length calculation, and interpolation.

- **AStar.h**  
  Generic and extensible A* pathfinding algorithm for graphs, grids, and navigation maps.  
  Features: iterative search, customizable heuristics, and support for user-defined map types.

- **Dijkstra.h**  
  Generic Dijkstra shortest path algorithm for graphs, grids, and navigation maps.  
  Features: callback-based cost and validity, flexible map integration, and optimal path computation without heuristics.


## Typical Usage

- Find the shortest or optimal path between two points in a grid, map, or graph.
- Represent and manipulate robot paths for following, smoothing, or analysis.
- Integrate planning algorithms with mapping and localization modules for autonomous navigation.

## Example

```cpp
#include "AStar.h"
#include "Path.h"

// Define your map or graph type (e.g., GridMap, PathMap)
tinyrobotics::GridMap<> grid(50, 50, 0.1f);
auto start = grid.worldToGrid(0.0f, 0.0f);
auto goal = grid.worldToGrid(4.0f, 4.0f);

tinyrobotics::AStar<decltype(grid)> astar(grid);
auto path = astar.findPath(start, goal);

if (!path.empty()) {
    for (const auto& node : path) {
        // Process or visualize the path
    }
}
```

## See Also

- [coordinates/](../coordinates/) for coordinate and frame representations
- [maps/](../maps/) for map and graph data structures
- [sensors/](../sensors/) for sensor integration

---

This module is designed for embedded and desktop robotics, autonomous vehicles, and any application requiring robust path or motion planning.
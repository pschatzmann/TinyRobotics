#pragma once

#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/utils/Common.h"

namespace tinyrobotics {

/**
 * @class PathSegment
 * @brief Represents a path segment (edge) between two coordinates in a graph.
 *
 * Each segment connects two nodes (coordinates) and can be used to define the
 * edges in a graph for pathfinding algorithms such as A* or Dijkstra's. The
 * segment has:
 *   - a start node (from)
 *   - an end node (to)
 *   - a cost (distance, time, or any metric relevant to the problem)
 *   - a directionality flag (directed or undirected)
 *
 * This class is a fundamental building block for creating a path map for
 * navigation, motion planning, and graph-based search.
 *
 * @tparam Coordinate The coordinate type (e.g., 2D or 3D point).
 */
template <typename CoordinateT = Coordinate<DistanceM>>
class PathSegment {
 public:
  CoordinateT from;
  CoordinateT to;
  float cost = 0.0;  // Default cost for traversing this segment
  bool directed = false;
};

} // namespace tinyrobotics
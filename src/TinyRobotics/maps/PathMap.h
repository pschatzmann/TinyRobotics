#pragma once
#include "TinyRobotics/utils/AllocatorPSRAM.h"

namespace tinyrobotics {

/**
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
template <typename Coordinate = Coordinate<DistanceM>>
class PathSegment {
 public:
  Coordinate from;
  Coordinate to;
  float cost = 0.0;  // Default cost for traversing this segment
  bool directed = false;
};

/**
 * @brief Represents a graph of valid path segments (edges) between coordinates
 * (nodes).
 *
 * PathMap is used for pathfinding algorithms (A*, Dijkstra, etc.) to find
 * optimal paths between nodes. The map can contain both directed (one-way) and
 * undirected (two-way) segments.
 *
 * Features:
 *   - Add segments (edges) between coordinates (nodes)
 *   - Query outgoing and incoming segments for a given node
 *   - Supports both directed and undirected graphs
 *
 * Typical usage: model a road network, robot navigation graph, or any scenario
 * where movement is constrained to a set of valid connections.
 *
 * @tparam Coordinate The coordinate type (e.g., 2D or 3D point).
 */
template <typename Coordinate = Coordinate<float>>
class PathMap {
  void addSegment(const Coordinate& from, const Coordinate& to,
                  bool directed = false) {
    auto distance = from.distance(to);  // Example cost based on distance
    PathSegment<Coordinate> segment{from, to, distance, directed};  // Default cost = 1.0
    segments.push_back(segment);
  }

  std::vector<PathSegment<Coordinate>> getSegments(
      const Coordinate& from) const {
    std::vector<PathSegment<Coordinate>> result;
    for (const auto& seg : segments) {
      if (seg.from == from) {
        result.push_back(seg);
      }
    }
    for (const auto& seg : segments) {
      if (seg.to == from && !seg.directed) {
        PathSegment<Coordinate> reverse_seg = seg;
        std::swap(reverse_seg.from, reverse_seg.to);
        result.push_back(reverse_seg);
      }
    }
    return result;
  }

  std::vector<Coordinate> getNeighbors(const Coordinate& from) const {
    std::vector<Coordinate> neighbors;
    for (const auto& seg : segments) {
      if (seg.from == from) {
        neighbors.push_back(seg.to);
      }
      if (seg.to == from && !seg.directed) {
        neighbors.push_back(seg.from);
      }
    }
    return neighbors;
  }

protected:
  std::vector<PathSegment<Coordinate>, tinyrobotics::AllocatorPSRAM<PathSegment<Coordinate>>> segments;
};

}  // namespace tinyrobotics

#pragma once

#include "TinyRobotics/maps/IMap.h"
#include "TinyRobotics/maps/PathSegment.h"
#include "TinyRobotics/utils/AllocatorPSRAM.h"

namespace tinyrobotics {

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
template <typename CoordinateT = Coordinate<float>>
class PathMap : public IMapNeighbors<typename std::remove_reference<
                    decltype(std::declval<CoordinateT>().x)>::type> {
 public:
  PathMap() = default;

  void addSegment(const CoordinateT& from, const CoordinateT& to,
                  bool directed = false) {
    auto distance = from.distance(to);  // Example cost based on distance
    PathSegment<CoordinateT> segment{from, to, distance,
                                     directed};  // Default cost = 1.0
    segments.push_back(segment);
  }

  void addSegment(PathSegment<CoordinateT> segment) {
    segments.push_back(segment);
  }

  std::vector<PathSegment<CoordinateT>> getSegments(
      const CoordinateT& from) const {
    std::vector<PathSegment<CoordinateT>> result;
    for (const auto& seg : segments) {
      if (seg.from == from) {
        result.push_back(seg);
      }
    }
    for (const auto& seg : segments) {
      if (seg.to == from && !seg.directed) {
        PathSegment<CoordinateT> reverse_seg = seg;
        std::swap(reverse_seg.from, reverse_seg.to);
        result.push_back(reverse_seg);
      }
    }
    return result;
  }

  // IMap interface: getNeighbors
  std::vector<CoordinateT> getNeighbors(CoordinateT from) const override {
    std::vector<CoordinateT> neighbors;
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

  // IMap interface: isValid (always true for graph nodes in PathMap)
  bool isValid(const Coordinate<float>& coord) const override {
    // A node is valid if it appears in any segment
    for (const auto& seg : segments) {
      if (seg.from == coord || seg.to == coord) return true;
    }
    return false;
  }

  size_t size() const { return segments.size(); }

  PathSegment<CoordinateT>& operator[](size_t index) { return segments[index]; }

  size_t writeTo(Print& out) { return serializer.write(*this, out); }

  size_t readFrom(Stream& in) { return serializer.read(*this, in); }

  void clear() { segments.clear(); }

 protected:
  std::vector<PathSegment<CoordinateT>,
              AllocatorPSRAM<PathSegment<CoordinateT>>>
      segments;
  PathMapSerializer<PathMap<CoordinateT>, CoordinateT> serializer;
};

}  // namespace tinyrobotics

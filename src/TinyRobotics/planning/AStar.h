#pragma once
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#include "Path.h"
#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/utils/AllocatorPSRAM.h"

namespace tinyrobotics {

/***
 * @brief Generic, flexible A* pathfinding algorithm for arbitrary graphs,
 * grids, or maps.
 *
 * The AStar class implements the A* search algorithm and is designed to work
 * with any map or graph type that provides a `getNeighbors(node)` method. It
 * supports user-defined cost and validity callbacks, allowing for custom cost
 * metrics, obstacle handling, and heuristic functions.
 *
 * Features:
 *   - Finds the optimal path from a start node to a goal node using A* search
 *   - Supports user-provided cost and validity callbacks for maximum
 * flexibility
 *   - Allows passing a user reference/context pointer to callbacks (for map
 * data, etc.)
 *   - Provides both full path reconstruction and efficient next-step queries
 *   - Suitable for grids, navigation graphs, road networks, and custom map
 * types
 *
 * Usage:
 *   1. Create an AStar instance, providing cost and validity callbacks as
 * needed.
 *   2. Call `findPath(map, start, goal)` to get the optimal path as a Path
 * object.
 *   3. Optionally, use `nextStep(map, start, goal)` to get only the next move.
 *   4. Customize cost and heuristic logic via the cost callback (e.g.,
 * Euclidean, Manhattan, or domain-specific).
 *
 * Example:
 * @code
 * AStar<MyMapType> astar;
 * astar.setCostCallback([](const Node& from, const Node& to, void*) {
 *   // Custom cost (e.g., Euclidean distance)
 *   return from.distance(to);
 * });
 * auto path = astar.findPath(map, start, goal);
 * if (!path.empty()) {
 *   // Use the path
 * }
 * @endcode
 *
 * This class is suitable for embedded and desktop robotics, navigation, and any
 * application requiring flexible, efficient pathfinding.
 */

template <typename MapType, typename Node = Coordinate<DistanceM>>
class AStar {
 public:
  using CostCallback = std::function<float(const Node&, const Node&, void* ref)>;
  using ValidCallback = std::function<bool(const Node&, void* ref)>;

 public:
  /// provide a callback to determine the cost of moving from one node to
  /// another
  void setCostCallback(CostCallback cb) { cost_cb = cb; }
  /// provide a callback to determine if a node is valid (e.g., not an obstacle)
  void setValidCallback(ValidCallback cb) { valid_cb = cb; }
  /// provide reference for callbacks (e.g., to access map data or other
  /// context)
  void setReference(void* reference) { ref = reference; }

  /// Finds the optimal path from start to goal. Returns an empty path if no path
  Path<Node> findPath(const MapType& map, const Node start, const Node goal) {
    std::string startStr = start.toCString();
    std::string goalStr = goal.toCString();
    TRLogger.debug("Finding path from %s to %s", startStr.c_str(), goalStr.c_str());
    using NodeMap = std::unordered_map<Node, Node, std::hash<Node>, std::equal_to<Node>, AllocatorPSRAM<std::pair<const Node, Node>>>;
    NodeMap cameFrom;
    bool found = aStarSearch(map, start, goal, &cameFrom, nullptr);
    if (found) {
      return reconstructPath(cameFrom, start, goal);
    }
    return Path<Node>();
  }

  /**
   * Returns the next node on the optimal path from start to goal.
   * If no path is found, returns start.
   */
  Node nextStep(const MapType& map, const Node& start, const Node& goal) {
    using NodeMap = std::unordered_map<Node, Node, std::hash<Node>, std::equal_to<Node>, AllocatorPSRAM<std::pair<const Node, Node>>>;
    NodeMap cameFrom;
    bool found = aStarSearch(map, start, goal, &cameFrom, nullptr);
    if (found) {
      Path<Node> path = reconstructPath(cameFrom, start, goal);
      if (path.size() >= 2) {
        return path[1];
      }
    }
    return start;
  }

 protected:
  struct NodeRecord {
    Node node;
    float costSoFar = 0;
    float estimatedTotalCost = 0;
    bool operator>(const NodeRecord& other) const {
      return estimatedTotalCost > other.estimatedTotalCost;
    }
  };

  // Core A* search logic, fills cameFrom and optionally costSoFar, returns true
  // if path found
  bool aStarSearch(
      const MapType& map, const Node& start,
      const Node& goal,
      std::unordered_map<Node, Node, std::hash<Node>, std::equal_to<Node>, AllocatorPSRAM<std::pair<const Node, Node>>>* cameFrom,
      std::unordered_map<Node, float, std::hash<Node>, std::equal_to<Node>, AllocatorPSRAM<std::pair<const Node, float>>>* outCostSoFar) {

    std::priority_queue<NodeRecord, std::vector<NodeRecord>,
                        std::greater<NodeRecord>>
        openSet;
    std::unordered_map<Node, float, std::hash<Node>, std::equal_to<Node>, AllocatorPSRAM<std::pair<const Node, float>>> costSoFar;
    openSet.push({start, 0.0f, cost_cb(start, goal, ref)});
    costSoFar[start] = 0.0f;
    if (cameFrom) cameFrom->clear();
    while (!openSet.empty()) {
      Node current = openSet.top().node;
      if (current == goal) {
        if (outCostSoFar) *outCostSoFar = costSoFar;
        return true;
      }
      openSet.pop();
      for (const auto& neighbor : map.getNeighbors(current)) {
        if (!valid_cb(neighbor, ref)) continue;
        float newCost = costSoFar[current] + cost_cb(current, neighbor, ref);
        if (!costSoFar.count(neighbor) || newCost < costSoFar[neighbor]) {
          costSoFar[neighbor] = newCost;
          float priority = newCost + cost_cb(neighbor, goal, ref);
          openSet.push({neighbor, newCost, priority});
          if (cameFrom) (*cameFrom)[neighbor] = current;
        }
      }
    }
    if (outCostSoFar) *outCostSoFar = costSoFar;
    return false;
  }

 protected:
  CostCallback cost_cb = defaultCost;
  ValidCallback valid_cb = defaultValid;
  void* ref = this;

  static float defaultCost(const Node& from, const Node& to, void* ref) {
    float result = from.distance(to);
    return result;
  }

  static bool defaultValid(const Node& node, void* ref) {
    return true;  // By default, all nodes are valid
  }

  Path<Node> reconstructPath(
      const std::unordered_map<Node, Node, std::hash<Node>, std::equal_to<Node>, AllocatorPSRAM<std::pair<const Node, Node>>>& cameFrom,
      const Node& start, const Node& goal) {
    Path<Node> path;
    Node current = goal;
    while (current != start) {
      path.addWaypoint(current);
      auto it = cameFrom.find(current);
      if (it == cameFrom.end()) break;
      current = it->second;
    }
    path.addWaypoint(start);
    path.reverse();
    return path;
  }
};

}  // namespace tinyrobotics
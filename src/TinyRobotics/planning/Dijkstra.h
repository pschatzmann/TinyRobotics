#pragma once
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/planning/Path.h"
#include "TinyRobotics/utils/Common.h"

namespace tinyrobotics {

/**
 * @class Dijkstra
 * @ingroup planning
 * @brief Generic Dijkstra shortest path algorithm for graphs/maps with
 * callback-based cost and validity.
 *
 * @tparam MapType Must provide:
 *   - std::vector<Node> getNeighbors(const Node& node) const
 */
template <typename T = DistanceM>
class Dijkstra {
 public:
  using CostCallback = std::function<float(const Coordinate<T>&,
                                           const Coordinate<T>&, void* ref)>;
  using ValidCallback = std::function<bool(const Coordinate<T>&, void* ref)>;

  Dijkstra() = default;

  /// provide a callback to determine the cost of moving from one node to
  /// another
  void setCostCallback(CostCallback cb) { cost_cb = cb; }
  /// provide a callback to determine if a node is valid (e.g., not an obstacle)
  void setValidCallback(ValidCallback cb) { valid_cb = cb; }
  /// provide reference for callbacks (e.g., to access map data or other
  /// context)
  void setReference(void* reference) { ref = reference; }

  /**
   * @brief Finds the shortest path from start to goal.
   * @param start The starting node.
   * @param goal The goal node.
   * @return Vector of nodes representing the path (empty if no path found).
   */
  Path<Coordinate<T>> findPath(const IMapNeighbors<T>& map, const Coordinate<T>& start,
                               const Coordinate<T>& goal) {
    using Pair = std::pair<float, Coordinate<T>>;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> open;
    std::unordered_map<Coordinate<T>, float> cost_so_far;
    std::unordered_map<Coordinate<T>, Coordinate<T>> came_from;

    open.emplace(0.0f, start);
    cost_so_far[start] = 0.0f;
    came_from[start] = start;

    while (!open.empty()) {
      Coordinate<T> current = open.top().second;
      open.pop();

      if (current == goal) break;

      for (const Coordinate<T>& next : map.getNeighbors(current)) {
        if (!valid_cb(next, ref)) continue;
        float new_cost = cost_so_far[current] + cost_cb(current, next, ref);
        if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
          cost_so_far[next] = new_cost;
          came_from[next] = current;
          open.emplace(new_cost, next);
        }
      }
    }

    // Reconstruct path
    Path<Coordinate<T>> path;
    if (!came_from.count(goal)) return path;
    for (Coordinate<T> at = goal; at != start; at = came_from[at]) {
      path.addWaypoint(at);
    }
    path.addWaypoint(start);
    path.reverse();
    return path;
  }

 protected:
  CostCallback cost_cb = defaultCost;
  ValidCallback valid_cb = defaultValid;
  void* ref = this;

  static float defaultCost(const Coordinate<T>& from, const Coordinate<T>& to,
                           void* ref) {
    return from.distance(to);
  }

  static bool defaultValid(const Coordinate<T>& node, void* ref) {
    return true;  // By default, all nodes are valid
  }
};

}  // namespace tinyrobotics
#pragma once
#include "TinyRobotics/coordinates/Coordinate.h"

namespace tinyrobotics {

/// Strategy for selecting the next frontier cell to explore
enum class FrontierSelectionStrategy {
  RANDOM,    // Randomly select a frontier cell
  NEAREST,   // Select the frontier cell closest to the robot
  FARTHEST,  // Select the frontier cell farthest from the robot
  FIRST,     // Select the first frontier cell found (e.g., in scan order)
  LAST,      // Select the last frontier cell found
  SWITCH_FIRST_LAST,  // Alternate between first and last on each call
  CUSTOM              // Use a user-defined callback to select the frontier
};

/**
 * @class FrontierExplorer
 * @brief Generic exploration and frontier-based SLAM utility for grid or
 * occupancy maps.
 *
 * The FrontierExplorer class provides a flexible framework for autonomous
 * exploration of a 2D or 3D environment using a frontier-based approach. It is
 * designed to work with any map type that provides cell state access (e.g.,
 * occupancy grid, bit vector map) and any coordinate type. The class supports
 * multiple strategies for selecting the next frontier to explore, including
 * random, nearest, farthest, first, last, and custom selection.
 *
 * Typical usage:
 *   - Set the robot's current position with setCurrentPosition().
 *   - Optionally set the frontier selection strategy with setStrategy().
 *   - Optionally provide a custom selection callback for advanced strategies.
 *   - Call getNextFrontier() to retrieve the next cell to explore.
 *
 * Features:
 *   - Works with any map type supporting getCell(x, y, state) and
 * getXCount()/getYCount().
 *   - Finds all frontiers (cells adjacent to unknown space) in the map.
 *   - Supports multiple selection strategies for exploration.
 *   - Allows user-defined custom selection logic.
 *
 * @tparam MapType The type of the map (e.g., GridMapBitVector,
 * GridMap<CellState>)
 * @tparam Coordinate The coordinate type (default: Coordinate<float>)
 */
template <typename MapType, typename CoordinateT = Coordinate<float>>
class FrontierExplorer {
 public:
  /**
   * @brief Construct a FrontierExplorer with a given map (by reference).
   * @param map The map to explore (must outlive this object).
   */
  FrontierExplorer(MapType& map) : map_(map) {}

  /**
   * @brief Set the current position of the explorer.
   * @param pos The current position.
   */
  void setCurrentPosition(const CoordinateT& pos) { current_pos = pos; }

  /**
   * @brief Get the current position of the explorer.
   * @return The current position.
   */
  CoordinateT getCurrentPosition() const { return current_pos; }

  /**
   * @brief Set the strategy for selecting the next frontier cell.
   * @param strategy The selection strategy to use.
   */
  void setStrategy(FrontierSelectionStrategy strategy) {
    strategy_ = strategy;
    if (strategy == FrontierSelectionStrategy::SWITCH_FIRST_LAST) {
      switchFirstLast = true;
    } else {
      switchFirstLast = false;
    }
  }

  /**
   * @brief Get the current frontier selection strategy.
   * @return The current strategy.
   */
  FrontierSelectionStrategy getStrategy() const {
    if (switchFirstLast) return FrontierSelectionStrategy::SWITCH_FIRST_LAST;
    return strategy_;
  }

  /**
   * @brief Provides the next frontier cell to explore based on the current
   * strategy.
   * @param nextCell Output parameter for the selected frontier cell.
   * @return true if a frontier was found, false otherwise.
   */
  bool getNextFrontier(CoordinateT& nextCell) {
     // find all frontier cells
     collectFrontiers();
     record_count_ = frontiers.size();
     if (frontiers.empty()) return false;

     // switch between FIRST and LAST if the strategy is SWITCH_FIRST_LAST
     if (switchFirstLast) {
       // switch between FIRST and LAST on each call
       strategy_ = (strategy_ == FrontierSelectionStrategy::FIRST)
                       ? FrontierSelectionStrategy::LAST
                       : FrontierSelectionStrategy::FIRST;
     }

     switch (strategy_) {
       case FrontierSelectionStrategy::RANDOM:
         return selectRandom(nextCell);
       case FrontierSelectionStrategy::NEAREST:
         return selectNearest(nextCell);
       case FrontierSelectionStrategy::FARTHEST:
         return selectFarthest(nextCell);
       case FrontierSelectionStrategy::FIRST:
         return selectFirst(nextCell);
       case FrontierSelectionStrategy::LAST:
         return selectLast(nextCell);
       case FrontierSelectionStrategy::CUSTOM:
         return selectCustom(nextCell);
     }
     return false;  // Should not reach here
   }

   /// Provides the number of potential frontier cells found in the last search.
   size_t size() const { return record_count_; }

  protected:
   CoordinateT current_pos{};
   FrontierSelectionStrategy strategy_ = FrontierSelectionStrategy::RANDOM;
   bool switchFirstLast = false;
   size_t record_count_ = 0;
   MapType & map_;
   std::vector<CoordinateT> frontiers;
   void* ref = this;
   int (*select_callback)(std::vector<CoordinateT>& frontiers,
                          void* ref) = nullptr;

   // --- Strategy implementations ---
   bool selectRandom(CoordinateT& nextCell) {
     int idx = random(0, frontiers.size());
     nextCell = frontiers[idx];
     frontiers.clear();
     return true;
   }

   bool selectNearest(CoordinateT& nextCell) {
     CoordinateT& result = frontiers[0];
     float distance = current_pos.distance(result);
     for (int i = 1; i < frontiers.size(); ++i) {
       CoordinateT& tmp = frontiers[i];
       float tmp_distance = current_pos.distance(tmp);
       if (tmp_distance < distance) {
         result = tmp;
         distance = tmp_distance;
       }
     }
     nextCell = result;
     frontiers.clear();
     return true;
   }

   bool selectFarthest(CoordinateT& nextCell) {
     CoordinateT& result = frontiers[0];
     float distance = current_pos.distance(result);
     for (int i = 1; i < frontiers.size(); ++i) {
       CoordinateT& tmp = frontiers[i];
       float tmp_distance = current_pos.distance(tmp);
       if (tmp_distance > distance) {
         result = tmp;
         distance = tmp_distance;
       }
     }
     nextCell = result;
     frontiers.clear();
     return true;
   }

   bool selectFirst(CoordinateT& nextCell) {
     nextCell = frontiers.front();
     frontiers.clear();
     return true;
   }

   bool selectLast(CoordinateT& nextCell) {
     nextCell = frontiers.back();
     frontiers.clear();
     return true;
   }

   bool selectCustom(CoordinateT& nextCell) {
     if (select_callback) {
       int idx = select_callback(frontiers, ref);
       if (idx >= 0 && idx < frontiers.size()) {
         nextCell = frontiers[idx];
         frontiers.clear();
         return true;
       }
     }
     frontiers.clear();
     return false;
   }

   /**
    * @brief Set a custom callback for selecting the next frontier cell.
    * You can implement your own optimization strategy here.
    * @param cb The callback function (returns index of selected cell).
    */
   void setSelectCallback(int (*cb)(std::vector<CoordinateT>& frontiers,
                                    void* ref),
                          void* ref = nullptr) {
     select_callback = cb;
     if (ref != nullptr)
       this->ref = ref;  // Default reference is the FrontierExplorer instance
     strategy_ = FrontierSelectionStrategy::CUSTOM;
   }

   /**
    * @brief Find all frontier cells in the map (cells adjacent to unknown
    * space).
    *
    * A frontier cell is defined as a cell that is FREE (traversable) and has at
    * least one neighboring cell (in any of the 8 directions) that is UNKNOWN
    * (unexplored). This method iterates over every cell in the map, and for
    * each FREE cell, checks all 8 neighbors. If any neighbor is UNKNOWN, the
    * cell is added to the internal `frontiers` vector. Each cell is considered
    * only once, so no duplicates occur.
    *
    * This is a key step in frontier-based exploration and SLAM, as it
    * identifies the boundary between explored and unexplored space, guiding the
    * robot to new areas.
    *
    * After calling this method, the `frontiers` vector will contain all current
    * frontier cells, which can then be used by the exploration strategy to
    * select the next goal.
    */
   void collectFrontiers() {
     frontiers.clear();
     for (int y = 0; y < map_.getYCount(); ++y) {
       for (int x = 0; x < map_.getXCount(); ++x) {
         CellState state;
         if (!map_.getCell(x, y, state) || state != CellState::FREE) continue;
         bool found = false;
         // Check 8 neighbors for UNKNOWN
         for (int dy = -1; dy <= 1 && !found; ++dy) {
           for (int dx = -1; dx <= 1 && !found; ++dx) {
             if (dx == 0 && dy == 0) continue;
             CellState neighborState;
             if (map_.getCell(x + dx, y + dy, neighborState) &&
                 neighborState == CellState::UNKNOWN) {
              // return world coordinates of the frontier cell 
               frontiers.push_back(map_.toWorld(x, y));
               found = true;
             }
           }
         }
       }
     }
   }
};

}  // namespace tinyrobotics

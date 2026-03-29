/**
 * @file girdmap-dijkstra
 * @brief Example: Dijkstra pathfinding on a 2D grid map using TinyRobotics.
 *
 * Demonstrates how to use the TinyRobotics GridMap and Dijkstra classes to perform
 * pathfinding in a 2D grid environment. Obstacles are represented as occupied cells.
 *
 * - Builds a 2D grid map with free and occupied cells.
 * - Uses Dijkstra's algorithm to find a path from start to goal, avoiding obstacles.
 * - Prints the resulting path and total distance to Serial.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 *
 * @author Phil Schatzmann
 */
#include <TinyRobotics.h>

// Dimensions
const int width = 5;
const int height = 5;
float resolution = 1.0f;  // 1 meter per cell

// Define a simple 2D grid map (5x5)
int grid[height][width] = {{0, 0, 0, 0, 0},
                           {0, 1, 1, 1, 0},
                           {0, 0, 0, 1, 0},
                           {0, 1, 0, 0, 0},
                           {0, 0, 0, 1, 0}};

// Define map
GridMap<CellState> gridMap(width, height, resolution);


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("TinyRobotics GridMap A* Example");

  // Initialize the GridMap from the grid
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      gridMap.setCell(x, y, grid[y][x] == 0 ? CellState::FREE : CellState::OCCUPIED);
    }
  }
}


void loop() {

  // set random start and goal
  Coordinate<DistanceM> start = gridMap.toWorld(random(0, width), random(0, height));
  Coordinate<DistanceM> goal = gridMap.toWorld(random(0, width), random(0, height));

  // Find path using Dijkstra
  Dijkstra<GridMap<CellState>> dijkstra;
  auto path = dijkstra.findPath(gridMap, start, goal);

  if (path.isEmpty()) {
    Serial.println("=> No path found!");
    return;
  }

  // Print the path
  for ( auto &step : path.getWaypoints()) {
    Serial.println(step.toCString());
  }
  Serial.print("Total distance: ");
  Serial.println(path.distance());

  delay(2000);  // Wait before next search
}


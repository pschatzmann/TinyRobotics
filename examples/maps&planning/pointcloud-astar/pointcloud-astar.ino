/**
 * @file pointcloud-astar.ino
 * @brief Example: A* pathfinding on a 2D point cloud using TinyRobotics.
 *
 * Demonstrates how to use the TinyRobotics PointCloud and AStar classes to perform
 * pathfinding in a 2D grid environment. Obstacles are represented as points in the cloud.
 *
 * - Builds a 2D point cloud from a grid map.
 * - Uses A* to find a path from start to goal, avoiding obstacles.
 * - Prints the resulting path and total distance to Serial.
 * - In real life the point cloud could come from a LIDAR or depth sensor
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 *
 * @author Phil Schatzmann
 */
#include <TinyRobotics.h>

// Define a larger 2D point cloud (5x5 grid)
const int width = 5;
const int height = 5;
int grid[height][width] = {{0, 0, 0, 0, 0},
                           {0, 1, 1, 1, 0},
                           {0, 0, 0, 1, 0},
                           {0, 1, 0, 0, 0},
                           {0, 0, 0, 1, 0}};

PointCloud cloud;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("TinyRobotics PointCloud A* Example");

  // Initialize the PointCloud from the grid
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (grid[y][x] == 1) cloud.add(x, y);
    }
  }

  // This example has no upper, left and right bounds: we want to allow the full range
  cloud.setBounds(0, 0, 0, 5, 5, 0);
  // Build voxel grid for fast occupancy queries
  cloud.buildVoxelGrid(1.0f);  // 1m unit voxels for 2D grid
}

void loop() {
  // Pick random start and goal
  Coordinate<float> start(0.5f, 0.5f, 0.0f);  // Center of voxel (0,0)
  Coordinate<float> goal(4.5f, 4.5f, 0.0f);   // Center of voxel (4,4)

  // Find path using A*
  AStar<PointCloud> astar;
  auto path = astar.findPath(cloud, start, goal);

  // Check result
  if (path.isEmpty()) {
    Serial.println("=> No path found!");
    return;
  }

  // Print the path
  for (auto& step : path.getWaypoints()) {
    Serial.println(step.toCString());
  }
  Serial.print("Total distance: ");
  Serial.println(path.distance());

  delay(2000);  // Wait before next search
}

// TinyRobotics PointCloud A* Example
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

void printPath(const std::vector<Coordinate<float>>& path) {
  Serial.println("Path:");
  for (const auto& node : path) {
    Serial.print("(");
    Serial.print(node.x);
    Serial.print(", ");
    Serial.print(node.y);
    Serial.print(") ");
  }
  Serial.println();
}

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
  // Build voxel grid for fast occupancy queries
  cloud.buildVoxelGrid(0.5f);  // 50 cm unit voxels for 2D grid
}

void loop() {
  // Pick random start and goal
  Coordinate<float> start(random(0, width), random(0, height));
  Coordinate<float> goal(random(0, width), random(0, height));

  Serial.print("Searching from (");
  Serial.print(start.x);
  Serial.print(",");
  Serial.print(start.y);
  Serial.print(") to (");
  Serial.print(goal.x);
  Serial.print(",");
  Serial.print(goal.y);
  Serial.println(")");

  // Find path using A* (assuming PointCloud has findPathAStar)
  std::vector<Coordinate<float>> path;
  bool found = cloud.findPathAStar(
      start, goal, path);  // This method must exist in your PointCloud

  if (found) {
    Serial.println("Path found!");
    printPath(path);
  } else {
    Serial.println("No path found.");
  }

  delay(2000);  // Wait before next search
}

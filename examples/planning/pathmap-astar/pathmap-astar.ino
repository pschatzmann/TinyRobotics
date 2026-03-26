// TinyRobotics PathMap A* Example (Graph-based)
#include <TinyRobotics.h>

PathMap<Coordinate<float>> map;

// Define some 2D coordinates (nodes)
Coordinate<float> A(0, 0);
Coordinate<float> B(1, 0);
Coordinate<float> C(1, 1);
Coordinate<float> D(0, 1);
Coordinate<float> E(2, 0);
Coordinate<float> F(2, 1);

Coordinate<float> coordinates[] = {A, B, C, D, E, F};

Coordinate<float>* pathSegments[][2] = {
    {&A, &B},  // A-B
    {&B, &C},  // B-C
    {&C, &D},  // C-D
    {&D, &A},  // D-A
    {&B, &E},  // B-E
    {&E, &F},  // E-F
    {&F, &C}   // F-C
};
const size_t numSegments = sizeof(pathSegments) / sizeof(pathSegments[0]);

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
  Serial.println("TinyRobotics PathMap A* Example (Graph-based)");

  // Create a PathMap and add segments (edges) from the global array
  for (size_t i = 0; i < numSegments; ++i) {
    map.addSegment(*pathSegments[i][0], *pathSegments[i][1]);
  }
}

void loop() {
  // Define start and goal
  Coordinate<float> start = coordinates[random(
      0, sizeof(coordinates))];  // Randomly choose A or D as start
  Coordinate<float> goal = coordinates[random(0, sizeof(coordinates))];

  // Find path using A* (assuming PathMap has findPathAStar)
  std::vector<Coordinate<float>> path;
  bool found = map.findPathAStar(
      start, goal, path);  // This method must exist in your PathMap

  if (found) {
    Serial.println("Path found:");
    printPath(path);
  } else {
    Serial.println("No path found.");
  }
}

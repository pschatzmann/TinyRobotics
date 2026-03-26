// TinyRobotics PathMap A* Example (Graph-based)
#include <TinyRobotics.h>
#undef F

PathMap<Coordinate<DistanceM>> pathMap;

// Define some 2D coordinates (nodes)
Coordinate<DistanceM> A(0, 0);
Coordinate<DistanceM> B(1, 0);
Coordinate<DistanceM> C(1, 1);
Coordinate<DistanceM> D(0, 1);
Coordinate<DistanceM> E(2, 0);
Coordinate<DistanceM> F(2, 1);

Coordinate<DistanceM> coordinates[] = {A, B, C, D, E, F};

Coordinate<DistanceM>* pathSegments[][2] = {
    {&A, &B},  // A-B
    {&B, &C},  // B-C
    {&C, &D},  // C-D
    {&D, &A},  // D-A
    {&B, &E},  // B-E
    {&E, &F},  // E-F
    {&F, &C}   // F-C
};
const size_t numSegments = sizeof(pathSegments) / sizeof(pathSegments[0]);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("TinyRobotics PathMap A* Example (Graph-based)");

  // Create a PathMap and add segments (edges) from the global array
  for (size_t i = 0; i < numSegments; ++i) {
    pathMap.addSegment(*pathSegments[i][0], *pathSegments[i][1]);
  }
}

void loop() {
  // Define random start and goal coordinates
  Coordinate<DistanceM> start = coordinates[random(0, sizeof(coordinates))];
  Coordinate<DistanceM> goal = coordinates[random(0, sizeof(coordinates))];

  // Find path using A*
  AStar<PathMap<Coordinate<DistanceM>>> astar;
  auto path = astar.findPath(pathMap, start, goal);

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

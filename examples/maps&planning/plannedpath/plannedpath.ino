/**
 * @file plannedpath.ino
 * @brief Example: GPS path planning using TinyRobotics.
 *
 * Demonstrates how to use the TinyRobotics Path and GPSCoordinate classes to plan
 * and print a path consisting of multiple GPS waypoints. Calculates and displays
 * the total distance of the planned path.
 *
 * - Defines a sequence of GPS waypoints (latitude, longitude).
 * - Uses Path<GPSCoordinate> to represent the route.
 * - Prints each waypoint and the total path distance to Serial.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 *
 * @author Phil Schatzmann
 */
#include <TinyRobotics.h>

// Define some GPS coordinates (latitude, longitude in degrees)
GPSCoordinate start(47.3769, 8.5417);      // Zurich
GPSCoordinate waypoint1(47.3775, 8.5450);  // Near Zurich
GPSCoordinate waypoint2(47.3780, 8.5500);  // Further east
GPSCoordinate goal(47.3800, 8.5600);       // East of Zurich
// Plan: start -> waypoint1 -> waypoint2 -> goal
Path<GPSCoordinate> path(start, waypoint1, waypoint2, goal);

void printPlan() {
  Serial.println("Planned GPS Path:");
  for (size_t i = 0; i < path.size(); ++i) {
    Serial.print("Waypoint ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(path[i].latitude, 6);
    Serial.print(", ");
    Serial.println(path[i].longitude, 6);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("TinyRobotics GPS Path Planning Example");

  printPlan();

  // compute total distance
  float totalDist = 0;
  for (size_t i = 1; i < path.size(); ++i) {
    totalDist += path[i - 1].distance(path[i]);
  }
  Serial.print("Total path distance: ");
  Serial.print(totalDist, 2);
  Serial.println(" meters");
}

void loop() {
  // Nothing to do
}

/**
 * @file record-gps.ino
 * @brief Example: Record GPS data into a path using TinyRobotics.
 *
 * Demonstrates how to use the TinyRobotics Path and GPSCoordinate classes to
 * record GPS waypoints from serial input (NMEA GGA format) into a path. Each new
 * waypoint is added only if it differs from the last recorded one.
 *
 * - Reads GPS data from serial input and parses it.
 * - Adds new waypoints to the path if they are different from the last.
 * - Uses a scheduler to record data at regular intervals.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 *
 * @author Phil Schatzmann
 */

#include "TinyRobotics.h"

// Schedule every 1000 milliseconds (1 second)
uint16_t scheduleIntervalMs = 1000;
Scheduler scheduler;
Path<GPSCoordinate> gpsTimedPath;

void recordGPS(void*) {
  Serial.println("Recording GPS data...");
  NMEAParser gpsParser{GPSFormat::GGA};
  GPSCoordinate gps;
  // Reading GPS data from serial
  auto str = Serial.readStringUntil('\n');
  gpsParser.parse(str, gps);
  Serial.println(gps.toString().c_str());

  // Add to path if different from last waypoint
  auto last = gpsTimedPath.getLastWaypoint();
  if (!last) {
    gpsTimedPath.addWaypoint(gps);
    Serial.println("- Added first entry to path");
  } else if (!last.value().equals(gps, 0.0001)) {
    gpsTimedPath.addWaypoint(gps);
    Serial.println("- Added to path");
  } else {
    Serial.println("- Same as last waypoint");
  }
}

void setup() {
  Serial.begin(9600);
  scheduler.begin(scheduleIntervalMs, recordGPS);
}

void loop() { scheduler.run(); }

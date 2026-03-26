/**
 * @file record-gps.ino
 * @brief This example demonstrates how to use the TinyRobotics library to
 * record GPS data into a path. It reads GPS data from the serial input, parses
 * it, and adds it to a path if it's different from the last recorded waypoint.
 * The GPS data is expected to be in NMEA GGA format.
 * @version 0.1
 * @date 2026-03-23
 *
 * @copyright Copyright (c) 2026
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
  Serial.println(gps.toString());

  // Add to path if different from last waypoint
  auto last = gpsPath.getLastWaypoint();
  if (!last) {
    gpsPath.addWaypoint(gps);
    Serial.println("- Added first entry to path");
  } else if (!last.value().equals(gps, 0.0001)) {
    gpsPath.addWaypoint(gps);
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

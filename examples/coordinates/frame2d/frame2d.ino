/**
 * @file frame2d.ino
 * @brief A simple example demonstrating the use of 2D frames and a range sensor
 * in a robotics context. The example defines a hierarchy of frames (world,
 * base, and lidar) and simulates a range sensor measurement to calculate the
 * obstacle's position in the world frame. This illustrates how to use the
 * FrameMgr2D class to manage coordinate frames and the RangeSensor class to
 * represent sensor data and perform coordinate transformations.
 * In addition we also demonstrate how to convert the vehiclue position to GPS
 * coordinates.
 * @version 0.1
 * @date 2026-03-23
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <TinyRobotics.h>

// Define a simple frame hierarchy: world -> base -> lidar
Frame2D world{FrameType::WORLD, 0};
// vehicule is at (10, 20) facing up (90 degrees)
Frame2D base{FrameType::BASE, 0, world, Transform2D(10, 20, 90)};
// LIDAR is 20cm forward and 10 cm to the right of the base, facing the same
// direction as the base
Frame2D lidar{FrameType::LIDAR, 0, base, Transform2D(0.2, -0.1, 0)};
// Frame mgmt
FrameMgr2D tf;
// lidar sensor
RangeSensor sensor(0);

void setup() {
  Serial.begin(115200);
  // define GPS coordinates for the world frame
  tf.setGPS(world, GPSCoordinate(46.2097, 7.2572, 503));
}

void processLidar() {
  auto tf_lidar2world = tf.getTransform(lidar, world);
  // Set the sensor's transform to world frame
  sensor.setTransform(tf_cam2world);
  // Simulate a random distance measurement between 10 and 50 cm
  sensor.setDistance(random(0.1, 0.5));

  // Get the obstacle coordinate in world coordinates
  Serial.print("Obstacle at (world frame): ");
  Serial.print(sensor.getObstacleCoordinate().toString().c_str());
}

void processGPS() {
  // Convert base coordinates to GPS
  GPSCoordinate base_gps = tf.toGPS(base);
  // Print the result
  Serial.print("Base GPS: ");
  Serial.println(base_gps.toString().c_str());
}

void loop() {
  processLidar();
  processGPS();
}
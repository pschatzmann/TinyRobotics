#/**
# * @file frame3d.ino
# * @brief Example: 3D frame management and GPS conversion using TinyRobotics.
# *
# * Demonstrates how to use the TinyRobotics FrameMgr3D and Frame3D classes to:
# * - Define a hierarchy of 3D frames (world, base, camera)
# * - Compute transforms between frames
# * - Convert vehicle position to GPS coordinates
# *
# * ## Dependencies
# * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
# *
# * @author Phil Schatzmann
# */

#include <TinyRobotics.h>

// Define a simple frame hierarchy: world -> base -> camera
Frame3D world(FrameType::WORLD, 0);
// Vehicle is at (10, 20, 1) facing up (yaw=90 deg, pitch=0, roll=0)
Transform3D base_tf{10, 20, 1, 0, 0, 0.7071f, 0.7071f}; // 90 deg yaw (z)
Frame3D base(FrameType::BASE, 0, &world, base_tf);
// Camera is 20cm forward, 10cm right, 30cm up, facing forward
Transform3D cam_tf{0.2f, -0.1f, 0.3f, 0, 0, 0, 1};
Frame3D camera(FrameType::CAMERA, 0, &base, cam_tf);

FrameMgr3D tf;

void setup() {
  Serial.begin(115200);
  // Set GPS coordinates for the world frame
  tf.setGPS(world, GPSCoordinate(46.2097, 7.2572, 503));

  // Print transform from camera to world
  Transform3D tf_cam2world = tf.getTransform(camera, world);
  Serial.print("Camera->World translation: ");
  Serial.print(tf_cam2world.tx, 3); Serial.print(", ");
  Serial.print(tf_cam2world.ty, 3); Serial.print(", ");
  Serial.println(tf_cam2world.tz, 3);

  // Convert base position to GPS
  std::array<float, 3> base_local = {0, 0, 0};
  GPSCoordinate base_gps = tf.toGPS(base, base_local);
  Serial.print("Base GPS: ");
  Serial.println(base_gps.toString().c_str());
}

void loop() {
  // Nothing to do in loop for this demo
}
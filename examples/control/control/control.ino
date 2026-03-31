/**
 * @file basic.ino
 * @brief Example: Path following with MotionController2D, CarAckerman, Odometry2D, 
 * PathMap, and AStar
 *
 * This example demonstrates:
 *   - Path planning using A* on a 2D map
 *   - Vehicle control with a CarAckerman model
 *   - Real-time pose estimation using Odometry
 *   - Modular scheduling of IMU and controller updates
 *
 * Hardware requirements:
 *   - ESP32 or compatible microcontroller
 *   - Adafruit ICM20948 IMU sensor (I2C)
 *   - CarAckerman-compatible vehicle (steering, drive, encoder pins)
 *
 * Pin configuration (example):
 *   - steer: 5
 *   - dir:   6
 *   - pwm:   9
 *   - encoder: 10
 *
 * Library dependencies:
 *   - TinyRobotics
 *   - AdafruitICM20X (install via Library Manager)
 *
 * Usage:
 *   - Upload to your board
 *   - Open Serial Monitor at 115200 baud
 *   - The vehicle will follow a planned path using real IMU data
 *
 * @author TinyRobotics contributors
 * @date 2026-03-30
 */

#include <Adafruit_ICM20948.h>
#include <TinyRobotics.h>

// Define some 2D coordinates (nodes)
Coordinate<DistanceM> A(0, 0);
Coordinate<DistanceM> B(2, 0);
Coordinate<DistanceM> C(2, 2);
Coordinate<DistanceM> D(0, 2);
Coordinate<DistanceM> E(2, 0);
Coordinate<DistanceM> F(4, 1);
PathMap<Coordinate<DistanceM>> pathMap;
AStar<PathMap<DistanceM>, Coordinate<DistanceM>> astar;
// Define start and goal
Coordinate<DistanceM> start(B);
Coordinate<DistanceM> goal(F);

// Vehicle and control
CarAckerman car(5, 6, 9, 10);  // Example pins: steer, dir, pwm, encoder
Odometry2D odom;
SpeedFromThrottle speedEstimator(2.0f);  // max speed 2 m/s (adjust as needed)
int maxSpeedKmh = 5;
int accelDistanceM = 0.5;
MotionController2D<float> controller(imu, car, maxSpeedKmh, accelDistanceM);
Frame2D world{FrameType::WORLD, 0};
Frame2D base{FrameType::BASE, 0, world, Transform2D(start, 0)};

Scheduler imuScheduler;
Scheduler controllerScheduler;

void buildMap() {
  pathMap.addSegment(A, B);
  pathMap.addSegment(B, C);
  pathMap.addSegment(C, D);
  pathMap.addSegment(D, A);
  pathMap.addSegment(B, E);
  pathMap.addSegment(E, F);
  pathMap.addSegment(F, C);
}


void updateController() { controller.update();
odom.update(speedEstimator.getSpeedMPS(),controller.ge);  // No steering angle for odometry, just speed
}

void setup() {
  Serial.begin(115200);

  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");

  buildMap();
  car.begin();
  controller.begin();

  // find path using A*
  auto path = astar.findPath(pathMap, start, goal);
  if (path.size() > 1) {
    controller.setPath(path);
    controller.begin();
    controllerScheduler.schedule(updateController, 100);
    imuScheduler.schedule(updateIMU, 10);
  } else {
    Serial.println("No path found!");
  }
}

void loop() {
  imuScheduler.run();
  controllerScheduler.run();
}

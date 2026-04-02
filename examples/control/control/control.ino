/**
 * @file basic.ino
 * @brief Example: Path following with MotionController2D, CarAckerman,
 * Odometry2D, PathMap, and AStar
 *
 * This example demonstrates:
 *   - Path planning using A* on a 2D map
 *   - Vehicle control with a CarAckerman model
 *   - Real-time pose estimation using Odometry
 *   - Modular scheduling of Odometry and controller updates
 *
 * Hardware requirements:
 *   - ESP32 or compatible microcontroller
 *   - CarAckerman-compatible vehicle (steering, drive, encoder pins)
 *
 * Pin configuration (example):
 *   - steer: 5
 *   - dir:   6
 *   - pwm:   9
 *   - encoder: 10
 *
 * Usage:
 *   - Upload to your board
 *   - Open Serial Monitor at 115200 baud
 *   - The vehicle will follow a planned path using odometry data
 *
 * @date 2026-03-30
 */

#include <TinyRobotics.h>
#undef F  // avoid conflicts with Macros

// Define some 2D coordinates (nodes)
Coordinate<float> A(0, 0);
Coordinate<float> B(2, 0);
Coordinate<float> C(2, 2);
Coordinate<float> D(0, 2);
Coordinate<float> E(2, 0);
Coordinate<float> F(4, 1);
PathMap<Coordinate<float>> pathMap;
// Define start and goal
Coordinate<float> start(B);
Coordinate<float> goal(F);
Frame2D world{FrameType::WORLD, 0};
Frame2D base{FrameType::BASE, 0, world, Transform2D(start, 0)};

AStar<PathMap<Coordinate<float>>, Coordinate<float>> astar;
CarAckerman<BrushedMotor, ServoMotor> car;
Odometry2D odometry;
SpeedFromThrottle speedEstimator(2.0f);  // max speed 2 m/s (adjust as needed)
int maxSpeedKmh = 5;
float accelDistanceM = 0.5; // distance to start decelerating in meters
float wheelBase = 0.3f;  // distance between front and rear axles in meters
MotionController2D<float> controller(odometry, maxSpeedKmh, accelDistanceM);

Scheduler scheduler;

void buildMap() {
  pathMap.addSegment(A, B);
  pathMap.addSegment(B, C);
  pathMap.addSegment(C, D);
  pathMap.addSegment(D, A);
  pathMap.addSegment(B, E);
  pathMap.addSegment(E, F);
  pathMap.addSegment(F, C);
}

void updateController(void*) {
  // Move to next waypoint
  controller.update();
  // estimate speed from throttle
  float speed = speedEstimator.getSpeedMPS(controller.getThrottlePercent());
  // update odometry with estimated speed and current steering angle
  odometry.update(Speed(speed, SpeedUnit::MPS), controller.getSteeringAngle());
}

void setup() {
  Serial.begin(115200);

  buildMap();

  // find path using A*
  auto path = astar.findPath(pathMap, start, goal);
  if (path.size() > 1) {
    controller.subscribe(car);  // subscribe to control messages from the controller
    controller.setPath(path);
    controller.begin();
    odometry.begin(base, Distance(wheelBase, DistanceUnit::M));

    // update every 100ms (adjust as needed)
    scheduler.begin(100, updateController);
  } else {
    Serial.println("No path found!");
  }
}

void loop() { scheduler.run(); }

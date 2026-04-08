/**
 * @file control.ino
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

AStar astar;

CarAckerman car;
Distance wheelBase(0.3f, DistanceUnit::M);
OdometryHeadingModel odomModel(wheelBase);
SpeedFromThrottle speedEstimator(Speed(5, SpeedUnit::KPH));
Odometry2D odometry(car, speedEstimator, odomModel);
Speed maxSpeedKmh(5, SpeedUnit::KPH);  // max speed in km/h
Distance accelDistanceM(0.5, DistanceUnit::M); // distance to start decelerating in meters
Angle maxSteeringAngle(30.0f, AngleUnit::DEG);
MotionController2D<float> controller(odometry, maxSpeedKmh, maxSteeringAngle, accelDistanceM);

Scheduler scheduler;
MessageHandlerPrintJSON json_printer(Serial);  // Print to Serial in JSON format

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
  if (controller.isGoalReached()) return;  // stop updating if goal is reached
  // Move to next waypoint
  controller.update();
  // update odometry (uses speed source and steering angle internally)
  odometry.update();
}

void setup() {
  Serial.begin(115200);
  TRLogger.begin(LoggerClass::INFO, Serial);  // Initialize logger with Serial output and INFO level

  buildMap();
  car.setPins(4, 5, 6);  // int in1, int in2, int steeringPin

  // find path using A*
  auto path = astar.findPath(pathMap, start, goal);
  if (path.size() > 1) {
    // setup odometry firs
    odometry.subscribe(json_printer);  // subscribe to odometry messages for telemetry
    odometry.begin(base);
    // then setup controller which depends on odometry
    controller.subscribe(car);  // subscribe to control messages from the controller
    controller.setPath(path);
    controller.begin();
    controller.subscribe(json_printer);  // subscribe to controller messages for telemetry
    car.subscribe(json_printer);  // subscribe to car messages for telemetry

    // update every 100ms (adjust as needed)
    scheduler.begin(200, updateController);
  } else {
    Serial.println("No path found!");
  }
}

void loop() { scheduler.run(); }

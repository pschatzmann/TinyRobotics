/**
 * @file contol-simple.ino
 * @brief Example: Simple path following with MotionController2D, CarAckerman,
 * and Odometry2D
 *
 * This example demonstrates:
 *   - Vehicle control with a CarAckerman model
 *   - Real-time pose estimation using Odometry2D
 *   - Path following using MotionController2D
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
 *   - The vehicle will follow a simple path using odometry data
 *
 * @date 2026-03-30
 */

#include <TinyRobotics.h>

Frame2D world{FrameType::WORLD, 0};
Frame2D base{FrameType::BASE, 0, world, Transform2D(0, 0, 90)};
Coordinate<float> target(10, 0);


CarAckerman car;
Distance wheelBase(0.3f, DistanceUnit::M);
OdometryModel2D odomModel(wheelBase);
SpeedFromThrottle speedEstimator(Speed(5, SpeedUnit::KPH));
Odometry2D odometry(car, speedEstimator, odomModel);
Speed maxSpeedKmh(5, SpeedUnit::KPH);  // max speed in km/h
Distance accelDistanceM(0.5, DistanceUnit::M);
Angle maxSteeringAngle(30.0f, AngleUnit::DEG);
MotionController2D<float> controller(odometry, maxSpeedKmh, maxSteeringAngle, accelDistanceM);

Scheduler scheduler;
MessageHandlerPrintJSON json_printer(Serial);  // Print to Serial in JSON format

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

  // subscribe to odometry messages for telemetry
  odometry.subscribe(json_printer);
  odometry.begin(base);

  // then setup controller which depends on odometry
  controller.subscribe(
      car);  // subscribe to control messages from the controller
  controller.addWaypoint(target);
  controller.setTargetAccuracy(0.10f);  // 10 cm accuracy
  controller.begin();
  controller.subscribe(
      json_printer);  // subscribe to controller messages for telemetry

  car.setPins(4, 5, 6);      // int in1, int in2, int steeringPin
  car.subscribe(json_printer);  // subscribe to car messages for telemetry
  car.begin();

  // update every 100ms (adjust as needed)
  scheduler.begin(100, updateController);
}

void loop() { scheduler.run(); }

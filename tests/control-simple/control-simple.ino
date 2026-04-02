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

Frame2D world{FrameType::WORLD, 0};
Frame2D base{FrameType::BASE, 0, world, Transform2D(0, 0, 180)};
Coordinate<float> target(10, 0);

CarAckerman<BrushedMotor, ServoMotor> car;
Odometry2D odometry;
Speed maxSpeedKmh(5, SpeedUnit::KPH);    // max speed in km/h
SpeedFromThrottle speedEstimator(maxSpeedKmh);  // max speed 2 m/s (adjust as needed)
Distance accelDistanceM(
    0.5, DistanceUnit::M);  // distance to start decelerating in meters
Distance wheelBase(
    0.3f, DistanceUnit::M);  // distance between front and rear axles in meters
MotionController2D<float> controller(odometry, maxSpeedKmh, accelDistanceM);

Scheduler scheduler;
MessageHandlerPrintJSON json_printer(
    NullPrint);  // Print to Serial in JSON format

void updateController(void*) {
  if (controller.isGoalReached()) return;  // stop updating if goal is reached
  // Move to next waypoint
  controller.update();
  // estimate speed from throttle
  float speed = speedEstimator.getSpeedMPS(controller.getThrottlePercent());
  // update odometry with estimated speed and current steering angle
  odometry.update(Speed(speed, SpeedUnit::MPS), controller.getSteeringAngle());
}

void setup() {
  Serial.begin(115200);

  // find path using A*
  // setup odometry firs
  odometry.begin(base, wheelBase);
  odometry.subscribe( json_printer);  // subscribe to odometry messages for telemetry
  // then setup controller which depends on odometry
  controller.subscribe(car);  // subscribe to control messages from the controller
  controller.addWaypoint(target);
  controller.setTargetAccuracy(0.10f);  // 10 cm accuracy
  controller.begin();
  controller.subscribe(
      json_printer);  // subscribe to controller messages for telemetry
  car.subscribe(json_printer);  // subscribe to car messages for telemetry

  // update every 100ms (adjust as needed)
  scheduler.begin(200, updateController);
}

void loop() { scheduler.run(); }

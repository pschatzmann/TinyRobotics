/**
 * @file basic.ino
 * @brief Example: Car with ackerman steering following a circular path using
 * Odometry2D
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

CarAckerman car;
OdometryHeadingModel headingModel(Distance(0.3f, DistanceUnit::M));  // wheelbase of 0.3m
SpeedFromThrottle speedEstimator(2.0f);  // max speed 2 m/s (adjust as needed)
Odometry2D odometry(car, speedEstimator, headingModel);
Speed maxSpeedKmh(5, SpeedUnit::KPH);    // max speed in km/h
Distance wheelBase(
    0.3f, DistanceUnit::M);  // distance between front and rear axles in meters
Frame2D world{FrameType::WORLD, 0};
Frame2D base{FrameType::BASE, 0, world, Transform2D(3, 3, 0)};

Scheduler scheduler;
MessageHandlerPrintJSON json_printer(Serial);  // Print to Serial in JSON format

void update(void*) {
  // update odometry (uses speed source and steering angle internally)
  odometry.update();
}

void setup() {
  Serial.begin(115200);

  car.setPins(4, 5, 6);  // int in1, int in2, int steeringPin
  odometry.subscribe(json_printer);
  odometry.begin(base);
  car.subscribe(json_printer);
  car.setSpeed(50);
  car.setSteeringAngle(30);  // 30 degrees steering angle for circular path

  scheduler.begin(100, update);  // Update every 50 ms
}

void loop() { scheduler.run(); }

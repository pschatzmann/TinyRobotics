/**
 * @file micro-ros.ino
 * @brief Example: micro-ROS communication with TinyRobotics
 *
 * This example demonstrates how to use the TinyRobotics MicroROSS class to
 * connect an ESP32 (or compatible) to a ROS2 system using micro-ROS. It
 * subscribes to velocity commands and publishes odometry.
 *
 * Features:
 *   - micro-ROS node setup for /cmd_vel and /odom
 *   - Handles Twist messages and publishes Odometry
 *   - Designed for Ackermann or differential drive robots
 *
 * Hardware requirements:
 *   - ESP32 or compatible microcontroller
 *   - micro-ROS compatible firmware and libraries
 *
 * Library dependencies:
 *   - TinyRobotics
 *   - micro_ros_arduino
 *
 * Usage:
 *   - Upload to your board
 *   - Connect to ROS2 system via micro-ROS agent
 *   - Send /cmd_vel messages and receive /odom
 *
 * @author TinyRobotics contributors
 * @date 2026-03-30
 */
#include "TinyRobotics.h"
#include "TinyRobotics/communication/MicroROS.h"


MicroROS ros;
CarAckerman car;
MessageHandlerPrint printer(Serial);
Scheduler scheduler;

// Odometry and speed estimation
SpeedFromThrottle speedEstimator(2.0f);  // max speed 2 m/s (adjust as needed)
const float wheelbase = 0.25;  // meters
OdometryModel2D odomModel(Distance(wheelbase, DistanceUnit::M));
Odometry2D odom(car, speedEstimator, odomModel);

void sendOdometryToROS(void* ref) {
  rclc_executor_spin_some(&ros.getExecutor(), RCL_MS_TO_NS(10));
  nav_msgs__msg__Odometry odom_msg;

  // Use Odometry2D state
  auto pos = odom.getPosition();
  float theta = odom.getTheta();
  float v = odom.getLinearVelocity();
  float omega = odom.getAngularVelocity();

  odom_msg.pose.pose.position.x = pos.x;
  odom_msg.pose.pose.position.y = pos.y;
  // Optionally set orientation (not shown)
  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.angular.z = omega;

  ros.sendOdometry(odom_msg);
}

// ROS callback for /cmd_vel
void onDataFromROS(const void* msgin) {
  const geometry_msgs__msg__Twist* cmd_msg =
      static_cast<const geometry_msgs__msg__Twist*>(msgin);
  float linear_x = cmd_msg->linear.x;
  float angular_z = cmd_msg->angular.z;
  TRLogger.info("Received cmd_vel: linear_x=%.2f, angular_z=%.2f", linear_x,
                angular_z);

  // Convert Twist to Ackermann steering
  float current_steering = 0.0;
  if (fabs(linear_x) > 0.001) {
    current_steering = atan(wheelbase * angular_z / linear_x);
  }
  // Convert linear_x to throttle (simple proportional, adjust as needed)
  float current_throttle = linear_x * 100.0f;  // -100 to 100

  // Set car actuators
  car.setSpeed((int)current_throttle);
  car.setSteeringAngle((int)(current_steering / M_PI * 180));

  // Update odometry using estimated speed
  odom.update();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics Messaging Example");

  // setup ROS using UDP
  ros.setTransport("your_ssid", "your_password", "192.168.1.39", 8888);
  ros.setCallback(onDataFromROS);
  ros.begin();

  // Log received commands
  car.subscribe(printer);

  // Initialize odometry
  odom.begin(Coordinate<DistanceM>(0, 0), 0.0f,
             Distance(wheelbase, DistanceUnit::M));

  // publish odometry
  scheduler.begin(10, sendOdometryToROS);
}

void loop() { scheduler.run(); }
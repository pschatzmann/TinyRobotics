/**
 * @file micro-ros.ino
 * @brief Example: micro-ROS communication with TinyRobotics
 *
 * This example demonstrates how to use the TinyRobotics MicroROSS class to connect an ESP32 (or compatible)
 * to a ROS2 system using micro-ROS. It subscribes to velocity commands and publishes odometry.
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
MessageHandlerPrint printer(Serial);  // forward to Serial
Scheduler scheduler;
float current_speed = 0.0;
float current_steering = 0.0;
const wheelbase = 0.25;  // meters

void sendOdometry(void* ref) {
  rclc_executor_spin_some(&ros.getExecutor(), RCL_MS_TO_NS(10));
  nav_msgs__msg__Odometry odom_msg;
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0;
  last_time = now;

  // ===== Kinematics =====
  float v = current_speed;
  float delta = current_steering;

  float omega = 0.0;
  if (fabs(delta) > 0.001) {
    omega = v * tan(delta) / wheelbase;
  }

  theta += omega * dt;
  pos_x += v * cos(theta) * dt;
  pos_y += v * sin(theta) * dt;

  // ===== Odometry message =====
  odom_msg.pose.pose.position.x = pos_x;
  odom_msg.pose.pose.position.y = pos_y;

  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.angular.z = omega;

  ros.sendOdometry(odom_msg);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics Messaging Example");

  // setup ROS using UDP
  ros.setTransportWiFiUDP("your_ssid", "your_password", "192.168.1.39", 8888);
  ros.setCallback([](const void* msgin) {
    const geometry_msgs__msg__Twist* cmd_msg =
        static_cast<const geometry_msgs__msg__Twist*>(msgin);
    // For simplicity, we only use linear.x and angular.z from the Twist message
    float linear_x = cmd_msg->linear.x;
    float angular_z = cmd_msg->angular.z;
    TRLogger.info("Received cmd_vel: linear_x=%.2f, angular_z=%.2f", linear_x,
                  angular_z);

    float v = linear_x;
    float omega = angular_z;

    // Convert Twist → Ackermann steering
    current_steering = 0.0;
    if (fabs(v) > 0.001) {
      current_steering = atan(wheelbase * omega / v);
    }

    // Convert to car control commands (simple example)
    int speed = (int)(linear_x * 100);  // Scale to PWM range
    int steer = (int)(current_steering / M_PI * 180);  // Scale to steering angle in degrees
    car.setSpeed(speed);
    car.setSteeringAngle(steer);
  });
  ros.begin();

  // Log received commands
  car.subscribe(printer);

  // publish odometry
  scheduler.schedule(sendOdometry, 10);
}

void loop() { scheduler.run(); }
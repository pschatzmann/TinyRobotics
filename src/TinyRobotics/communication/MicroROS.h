#pragma once
#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <math.h>

#include "TinyRobotics/utils/LoggerClass.h"

namespace tinyrobotics {

/**
 * @class MicroROS
 * @brief micro-ROS interface for vehicle control and odometry on
 * Arduino/ESP32.
 *
 * This class sets up a micro-ROS node e.g. for an Ackermann steering robot,
 * subscribing to velocity commands (geometry_msgs/Twist on /cmd_vel) and
 * publishing odometry (nav_msgs/Odometry on /odom).
 *
 * Features:
 *   - Receives velocity commands e.g. to converts them to Ackermann steering
 * and speed.
 *   - Publishes odometry
 * received.
 *   - Uses micro-ROS Arduino library for communication.
 *   - User provides a callback for handling incoming Twist messages.
 *
 * Usage:
 *   1. Call setCallback() to provide a function for handling incoming /cmd_vel
 * messages.
 *   2. Call begin() to initialize micro-ROS and set up publishers/subscribers.
 *   3. Periodically call sendOdometry() in your main loop to process messages
 * and publish odometry.
 *
 * @note optional functionality: include "TinyRobotics/communication/MicroROS.h"
 * @author Phil Schatzmann
 * @date 2026-03-30
 */

class MicroROS {
 public:
  MicroROS(const char* node_name = "tinyrobotics_node") {
    ros_node_name = node_name;
  }

  void setCallback(void (*cb)(const void* msgin)) { cmd_callback = cb; }

  //   void setTransport(Print&out) {
  //     set_microros_serial_transports(Serial);
  //     is_tranport_defined = true;
  //   }

  void setTransport(char* ssid, char* pwd, char* ip, int port = 8888) {
    set_microros_wifi_transports(ssid, pwd, ip, port);
    is_tranport_defined = true;
  }

  bool begin() {
    if (cmd_callback == nullptr) {
      TRLogger.error("MicroROSS: Callback function not set!");
      return false;
    }

    if (!is_tranport_defined) set_microros_transports();

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, ros_node_name, "", &support);

    // Subscriber: /cmd_vel
    rclc_subscription_init_default(
        &sub_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    // Publisher: /odom
    rclc_publisher_init_default(
        &pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom");

    // Executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &sub_cmd, &cmd_msg, cmd_callback,
                                   ON_NEW_DATA);

    return true;
  }

  /// Publish odometry message
  bool sendOdometry(const nav_msgs__msg__Odometry& msg) {
    auto rc = rcl_publish(&pub_odom, &msg, NULL);
    return rc == RCL_RET_OK;
  }

  rclc_executor_t& getExecutor() { return executor; }

 protected:
  const char* ros_node_name = nullptr;
  void (*cmd_callback)(const void* msgin) = nullptr;
  bool is_tranport_defined = false;

  rcl_node_t node;
  rclc_executor_t executor;
  rclc_support_t support;
  rcl_allocator_t allocator;

  rcl_subscription_t sub_cmd;
  rcl_publisher_t pub_odom;
  geometry_msgs__msg__Twist cmd_msg;
};

}  // namespace tinyrobotics
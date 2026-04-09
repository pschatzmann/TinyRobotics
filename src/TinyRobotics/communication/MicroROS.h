#pragma once
#include <math.h>
#include <micro_ros_arduino.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/point32.h>
#include <geometry_msgs/msg/twist.h>

#include "TinyRobotics/communication/MessageHandler.h"
#include "TinyRobotics/maps/IMap.h"
#include "TinyRobotics/utils/AllocatorPSRAM.h"
#include "TinyRobotics/utils/LoggerClass.h"

namespace tinyrobotics {

/**
 * @class MicroROS
 * @ingroup communication
 * @brief micro-ROS interface for vehicle control and odometry on
 * Arduino/ESP32.
 *
 * @note Install the micro_ros_arduino library
 *
 * This class sets up a micro-ROS node e.g. for an Ackermann steering robot,
 * subscribing to velocity commands (geometry_msgs/Twist on /cmd_vel) and
 * publishing odometry (nav_msgs/Odometry on /odom).
 *
 * Features:
 *   - Receives velocity commands e.g. to converts them to Ackermann steering
 * and speed.
 *   - Publishes odometry
 *   - Publishes occupancy grid maps from IMap interface
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

class MicroROS : public MessageHandler {
 public:
  MicroROS(const char* node_name = "tinyrobotics_node") {
    ros_node_name = node_name;
  }

  void setCallback(void (*cb)(const void* msgin)) { cmd_callback = cb; }

  // void setTransport(Print&out) {
  //   set_microros_serial_transports(Serial);
  //   is_tranport_defined = true;
  // }

#ifdef ESP32
  void setTransport(char* ssid, char* pwd, char* ip, int port = 8888) {
    set_microros_wifi_transports(ssid, pwd, ip, port);
    is_tranport_defined = true;
  }
#endif

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
        &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom");

    // Executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &sub_cmd, &cmd_msg, cmd_callback,
                                   ON_NEW_DATA);

    return true;
  }

  /// Publish odometry message
  bool sendOdometry(const nav_msgs__msg__Odometry& msg) {
    auto rc = rcl_publish(&publisher, &msg, NULL);
    return rc == RCL_RET_OK;
  }

  /// Publish odometry message
  bool sendOdometry(const MotionState3D state) {
    nav_msgs__msg__Odometry msg;
    // Zero/init all fields
    memset(&msg, 0, sizeof(msg));

    // Position
    msg.pose.pose.position.x = state.getPosition().x;
    msg.pose.pose.position.y = state.getPosition().y;
    msg.pose.pose.position.z = state.getPosition().z;

    // Orientation (convert yaw/pitch/roll to quaternion)
    float cy = cosf(state.getOrientation().yaw * 0.5f);
    float sy = sinf(state.getOrientation().yaw * 0.5f);
    float cp = cosf(state.getOrientation().pitch * 0.5f);
    float sp = sinf(state.getOrientation().pitch * 0.5f);
    float cr = cosf(state.getOrientation().roll * 0.5f);
    float sr = sinf(state.getOrientation().roll * 0.5f);
    msg.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    msg.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    msg.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    msg.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;

    // Linear velocity
    msg.twist.twist.linear.x = state.getSpeed().x;
    msg.twist.twist.linear.y = state.getSpeed().y;
    msg.twist.twist.linear.z = state.getSpeed().z;

    // Angular velocity
    msg.twist.twist.angular.x = state.getAngularVelocity().x;
    msg.twist.twist.angular.y = state.getAngularVelocity().y;
    msg.twist.twist.angular.z = state.getAngularVelocity().z;

    // Optionally: set frame_id, child_frame_id, covariance, etc.
    // strncpy(msg.header.frame_id, "odom", sizeof(msg.header.frame_id));
    // strncpy(msg.child_frame_id, "base_link", sizeof(msg.child_frame_id));

    return sendOdometry(msg);
  }

  /// Publish odometry message
  bool sendOdometry(Coordinate<DistanceM> position, Orientation3D orientation) {
    MotionState3D state(
        position, orientation, Speed3D(0, 0, 0, SpeedUnit::MPS),
        AngularVelocity3D(0, 0, 0, AngularVelocityUnit::RadPerSec));
    return sendOdometry(state);
  }

  /// Publish Point32 (occupancy grid map message not available in micro-ROS)
  bool sendPoint(const geometry_msgs__msg__Point32& msg) {
    auto rc = rcl_publish(&publisher, &msg, NULL);
    return rc == RCL_RET_OK;
  }

  bool sendCoordinate(const Coordinate<DistanceM>& coordinate) {
    geometry_msgs__msg__Point32 msg;
    msg.x = coordinate.x;
    msg.y = coordinate.y;
    msg.z = coordinate.z;
    return sendPoint(msg);
  }

  /// Publish Point32 (occupancy grid map message not available in micro-ROS)
  template <typename T>
  bool sendMap(IMap<T>& entry) {
    Coordinate<DistanceM> coordinate;
    coordinate.x = entry.getCoordinate().x;
    coordinate.y = entry.getCoordinate().y;
    coordinate.z =
        entry.getValue();  // Assuming getValue() returns occupancy value
    return sendCoordinate(coordinate);
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
  rcl_publisher_t publisher;
  geometry_msgs__msg__Twist cmd_msg;

  bool onMessage(const Message<MotionState3D>& msg) override {
    sendOdometry(msg.value);
    return true;
  }

  bool onMessage(const Message<Coordinate<float>>& msg) {
    sendCoordinate(msg.value);
    return true;
    };

};

}  // namespace tinyrobotics
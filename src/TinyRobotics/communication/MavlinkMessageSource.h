#pragma once
#include <SimpleMavlinkDrone.h>  // https://github.com/pschatzmann/ArduinoMavlinkDrone
#include "MessageSource.h"

namespace tinyrobotics {

enum class ControlScenario { Car, Boat, Drone, Plane };

/**
 * @class MavlinkMessageSource
 * @ingroup communication
 * @brief Message source for MAVLink integration using ArduinoMavlinkDrone.
 *
 * @note This class depends on the ArduinoMavlinkDrone library:
 *       https://github.com/pschatzmann/ArduinoMavlinkDrone
 *
 * This class connects to a MAVLink drone and translates MAVLink telemetry and
 * control messages into TinyRobotics Message objects. It can be used to bridge
 * MAVLink-based vehicles with the TinyRobotics message bus and control
 * framework.
 *
 * Usage:
 *   MavlinkMessageSource mavlink(drone);
 *   mavlink.begin();
 *   // In loop: mavlink.update();
 *
 */
class MavlinkMessageSource : public MessageSource {
public:
  MavlinkMessageSource(SimpleMavlinkDrone& drone,
                       ControlScenario scenario = ControlScenario::Car)
      : drone(drone), scenario(scenario) {}

  /// Start the message source

  bool begin(ControlScenario scenario = ControlScenario::Car) {
    this->scenario = scenario;
    is_active_ = true;
    return true;
  }

  /// Stop the message source
  void end() { is_active_ = false; }

  /// Call this in your main loop to handle MAVLink messages
  void update() {
    if (!is_active_) return;
    drone.loop();
    publishScenario();
  }

  /// @brief  Defines the max steering ange in degrees: default 45 
  /// @param angleDeg 
  void setMaxSteeringAngle(float angleDeg) {
    max_steering_deg = angleDeg;
  }

 protected:
  SimpleMavlinkDrone& drone;
  ControlScenario scenario;
  bool is_active_ = false;
  float max_steering_deg = 45.0f;

  void publishScenario() {
    switch (scenario) {
      case ControlScenario::Car:
        publishCar();
        break;
      case ControlScenario::Boat:
        publishBoat();
        break;
      case ControlScenario::Drone:
        publishDrone();
        break;
      case ControlScenario::Plane:
        publishPlane();
        break;
    }
  }

  void publishCar() {
    // Car: SteeringAngle (YAW), Throttle
    float steeringAngle = -drone.getValue(YAW) * max_steering_deg / 100.0f;
    steeringAngle = normalizeAngleDeg(steeringAngle);
    float throttle = drone.getValue(THROTTLE) * 100.0f;  // 0..1 to percent
    publish(Message(MessageContent::SteeringAngle, steeringAngle, Unit::AngleDegree));
    publish(Message(MessageContent::Throttle, throttle, Unit::Percent));
  }

  void publishBoat() {
    // Boat: SteeringAngle (YAW), Throttle
    float steeringAngle = -drone.getValue(YAW) * max_steering_deg / 100.0f;
    steeringAngle = normalizeAngleDeg(steeringAngle);
    float throttle = drone.getValue(THROTTLE) * 100.0f;  // 0..1 to percent
    publish(Message(MessageContent::SteeringAngle, steeringAngle, Unit::AngleDegree));
    publish(Message(MessageContent::Throttle, throttle, Unit::Percent));
  }

  void publishDrone() {
    // Drone: Roll, Pitch, Throttle, Yaw
    float roll = drone.getValue(ROLL);
    float pitch = drone.getValue(PITCH);
    float throttle = drone.getValue(THROTTLE) * 100.0f;
    float yaw = -drone.getValue(YAW);  // Convert to ROS: positive is right
    publish(Message(MessageContent::Roll, roll, Unit::Percent));
    publish(Message(MessageContent::Pitch, pitch, Unit::Percent));
    publish(Message(MessageContent::Throttle, throttle, Unit::Percent));
    publish(Message(MessageContent::Yaw, yaw, Unit::Percent));
  }

  void publishPlane() {
    // Plane: Roll (Aileron), Pitch (Elevator), Throttle, Yaw (Rudder)
    float aileron = drone.getValue(ROLL);
    float elevator = drone.getValue(PITCH);
    float throttle = drone.getValue(THROTTLE) * 100.0f;
    float rudder = -drone.getValue(YAW);  // Convert to ROS: positive is right
    publish(Message(MessageContent::Roll, aileron, Unit::Percent));
    publish(Message(MessageContent::Pitch, elevator, Unit::Percent));
    publish(Message(MessageContent::Throttle, throttle, Unit::Percent));
    publish(Message(MessageContent::Yaw, rudder, Unit::Percent));
  }

  // Overload for publishing Message<float>
  void publish(const Message<float>& msg) {
    Message<float> m = msg;
    sendMessage(m);
  }

  // Helper for float mapping (Arduino's map only works for long)
  static float mapFloat(float x, float in_min, float in_max, float out_min,
                        float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
};

}  // namespace tinyrobotics

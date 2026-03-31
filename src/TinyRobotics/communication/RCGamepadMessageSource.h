#pragma once
#include "GamepadServer.h"  // install https://github.com/pschatzmann/VirtualGamePadArduino
#include "MessageSource.h"

namespace tinyrobotics {

/**
 * @brief Control scenario types for remote control vehicles.
 *
 * Used to select the control mapping for different vehicle types.
 */

enum class ControlScenario { Car, Boat, Drone, Plane };

/**
 * @class RCGamepadMessageSource
 * @ingroup communication
 * @brief Message source for remote control via a virtual gamepad.
 *
 * @note This class depends on the VirtualGamePadArduino library:
 *       https://github.com/pschatzmann/VirtualGamePadArduino
 *
 * @note this is an optional class, so it must be included in your sketch with:
 * #include "TinyRobotics/communication/RCGamepadMessageSource.h"
 *
 * This class connects to a GamepadServer and translates gamepad input into
 * TinyRobotics Message objects for remote control scenarios (Car, Boat, etc).
 * It supports different navigation scenarios and publishes control messages
 * (steering, throttle, turn, etc) based on the selected scenario.
 *
 * Usage:
 *   RCGamepadMessageSource source(server);
 *   source.begin(NaviationScenario::Car);
 *   // In loop: source.update();
 *
 * ## Callback Mechanism
 *
 * You can register a custom callback using setCallback(). The callback is
 * invoked whenever a new GamepadState is received. If the callback returns
 * false, the default message publishing is skipped, allowing you to publish
 * custom messages instead. If the callback returns true (or is not set), the
 * default messages for the selected scenario are published.
 *
 * Example:
 *   source.setCallback([](const GamepadState& state, MessageSource& src) {
 *     // Custom message publishing logic
 *     return false; // skip default publishing
 *   });
 *
 * The class is designed for integration with the TinyRobotics message bus and
 * remote control framework.
 */
class RCGamepadMessageSource : public MessageSource {
 public:
  RCGamepadMessageSource(NetworkServer& server) : gamepad(server) {}

  /// Start the message source and connect to the gamepad server
  bool begin(ControlScenario scenario = ControlScenario::Car) {
    this->scenario = scenario;
    gamepad.begin();
    is_active_ = true;
    return true;
  }

  /// Stop the message source and disconnect from the gamepad server
  void end() { is_active_ = false; }

  /// Call this in your main loop to handle incoming gamepad messages and
  /// publish control messages
  void update() {
    if (is_active_) {
      gamepad.handleClient();
      // Poll for new state and publish if changed
      GamepadState state = gamepad.getState();
      if (callback == nullptr || callback(state, *this)) {
        publish(state);
      }
    }
  }

  /// Add a callback to receive and publish additional messages
  void setCallback(bool (*cb)(const GamepadState&, MessageSource& source)) {
    callback = cb;
  }

 private:
  GamepadServer gamepad;
  ControlScenario scenario;
  bool (*callback)(const GamepadState&, MessageSource& source) = nullptr;
  bool is_active_ = false;

  void publish(const GamepadState& state) {
    switch (scenario) {
      case ControlScenario::Car:
        publishCar(state);
        break;
      case ControlScenario::Boat:
        publishBoat(state);
        break;
      case ControlScenario::Drone:
        publishDrone(state);
        break;
      case ControlScenario::Plane:
        publishPlane(state);
        break;
    }
  }

  // Overload for publishing Message<float>
  void publish(const Message<float>& msg) {
    // Use sendMessage to forward to all handlers
    // Make a non-const copy as sendMessage expects non-const
    Message<float> m = msg;
    sendMessage(m);
  }

  void publishCar(const GamepadState& state) {
    float steeringAngleDegree = state.left_thumbstick.angleDegROS();
    float throttlePercent = 100.0f * state.left_thumbstick.magnitude();
    float turnPercent = steeringAngleDegree / 180.0f * 100.0f;
    publish(Message(MessageContent::SteeringAngle, steeringAngleDegree,
                    Unit::AngleDegree));
    publish(Message(MessageContent::Throttle, throttlePercent, Unit::Percent));
    publish(Message(MessageContent::Turn, turnPercent, Unit::Percent));
  }

  void publishDrone(const GamepadState& state) {
    float rollAngleDegree = state.left_thumbstick.angleDegROS();
    float pitchAngleDegree = state.right_thumbstick.angleDegROS();
    float throttlePercent = 100.0f * state.left_thumbstick.magnitude();
    float yawTurnPercent = 100.0f * state.right_thumbstick.magnitude();
    publish(Message(MessageContent::Roll, rollAngleDegree, Unit::AngleDegree));
    publish(
        Message(MessageContent::Pitch, pitchAngleDegree, Unit::AngleDegree));
    publish(Message(MessageContent::Throttle, throttlePercent, Unit::Percent));
    publish(Message(MessageContent::Yaw, yawTurnPercent, Unit::Percent));
  }

  void publishPlane(const GamepadState& state) {
    float aileronAngleDegree = state.left_thumbstick.angleDegROS();
    float elevatorAngleDegree = state.right_thumbstick.angleDegROS();
    float throttlePercent = 100.0f * state.left_thumbstick.magnitude();
    float rudderTurnPercent = 100.0f * state.right_thumbstick.magnitude();
    publish(Message(MessageContent::Roll, aileronAngleDegree,
                    Unit::AngleDegree));  // Aileron
    publish(Message(MessageContent::Pitch, elevatorAngleDegree,
                    Unit::AngleDegree));  // Elevator
    publish(Message(MessageContent::Throttle, throttlePercent,
                    Unit::Percent));  // Throttle
    publish(Message(MessageContent::Yaw, rudderTurnPercent,
                    Unit::Percent));  // Rudder
  }

  void publishBoat(const GamepadState& state) {
    float steeringAngleDegree = state.left_thumbstick.angleDegROS();
    float throttlePercent = 100.0f * state.left_thumbstick.magnitude();
    float turnPercent = steeringAngleDegree / 180.0f * 100.0f;

    publish(Message(MessageContent::SteeringAngle, steeringAngleDegree,
                    Unit::AngleDegree));
    publish(Message(MessageContent::Throttle, throttlePercent, Unit::Percent));
  }

  // No static callback needed; polling is used in update()
};

}  // namespace tinyrobotics
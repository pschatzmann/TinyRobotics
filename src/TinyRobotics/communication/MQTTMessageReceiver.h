#pragma once
#include <PubSubClient.h>

#include <string>

#include "MessageHandler.h"

namespace tinyrobotics {

/**
 * @class MQTTMessageReceiverTask
 * @ingroup communication
 * @brief Task for receiving MQTT messages and dispatching them to a
 * MessageHandler.
 *
 * This class subscribes to MQTT topics and, upon receiving a message,
 * deserializes the binary payload into the appropriate Message<T> type and
 * forwards it to the provided MessageHandler.
 *
 * @note Dependency: PubSubClient (https://github.com/knolleary/pubsubclient)
 *
 * Usage Example:
 *   MessageHandler& handler = ...;
 *   PubSubClient mqttClient(wifiClient);
 *   tinyrobotics::MQTTMessageReceiverTask receiver(mqttClient, handler,
 * "robot/"); receiver.subscribeAll();
 *   ...
 *   mqttClient.loop(); // in main loop
 */
class MQTTMessageReceiver {
 public:

  /// Construct a receiver for MQTT messages and dispatch to handler.
  MQTTMessageReceiver(PubSubClient& client, MessageHandler& handler,
                      const std::string& baseTopic = "tinyrobotics/")
      : mqttClient_(client), handler_(handler), baseTopic_(baseTopic) {
    // Register the static trampoline as the MQTT callback
    mqttClient_.setCallback(MQTTMessageReceiver::mqttCallbackTrampoline);
    // Store this instance for the trampoline
    instance_ = this;
  }


  /// Subscribe to all supported MQTT topics (call before use).
  bool begin() {
    subscribeAll();
    return true;
  }

  /// Unsubscribe from all MQTT topics.
  void end() { unsubscribeAll(); }

  /// Set the base MQTT topic for receiving messages.
  void setBaseTopic(const std::string& topic) { baseTopic_ = topic; }

 protected:
  PubSubClient& mqttClient_;
  MessageHandler& handler_;
  std::string baseTopic_;
  static MQTTMessageReceiver* instance_;

  void unsubscribeAll() {
    mqttClient_.unsubscribe((baseTopic_ + "value").c_str());
    mqttClient_.unsubscribe((baseTopic_ + "coordinate").c_str());
    mqttClient_.unsubscribe((baseTopic_ + "gps").c_str());
    mqttClient_.unsubscribe((baseTopic_ + "motionstate").c_str());
  }

  // Subscribe to all supported message topics
  void subscribeAll() {
    mqttClient_.subscribe((baseTopic_ + "value").c_str());
    mqttClient_.subscribe((baseTopic_ + "coordinate").c_str());
    mqttClient_.subscribe((baseTopic_ + "gps").c_str());
    mqttClient_.subscribe((baseTopic_ + "motionstate").c_str());
  }

  static bool endsWith(const std::string& str, const std::string& suffix) {
    if (suffix.size() > str.size()) return false;
    return std::equal(suffix.rbegin(), suffix.rend(), str.rbegin());
  }

  // This is now private; use static trampoline for MQTT callback
  void onMqttMessage(char* topic, uint8_t* payload, unsigned int length) {
    std::string t(topic);
    if (endsWith(t, baseTopic_ + "value") && length == sizeof(Message<float>)) {
      Message<float> msg;
      memcpy(&msg, payload, sizeof(msg));
      handler_.onMessage(msg);
    } else if (endsWith(t, baseTopic_ + "coordinate") &&
               length == sizeof(Message<Coordinate<float>>)) {
      Message<Coordinate<float>> msg;
      memcpy(&msg, payload, sizeof(msg));
      handler_.onMessage(msg);
    } else if (endsWith(t, baseTopic_ + "gps") &&
               length == sizeof(Message<GPSCoordinate>)) {
      Message<GPSCoordinate> msg;
      memcpy(&msg, payload, sizeof(msg));
      handler_.onMessage(msg);
    } else if (endsWith(t, baseTopic_ + "motionstate") &&
               length == sizeof(Message<MotionState3D>)) {
      Message<MotionState3D> msg;
      memcpy(&msg, payload, sizeof(msg));
      handler_.onMessage(msg);
    }
  }

  // Static trampoline for PubSubClient callback
  static void mqttCallbackTrampoline(char* topic, uint8_t* payload,
                                     unsigned int length) {
    if (instance_) instance_->onMqttMessage(topic, payload, length);
  }
};

// Static instance pointer for trampoline
inline MQTTMessageReceiver* MQTTMessageReceiver::instance_ = nullptr;

}  // namespace tinyrobotics

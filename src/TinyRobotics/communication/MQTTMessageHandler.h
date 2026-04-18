#pragma once
#include <PubSubClient.h>

#include <string>

#include "MessageHandler.h"

namespace tinyrobotics {

/**
 * @class MQTTMessageHandler
 * @ingroup communication
 * @brief MessageHandler implementation for sending messages via MQTT using
 * PubSubClient.
 *
 * This handler publishes TinyRobotics messages to MQTT topics using the popular
 * Arduino PubSubClient library. Each supported message type is sent as a binary
 * payload (the full Message<T> struct) to a topic derived from the base topic
 * and message type.
 *
 * @note Dependency: PubSubClient (https://github.com/knolleary/pubsubclient)
 *       Install via Arduino Library Manager: "PubSubClient" by Nick O'Leary
 *
 * Usage Example:
 *   #include <PubSubClient.h>
 *   #include <TinyRobotics/communication/MQTTMessageHandler.h>
 *   WiFiClient wifiClient;
 *   PubSubClient mqttClient(wifiClient);
 *   tinyrobotics::MQTTMessageHandler handler(mqttClient, "robot/");
 *   ...
 *   handler.onMessage(msg); // Publishes msg to MQTT
 *
 * The MQTT client must be connected before calling onMessage().
 * The topic for each message is baseTopic + type (e.g., "robot/value",
 * "robot/coordinate").
 *
 * Supported message types:
 *   - Message<float>
 *   - Message<Coordinate<float>>
 *   - Message<GPSCoordinate>
 *   - Message<MotionState3D>
 *
 * The payload is the binary representation of the full Message<T> struct.
 */

class MQTTMessageHandler : public MessageHandler {
 public:
  MQTTMessageHandler(PubSubClient& client,
                     const std::string& baseTopic = "tinyrobotics/")
      : mqttClient_(client), baseTopic_(baseTopic) {}

  bool onMessage(const Message<float>& msg) override {
    // Send as binary (full Message struct)
    const char* data = reinterpret_cast<const char*>(&msg);
    return publishMessageBinary("value", data, sizeof(msg));
  }

  bool onMessage(const Message<Coordinate<float>>& msg) override {
    // Send as binary (full Message struct)
    const char* data = reinterpret_cast<const char*>(&msg);
    return publishMessageBinary("coordinate", data, sizeof(msg));
  }

  bool onMessage(const Message<GPSCoordinate>& msg) override {
    // Send as binary (full Message struct)
    const char* data = reinterpret_cast<const char*>(&msg);
    return publishMessageBinary("gps", data, sizeof(msg));
  }

  bool onMessage(const Message<MotionState3D>& msg) override {
    // Send as binary (full Message struct)
    const char* data = reinterpret_cast<const char*>(&msg);
    return publishMessageBinary("motionstate", data, sizeof(msg));
  }

  void setBaseTopic(const std::string& topic) { baseTopic_ = topic; }

 protected:
  PubSubClient& mqttClient_;
  std::string baseTopic_;

  bool publishMessageBinary(const std::string& type, const char* data,
                            size_t size) {
    std::string topic = baseTopic_ + type;
    return mqttClient_.publish(topic.c_str(),
                               reinterpret_cast<const uint8_t*>(data), size);
  }
};

}  // namespace tinyrobotics

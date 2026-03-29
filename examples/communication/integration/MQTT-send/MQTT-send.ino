/**
 * @file MQTT-send.ino
 * @brief Example: Send TinyRobotics messages as binary over MQTT.
 *
 * Demonstrates how to use PubSubClient with TinyRobotics to send binary-encoded
 * messages over MQTT.
 *
 * - Schedules a periodic message (throttle value) to be sent every 5 seconds.
 * - Publishes the message as a binary payload to an MQTT topic.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 * - PubSubClient: https://github.com/knolleary/pubsubclient
 * - WiFi.h (ESP32/ESP8266)
 *
 * @author Phil Schatzmann
 */

#include <PubSubClient.h>
#include <TinyRobotics.h>
#include <WiFi.h>

const char* ssid = "your-ssid";
const char* password = "your-password";
const char* mqtt_server = "192.168.1.100";
const int mqtt_port = 1883;
const char* topic = "tinyrobotics/messages";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
Scheduler scheduler;

void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

void connectToMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("TinyRoboticsClient")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

void sendMessage(void*) {
  using namespace tinyrobotics;
  Message<float> msg(MessageContent::Throttle, random(0, 100), Unit::Percent,
                     MessageOrigin::RemoteControl);
  mqttClient.publish(topic, (const uint8_t*)&msg, sizeof(msg));
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics MQTT Send Example");

  connectToWiFi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  connectToMQTT();
  scheduler.begin(5000, sendMessage, nullptr);  // Send every 5 seconds
}

void loop() {
  if (!mqttClient.connected()) {
    connectToMQTT();
  }
  mqttClient.loop();
  scheduler.run();
}

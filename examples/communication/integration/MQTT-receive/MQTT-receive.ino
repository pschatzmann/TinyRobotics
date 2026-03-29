/**
 * @file MQTT-receive.ino

 * @brief Example: Receive TinyRobotics messages as binary over MQTT and print
 as JSON.

 * Demonstrates how to use PubSubClient with TinyRobotics to receive
 binary-encoded messages
 * over MQTT and print them in JSON format.

 * - Uses PubSubClient to subscribe to an MQTT topic.
 * - The MQTT callback directly deserializes the binary payload as a
 TinyRobotics Message<float>.
 * - Uses MessageHandlerPrintJSON to print the received message as JSON to
 Serial.
 *
 * Note: This approach assumes the sender publishes TinyRobotics Message<float>
 objects as raw binary payloads.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 * - PubSubClient3: https://github.com/knolleary/pubsubclient
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
MessageHandlerPrintJSON json(Serial);

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
      mqttClient.subscribe(topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Create a stream from the payload buffer
  if (length == sizeof(Message<float>)) {
    Message<float>* msg = reinterpret_cast<Message<float>*>(payload);
    json.onMessage(*msg);  // Print the message as JSON
  } else {
    TRLogger.error(
        "MQTT callback: Invalid message size (expected %d bytes, got %d)",
        sizeof(Message<float>), length);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics MQTT Receive Example");

  connectToWiFi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);
  connectToMQTT();
}

void loop() {
  if (!mqttClient.connected()) {
    connectToMQTT();
  }
  mqttClient.loop();
}
/**
 * @file IP-receive.ino
 * @brief Example: Receive TinyRobotics messages as binary over TCP/IP
 * (WiFiServer) and print as JSON.
 *
 * Demonstrates how to use WiFiServer with TinyRobotics to receive
 * binary-encoded messages and print them in JSON format.
 *
 * - Uses WiFiServer to receive TCP packets.
 * - Uses MessageDispatcher to decode and dispatch messages.
 * - Uses MessageHandlerPrintJSON to print messages as JSON to Serial.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 * - WiFi.h (ESP32/ESP8266)
 *
 * @author Phil Schatzmann
 */

#include <TinyRobotics.h>
#include <WiFi.h>

const char* ssid = "your-ssid";
const char* password = "your-password";
const int port = 9000;

WiFiServer server(port);
WiFiClient client;

MessageHandlerPrintJSON json(Serial);
MessageDispatcher mgr(json);

void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics IP (TCP) Receive Example");

  connectToWiFi();
  server.begin();
  Serial.print("Listening on port ");
  Serial.println(port);
}

void loop() {
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("Client connected");
      mgr.setStream(client);
      mgr.begin();
    }
  }
  if (client && client.connected()) {
    mgr.run();
  }
}

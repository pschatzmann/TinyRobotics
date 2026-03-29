/**
 * @file UDP-send.ino
 * @brief Example: Send TinyRobotics messages as binary over UDP.
 *
 * Demonstrates how to use TinyRobotics to send a binary-encoded message over
 * UDP.
 *
 * - Schedules a periodic message (throttle value) to be sent every 5 seconds.
 * - Uses MessageHandlerBinary to write the message as raw binary to UDP.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 *
 * @author Phil Schatzmann
 */

#include <WiFi.h>
#include <TinyRobotics.h>
#include "TinyRobotics/communication/UDPStream.h"

// WiFi credentials
const char* ssid = "your-ssid";
const char* password = "your-password";

// UDP settings
const char* remoteIp = "192.168.1.100";  // Set the receiver's IP
const int remotePort = 8080;             // Set the receiver's UDP port
const int localPort = 8081;              // Local port for this sender

UDPStream udp;
MessageHandlerBinary out(udp);
Scheduler scheduler;

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
  TRLogger.info("TinyRobotics UDP Send Example");

  connectToWiFi();
  udp.begin(remoteIp, localPort);   

  scheduler.begin(5000, sendMessage, nullptr);  // Send every 5 seconds
}

void sendMessage(void*) {
  Message<float> msg(MessageContent::Throttle, random(0, 100), Unit::Percent,
                     MessageOrigin::RemoteControl);
  out.onMessage(msg);  // Send as binary over UDP
}

void loop() { scheduler.run(); }
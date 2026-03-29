/**
 * @file UDP-receive.ino
 * @brief Example: Receive TinyRobotics messages as binary over UDP and print as JSON.
 *
 * Demonstrates how to use TinyRobotics to receive binary-encoded messages from UDP
 * and print them in JSON format.
 *
 * - Uses UDPStream to receive UDP packets.
 * - Uses MessageDispatcher to decode and dispatch messages from UDP.
 * - Uses MessageHandlerPrintJSON to print messages as JSON to Serial.
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
const int localPort = 8080; // Set your UDP port

// Set up UDPStream to listen on the specified port
UDPStream udp;
MessageHandlerPrintJSON json(Serial);
MessageDispatcher mgr(json, udp);


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
  TRLogger.info("TinyRobotics UDP Receive Example");

  connectToWiFi();

  udp.begin(localPort); // Start listening on UDP port
  mgr.begin();
}

void loop() { mgr.run(); }
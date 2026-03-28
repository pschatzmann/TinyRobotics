/**
 * @file remote-control.ino
 * @brief Example: Remote control using ESPNow and TinyRobotics.
 *
 * Demonstrates how to set up a simple remote control for a car using ESPNow as
 * the communication stream. You can also use Serial, UDPStream, etc. by
 * changing the stream and CommunicationManager configuration.
 *
 * - Sets up a car and message handler for Serial output.
 * - Uses ESPNowStream for wireless communication.
 * - CommunicationManager dispatches received messages to the car.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 * - ESPNow (for ESP32 wireless communication)
 *
 * @author Phil Schatzmann
 */
#include <TinyRobotics.h>

#include "TinyRobotics/communication/ESPNowStream.h"

CarAckerman car;
MessageHandlerPrint printer(Serial);  // forward to Serial
ESPNowStream espnow;
// receive messages from ESPNow and dispatch to car
CommunicationManager mgr(car, espnow);

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics Messaging Example");

  // Log received commands
  car.addMessageHandler(printer);

  // Start ESPNow in receive mode
  if (!espnow.begin()) {
    TRLogger.error("Failed to initialize ESPNowStream");
  }
}

void loop() {
  mgr.run();  // Process incoming messages and dispatch to car
  delay(10);  // Small delay to avoid busy loop
}
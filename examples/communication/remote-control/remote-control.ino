/**
 * @file remote-control.ino
 * @brief Example how to set up a simple remote control using ESPNow.
 * You could also use Serial, USPStream etc. Just change the stream and the configuration of the CommunicationManager.
 * @version 0.1
 * @date 2026-03-26
 * 
 * @copyright Copyright (c) 2026
 * 
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
  car.subscribe(printer);

  // Start ESPNow in receive mode
  if (!espnow.begin()) {
    TRLogger.error("Failed to initialize ESPNowStream");
  }
}

void loop() {
  mgr.run();  // Process incoming messages and dispatch to car
  delay(10);  // Small delay to avoid busy loop
}
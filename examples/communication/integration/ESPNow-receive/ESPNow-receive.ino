/**
 * @file ESPNow-receive.ino
 * @brief Example: Receive TinyRobotics messages as binary over ESP-NOW and print as JSON.
 *
 * Demonstrates how to use TinyRobotics to receive binary-encoded messages from ESP-NOW
 * and print them in JSON format.
 *
 * - Uses ESPNowStream to receive ESP-NOW packets.
 * - Uses MessageDispatcher to decode and dispatch messages from ESP-NOW.
 * - Uses MessageHandlerPrintJSON to print messages as JSON to Serial.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 *
 * @author Phil Schatzmann
 */

#include <TinyRobotics.h>
#include "TinyRobotics/communication/ESPNowStream.h"

ESPNowStream espnow;
MessageHandlerPrintJSON json(Serial);
MessageDispatcher mgr(json, espnow);

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics ESP-NOW Receive Example");

  espnow.begin(); // Start ESP-NOW
  mgr.begin();
}

void loop() { mgr.run(); }
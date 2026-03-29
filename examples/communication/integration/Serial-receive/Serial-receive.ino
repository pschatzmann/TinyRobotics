
#/**
 * @file Serial-receive.ino
 * @brief Example: Receive TinyRobotics messages as binary over Serial and print as JSON.
 *
 * Demonstrates how to use TinyRobotics to receive binary-encoded messages from Serial
 * and print them in JSON format.
 *
 * - Uses MessageDispatcher to decode and dispatch messages from Serial.
 * - Uses MessageHandlerPrintJSON to print messages as JSON to Serial.
 * - Just replace MessageHandlerPrintJSON with any other MessageHandler to forward
 *   messages to a different stream or format.  
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 *
 * @author Phil Schatzmann
 */

#include <TinyRobotics.h>

MessageHandlerPrintJSON json(Serial);
MessageDispatcher mgr(json, Serial);

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics Serial Receive Example");
  
  mgr.begin();
}

void loop() { mgr.run(); }
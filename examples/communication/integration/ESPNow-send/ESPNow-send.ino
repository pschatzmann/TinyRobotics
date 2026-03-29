/**
 * @file ESPNow-send.ino
 * @brief Example: Send TinyRobotics messages as binary over ESP-NOW.
 *
 * Demonstrates how to use TinyRobotics to send a binary-encoded message over ESP-NOW.
 *
 * - Schedules a periodic message (throttle value) to be sent every 5 seconds.
 * - Uses MessageHandlerBinary to write the message as raw binary to ESP-NOW.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 *
 * @author Phil Schatzmann
 */

#include <TinyRobotics.h>
#include "TinyRobotics/communication/ESPNowStream.h"

ESPNowStream espnow;
MessageHandlerBinary out(espnow);
Scheduler scheduler;

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics ESP-NOW Send Example");

  espnow.begin(); // Start ESP-NOW
  scheduler.begin(5000, sendMessage, nullptr); // Send every 5 seconds
}

void sendMessage(void*) {
  Message<float> msg(MessageContent::Throttle, random(0, 100), Unit::Percent,
                     MessageOrigin::RemoteControl);
  out.onMessage(msg); // Send as binary over ESP-NOW
}

void loop() { scheduler.run(); }
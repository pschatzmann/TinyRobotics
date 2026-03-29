#/**
 * @file Serial-send.ino
 * @brief Example: Send TinyRobotics messages as binary over Serial.
 *
 * Demonstrates how to use TinyRobotics to send a binary-encoded message over Serial.
 *
 * - Schedules a periodic message (throttle value) to be sent every 5 seconds.
 * - Uses MessageHandlerBinary to write the message as raw binary to Serial.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 *
 * @author Phil Schatzmann
 */
#include <Arduino.h>
#include <TinyRobotics.h>

// forward binary message to Serial
MessageHandlerBinary out(Serial);
Scheduler scheduler;

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics Serial Send Example");

  // Schedule sendMessage to run after 5 seconds
  scheduler.begin(5000, sendMessage, nullptr);
}

void sendMessage(void*) {
  // Create a message
  Message<float> msg(MessageContent::Throttle, random(0, 100), Unit::Percent,
                     MessageOrigin::RemoteControl);
  out.onMessage(msg);
}

void loop() { scheduler.run(); }
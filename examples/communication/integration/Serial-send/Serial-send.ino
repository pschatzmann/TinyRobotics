#include <Arduino.h>
#include <TinyRobotics.h>

// forward binary message to Serial
MessageHandlerObject out(Serial);
Scheduler scheduler;

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics Serial Send Example");

  // Schedule sendMessage to run after 5 seconds
  scheduler.schedule(sendMessage, 5000);
}

void sendMessage() {
  // Create a message source and send a test message
  Message<float> msg;
  msg.type = MessageContent::Throttle;
  msg.value = random(0, 100);
  msg.unit = Unit::Percent;
  msg.source = MessageOrigin::Controller;
  out.sendMessage(msg);  // This will not be received by any handler since none
                         // are subscribed
}

void loop() { scheduler.run(); }
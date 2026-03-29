
/**
 * @file task.ino
 * @brief Example: Using TinyRobotics RTOS Task and QueueRTOS.
 *
 * Demonstrates how to use the TinyRobotics RTOS Task and QueueRTOS classes to
 * send Message<float> objects from the main loop to a background task, which
 * prints the messages.
 *
 * @author Phil Schatzmann
 */

#include "TinyRobotics.h"
#include "TinyRobotics/concurrency/RTOS.h"


QueueRTOS<Message<float>> queue(8);  // Queue for up to 8 messages
Task consumerTask("Consumer", 2048, 1);
MessageHandlerPrint printer(Serial);

void consumeMessages(void* ref) {
  Message<float> msg;
  if (queue.dequeue(msg)) {
    printer.onMessage(msg);
  } else {
    delay(10);  // No message, wait a bit
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  consumerTask.begin(consumeMessages);
  consumerTask.resume();
}

void loop() {
  Message<float> msg(MessageContent::Throttle, random(0, 100), Unit::Percent,
                     MessageOrigin::RemoteControl);
  if (queue.enqueue(msg)) {
    Serial.println("[Main] Message enqueued");
  } else {
    Serial.println("[Main] Queue full, message dropped");
  }
}

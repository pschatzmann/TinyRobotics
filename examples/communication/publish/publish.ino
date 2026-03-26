/**
 * @file subscriptions.ino
 * @brief This sketch demonstrates how to use the messaging framework in
 * TinyRobotics to create a simple publisher-subscriber system.
 * @version 0.1
 * @date 2026-03-26
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <TinyRobotics.h>

CarAckerman car;
MessageHandlerPrint printer(Serial); // forward to Serial
MessageHandlerPrintXML xml_printer(Serial); // forward to Serial in XML format
MessageHandlerPrintJSON json_printer(Serial); // forward to Serial in JSON format

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics Messaging Example");

  // Subscribe to messages 
  car.addMessageHandler(printer);
  car.addMessageHandler(xml_printer);
  car.addMessageHandler(json_printer);
}

void loop() {
  // Simulate publishing messages from the car
  car.setSpeed(1.5f, DistanceUnit::MPS, MessageSource::Controller);
  car.setSteeringAngle(15.0f, AngleUnit::DEG, MessageSource::Controller);

  delay(5000);  // Wait before publishing next set of messages

}
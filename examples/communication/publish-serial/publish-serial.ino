/**
 * @file publish.ino
 * @brief Example: Publisher-subscriber messaging with TinyRobotics.
 *
 * Demonstrates how to use the TinyRobotics messaging framework to create a
 * simple publisher-subscriber system for a car. Multiple message handlers are
 * used to print messages in different formats (plain, XML, JSON).
 *
 * - Publishes car speed and steering messages.
 * - Subscribes with multiple handlers for different output formats.
 * - Sends the data to Serial: just replace this with any other Stream: e.g.
 * File, UDPStream etd
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 *
 * @author Phil Schatzmann
 */
#include <TinyRobotics.h>

CarAckerman car;
MessageHandlerPrint printer(Serial);         // forward to Serial
MessageHandlerPrintXML xml_printer(Serial);  // forward to Serial in XML format
MessageHandlerPrintJSON json_printer(
    Serial);  // forward to Serial in JSON format

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
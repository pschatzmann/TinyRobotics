/**
 * @file messagebus.ino
 * @brief Example: Broadcasting messages to multiple outputs using MessageBus.
 *
 * Demonstrates how to use the TinyRobotics MessageBus class to connect sensors,
 * vehicles, and message handlers in a publisher-subscriber pattern. This
 * example shows how to broadcast messages from a RangeSensor and CarAckerman to
 * multiple outputs, including Serial and UDP.
 *
 * - Registers JSON message handlers for Serial and UDP output.
 * - Forwards RangeSensor and CarAckerman messages to the bus.
 * - Allows CarAckerman to receive messages from the bus (bidirectional
 * communication).
 * - Simulates obstacle distance measurements and broadcasts them to all
 * registered outputs.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 * - WiFi/UDP (for UDPStream output)
 *
 * @author Phil Schatzmann
 */
#include <TinyRobotics.h>
#include <TinyRobotics/communication/UDPStream.h>

RangeSensor sensor(0);  // 0 degrees = forward
CarAckerman car;
IPAddress ip(192, 168, 1, 100);
int port = 9999;
UDPStream udpStream;  // Example UDP stream on port 12345
MessageHandlerPrintJSON json_printer(Serial);  // Print to Serial in JSON format
MessageHandlerPrintJSON json_udp(udpStream);   // Print to Serial in JSON format
MessageBus bus;

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics MessageBus Example");

  // Forward data to ip address on port 9999
  udpStream.begin(ip, port);

  // Register message destinations (handlers)  with the bus
  bus.add(json_printer);
  bus.add(json_udp);
  bus.add(car);

  // Add bus as destination for sensor messages (sensor -> bus -> handlers)
  sensor.addMessageHandler(bus);
  // Add bus as destination for vehicle messages (car -> bus -> handlers)
  car.addMessageHandler(bus);
}

void loop() {
  sensor.setObstacleDistance(Distance(random(0, 500), DistanceUnit::CM));
  delay(2000);  // Wait before sending the next message
}

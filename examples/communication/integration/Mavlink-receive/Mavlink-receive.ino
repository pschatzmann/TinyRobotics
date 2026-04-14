/**
 * @file Mavlink-receive.ino
 * @brief Example: Receive MAVLink messages over UDP and control an Ackerman car.
 *
 * Demonstrates how to integrate MAVLink telemetry/control data with TinyRobotics.
 *
 * - Connects to WiFi.
 * - Starts a UDP endpoint on the MAVLink default port.
 * - Uses `MavlinkMessageSource` to process incoming MAVLink messages.
 * - Forwards MAVLink messages to `CarAckerman` as a message source.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 * - WiFi.h (ESP32/ESP8266)
 *
 * @author Phil Schatzmann
 */

#include "SimpleMavlinkDrone.h"
#include "SimpleUDP.h"
#include "TinyRobotics.h"
#include "TinyRobotics/communication/MavlinkMessageSource.h"
#include "WiFi.h"

const int port = 14550;
const char* ssid = "ssid";
const char* password = "password";
SimpleUDP udp;  // WiFiUDP udp;
SimpleMavlinkDrone drone(udp);
MavlinkMessageSource mav(drone);
CarAckerman car;

void connectToWifi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }

  Serial.print("Connected with IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  // setup log
  Serial.begin(115200);

  // setup Wifi
  connectToWifi();

  // start the  server
  udp.begin(port);

  // subscribe to MAVLink messages and start the car control
  mav.subscribe(car);
  car.begin();
}

void loop() { mav.update(); }
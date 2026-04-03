
/**
 * @file RCCarAckermanWithRemoteSensor.ino
 * @brief Example: RC Ackerman car with remote control and range sensor obstacle avoidance.
 *
 * This example demonstrates how to control an Ackerman steering vehicle using TinyRobotics
 * and VirtualGamePadArduino, with additional integration of a range (ultrasonic) sensor for
 * basic obstacle avoidance. The RCGamepadMessageSource class receives gamepad input from an
 * Android device running the VirtualGamePad app, which is translated into TinyRobotics messages
 * and forwarded to the vehicle class. The range sensor periodically measures the distance to
 * obstacles ahead and automatically reduces the car's speed as it approaches an obstacle.
 *
 * ## Usage
 * - Connect your ESP32 to WiFi (update SSID and password below).
 * - Install the VirtualGamePadArduino library:
 *     https://github.com/pschatzmann/VirtualGamePadArduino
 * - On your Android device, install and run the VirtualGamePad app.
 * - Connect the app to the ESP32's IP address.
 * - Use the gamepad controls to drive the vehicle.
 * - The car will automatically slow down as it approaches obstacles detected by the range sensor.
 *
 * ## Customization
 * - To use a different vehicle type, replace `CarAckerman` with another class
 *   (e.g., `CarDifferential`, `Boat`, `Drone`, `Plane`) and adjust the pin setup.
 * - The RCGamepadMessageSource supports multiple control scenarios via the
 *   `ControlScenario` enum.
 * - You can adjust the obstacle avoidance behavior by changing the breakingDistance value.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 * - VirtualGamePadArduino: https://github.com/pschatzmann/VirtualGamePadArduino
 * - Ultrasonic sensor library (for distance measurement)
 *
 * ## Pinout Example
 *   car.setPins(5, 6, 9, 10); // in1, in2, pwm, steeringPin
 *   Ultrasonic ultrasonic(triggerPin, echoPin);
 *
 * @author Phil Schatzmann
 */

#include <WiFi.h>

#include "TinyRobotics.h"
#include "TinyRobotics/communication/RCGamepadMessageSource.h"
#include <Ultrasonic.h>
#undef CM

const char* ssid = "your-ssid";
const char* password = "your-password";
WiFiServer server(80);
RCGamepadMessageSource rcSource(server);
CarAckerman car;
Ultrasonic ultrasonic(12, 13);        // Example pins for trigger and echo
RangeSensor sensor(0);                // 0 degrees = forward
MessageHandlerPrint printer(Serial);  // print info on Serial
Scheduler scheduler;
Distance breakingDistance(2, DistanceUnit::M);

// We use WiFi for RC control
void startWiFi() {
  // Connect to WiFi (replace with your network credentials)
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void processMeasurement(void*) {
  // Read distance in cm from the ultrasonic sensor
  int distance_cm = ultrasonic.read();
  // Convert to meters and update the RangeSensor
  sensor.setObstacleDistance(Distance(distance_cm, DistanceUnit::CM));
  // Adapt car speed for obstacle
  car.setSpeedFactor(sensor.getSpeedFactor(breakingDistance));
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("RC Car example starting");

  // Set up the car pins (example for 1 drive motor and 1 steering servo)
  car.setPins(5, 6, 9, 10);         // in1, in2, pwm, steeringPin
  rcSource.subscribe(car);  // Connect the car to the RC message source

  // Start the RC gamepad message source
  rcSource.begin(ControlScenario::Car);

  // --- Range Sensor Setup ---
  sensor.subscribe(printer);
  sensor.begin();
  scheduler.begin(500, processMeasurement, nullptr); // 500 ms
}


void loop() {
  // Update the RC gamepad message source to handle incoming messages
  rcSource.update();
  scheduler.run();
}

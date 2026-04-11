
/**
 * @file RcVehicle.ino
 * @brief Example: Remote controlled vehicle using TinyRobotics and VirtualGamePadArduino.
 * 
 * This example demonstrates how to control a vehicle (car, boat, drone, or plane)
 * using the RCGamepadMessageSource class to receive gamepad input from an Android
 * device running the VirtualGamePad app. The received messages are translated into
 * TinyRobotics messages and forwarded to the vehicle class.
 *
 * ## Usage
 * - Connect your ESP32 to WiFi (update SSID and password below).
 * - Install the VirtualGamePadArduino library:
 *     https://github.com/pschatzmann/VirtualGamePadArduino
 * - On your Android device, install and run the VirtualGamePad app.
 * - Connect the app to the ESP32's IP address.
 * - Use the gamepad controls to drive the vehicle.
 *
 * ## Customization
 * - To use a different vehicle type, replace `CarAckerman` with another class
 *   (e.g., `CarDifferential`, `Boat`, `Drone`, `Plane`) and adjust the pin setup.
 * - The RCGamepadMessageSource supports multiple control scenarios via the
 *   `ControlScenario` enum.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 * - VirtualGamePadArduino: https://github.com/pschatzmann/VirtualGamePadArduino
 *
 * ## Pinout Example
 *   car.setPins(5, 6, 9); // in1, in2, steeringPin
 *
 * @author Phil Schatzmann
 */

#include <WiFi.h>

#include "TinyRobotics.h" 
#include "TinyRobotics/communication/RCGamepadMessageSource.h" 

WiFiServer server(80);
RCGamepadMessageSource rcSource(server);
CarAckerman car;
const char* ssid = "your-ssid";
const char* password = "your-password";

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

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("RC Car example starting");

  // Set up the car pins (example for 1 drive motor and 1 steering servo)
  car.setPins(5, 6, 9);         // in1, in2, steeringPin
  car.begin();
  rcSource.subscribe(car);  // Connect the car to the RC message source

  // Start the RC gamepad message source
  rcSource.begin(ControlScenario::Car);
}

void loop() {
  // Update the RC gamepad message source to handle incoming messages
  rcSource.update();
}

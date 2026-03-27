// A remote controlled car example using the RCGamepadMessageSource to receive
// gamepad input from an Android game controller and control the car accordingly.
// For other vehicle types, just replace CarAckerman with the appropriate
// vehicle class (e.g., CarDifferential, Boat, Drone, Plane) and set up the pins
// accordingly.
// You must install the VirtualGamePadArduino library and set up the Android game controller

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
  car.setPins(5, 6, 9, 10);         // in1, in2, pwm, steeringPin
  rcSource.addMessageHandler(car);  // Connect the car to the RC message source

  // Start the RC gamepad message source
  rcSource.begin(ControlScenario::Car);
}

void loop() {
  // Update the RC gamepad message source to handle incoming messages
  rcSource.update();
}

/**
 * @file IP-send.ino
 * @brief Example: Send TinyRobotics messages as binary over TCP/IP (WiFiClient).
 *
 * Demonstrates how to use WiFiClient with TinyRobotics to send binary-encoded messages over TCP.
 *
 * - Schedules a periodic message (throttle value) to be sent every 5 seconds.
 * - Uses MessageHandlerBinary to write the message as raw binary to TCP.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 * - WiFi.h (ESP32/ESP8266)
 *
 * @author Phil Schatzmann
 */

#include <TinyRobotics.h>
#include <WiFi.h>

const char* ssid = "your-ssid";
const char* password = "your-password";
const char* serverIp = "192.168.1.100"; // Set the receiver's IP
const int serverPort = 9000;            // Set the receiver's TCP port

WiFiClient client;
MessageHandlerBinary out;
Scheduler scheduler;

void connectToWiFi() {
	WiFi.begin(ssid, password);
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	Serial.println("\nWiFi connected");
}

void connectToServer() {
	while (!client.connect(serverIp, serverPort)) {
		Serial.print("Connecting to server...");
		delay(1000);
	}
	Serial.println("\nConnected to server");
	out.setOutput(client);
}

void setup() {
	Serial.begin(115200);
	delay(1000);
	TRLogger.info("TinyRobotics IP (TCP) Send Example");

	connectToWiFi();
	connectToServer();
	scheduler.begin(5000, sendMessage, nullptr); // Send every 5 seconds
}

void sendMessage(void*) {
	if (!client.connected()) {
		connectToServer();
	}
	Message<float> msg(MessageContent::Throttle, random(0, 100), Unit::Percent,
										 MessageOrigin::RemoteControl);
	out.onMessage(msg); // Send as binary over TCP
}

void loop() { scheduler.run(); }

/**
 * @file IEEE802_15_4-receive.ino
 * @brief Example: Receive TinyRobotics messages as binary over IEEE 802.15.4 and print as JSON.
 *
 * Demonstrates how to use ESP32TransceiverIEEE802_15_4 to receive binary-encoded messages
 * and TinyRobotics to decode and print them as JSON.
 *
 * - Uses ESP32TransceiverIEEE802_15_4 to receive IEEE 802.15.4 packets.
 * - Uses MessageDispatcher to decode and dispatch messages.
 * - Uses MessageHandlerPrintJSON to print messages as JSON to Serial.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 * - ESP32TransceiverIEEE802_15_4: https://github.com/pschatzmann/ESP32TransceiverIEEE802_15_4
 *
 * @author Phil Schatzmann
 */

#include <TinyRobotics.h>
#include "ESP32TransceiverStreamIEEE802_15_4.h"

const channel_t channel = channel_t::CHANNEL_11;
Address local({0xAB, 0xCD});  // Different from sender
ESP32TransceiverStreamIEEE802_15_4 transceiver(channel, 0x1234, local);

MessageHandlerPrintJSON json(Serial);
MessageDispatcher mgr(json, transceiver);

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics IEEE 802.15.4 Receive Example");

  transceiver.begin(); // Start IEEE 802.15.4 transceiver
  mgr.begin();
}

void loop() { mgr.run(); }
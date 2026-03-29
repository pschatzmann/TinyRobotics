/**
 * @file IEEE802_15_4-send.ino
 * @brief Example: Send TinyRobotics messages as binary over IEEE 802.15.4.
 * @note  This example works only for ESP devices that support IEEE802.15.4
 e.g.C6 .

 * Demonstrates how to use ESP32TransceiverStreamIEEE802_15_4 to send
 binary-encoded messages
 * with TinyRobotics.
 *
 * - Schedules a periodic message (throttle value) to be sent every 5 seconds.
 * - Uses MessageHandlerBinary to write the message as raw binary to IEEE
 802.15.4.
 *
 * ## Dependencies
 * * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 * - ESP32TransceiverIEEE802_15_4:
 https://github.com/pschatzmann/ESP32TransceiverIEEE802_15_4
 *
 * @author Phil Schatzmann
 */

#include <TinyRobotics.h>

#include "ESP32TransceiverStreamIEEE802_15_4.h"

const channel_t channel = channel_t::CHANNEL_11;
Address local({0xAB, 0xCF});
Address remote({0xAB, 0xCD});  // Receiver address (must match receiver sketch)
ESP32TransceiverStreamIEEE802_15_4 transceiver(channel, 0x1234, local);

MessageHandlerBinary out(transceiver);
Scheduler scheduler;

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics IEEE 802.15.4 Send Example");

  // maximise throughput by reducing inter-frame delay, but still allow some
  // time for
  transceiver.setSendDelay(5);
  transceiver.setDestinationAddress(Address({0xAB, 0xCD}));
  transceiver.begin();  // Start IEEE 802.15.4 transceiver
  scheduler.begin(5000, sendMessage, nullptr);  // Send every 5 seconds
}

void sendMessage(void*) {
  Message<float> msg(MessageContent::Throttle, random(0, 100), Unit::Percent,
                     MessageOrigin::RemoteControl);
  out.onMessage(msg);  // Send as binary over IEEE 802.15.4
}

void loop() { scheduler.run(); }
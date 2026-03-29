/**
 * @file RF455MHz-send.ino
 * @brief Example: Send TinyRobotics messages over 433/455MHz RF using
 * PulseWire.
 *
 * Demonstrates how to use the PulseWire library to send binary-encoded
 * TinyRobotics messages via a 433/455MHz RF transmitter module.
 *
 * - The data is manchester-encoded and transmitted as RF pulses.
 * - Encodes TinyRobotics messages as bytes and transmits them as RF pulses.
 * - Uses PulseWire for RF transmission.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 * - PulseWire: https://github.com/pschatzmann/PulseWire
 *
 * @author Phil Schatzmann
 */

#include <TinyRobotics.h>
// pulsewire
#include "Codecs.h"
#include "DriverArduino.h"
#include "Transceiver.h"

const uint8_t txPin = 5;  // Adjust to your RF transmitter data pin
int baud = 1000;

ManchesterCodec codec;
DigitalSignal digital;
TxDriverArduino tx(codec, txPin, digital, true);
Transceiver rfTransmitter(tx);
Scheduler scheduler;

void setup() {
  Serial.begin(115200);
  TRLogger.info("TinyRobotics RF 455MHz Send Example (PulseWire)");
  rfTransmitter.begin(baud);
  scheduler.begin(1000, sendMessage, nullptr);  // Send every 1 second
}

void sendMessage(void*) {
  Message<float> msg(MessageContent::Throttle, random(0, 100), Unit::Percent,
                     MessageOrigin::RemoteControl);
  rfTransmitter.write((uint8_t*)&msg, sizeof(msg));
  rfTransmitter.flush();  // Ensure the message is sent immediately
}

void loop() { scheduler.run(); }

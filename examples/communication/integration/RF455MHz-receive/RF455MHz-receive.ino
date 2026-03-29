/**
 * @file RF455MHz-receive.ino
 * @brief Example: Receive TinyRobotics messages over 433/455MHz RF using
 * PulseWire.
 *
 * Demonstrates how to use the PulseWire library to receive binary-encoded
 * TinyRobotics messages via a 433/455MHz RF receiver module.
 *
 * - Decodes RF pulses into bytes using PulseWire.
 * - Parses and dispatches TinyRobotics messages using MessageParser.
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

const uint8_t rxPin = 5;  // Adjust to your RF receiver data pin
int baud = 1000;

ManchesterCodec codec;
RxDriverArduino rx(codec, rxPin);
Transceiver rfReceiver(rx);
MessageHandlerPrintJSON json(Serial);
MessageDispatcher mgr(json, rfReceiver);

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics RF 455MHz Receive Example (PulseWire)");

  mgr.begin();
  rfReceiver.begin(baud);
}

void loop() { mgr.run(); }

/**
 * @file IR-receive.ino
 * @brief Example: Receive TinyRobotics messages over IR using PulseWire.
 *
 * Demonstrates how to use the PulseWire library to receive binary-encoded
 * TinyRobotics messages via an IR receiver.
 *
 * - Decodes IR pulses into bytes using PulseWire.
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

const uint8_t rxPin = 5;
int baud = 1000;

ManchesterCodec codec;
RxDriverArduino rx(codec, rxPin);
Transceiver irReceiver(rx);
MessageHandlerPrintJSON json(Serial);
MessageDispatcher mgr(json, irReceiver);

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics Serial Receive Example");

  mgr.begin();
  irReceiver.begin(baud);
}

void loop() { mgr.run(); }

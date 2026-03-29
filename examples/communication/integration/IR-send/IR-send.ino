/**
 * @file IR-send.ino
 * @brief Example: Send TinyRobotics messages over IR using PulseWire.
 *
 * Demonstrates how to use the PulseWire library to send binary-encoded
 * TinyRobotics messages via an IR LED Module.
 *
 * - Encodes TinyRobotics messages as bytes and transmits them as IR pulses.
 * - The data is manchester-encoded and transmitted as IR pulses.
 * - Uses PulseWire for IR transmission.
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

const uint8_t txPin = 5;
int baud = 1000;
int carrierFreq = 38000;

ManchesterCodec codec;
PWMSignal pwm(carrierFreq);
TxDriverArduino tx(codec, txPin, pwm, true);
Transceiver irTransmitter(tx);
Scheduler scheduler;

void setup() {
  Serial.begin(115200);
  TRLogger.info("TinyRobotics IR Send Example (PulseWire)");
  irTransmitter.begin(baud);
  scheduler.begin(1000, sendMessage, nullptr);  // Send every 1 second
}

void sendMessage(void*) {
  Message<float> msg(MessageContent::Throttle, random(0, 100), Unit::Percent,
                     MessageOrigin::RemoteControl);
  irTransmitter.write((uint8_t*)&msg, sizeof(msg));
  irTransmitter.flush();  // Ensure the message is sent immediately
}

void loop() {
  scheduler.run();
}

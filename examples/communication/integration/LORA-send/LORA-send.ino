/**
 * @file LORA-send.ino
 * @brief Example: Send TinyRobotics messages as binary over LoRa (RadioLib).
 *
 * Demonstrates how to use RadioLib to send binary-encoded TinyRobotics messages
 * over LoRa.
 *
 * - Schedules a periodic message (throttle value) to be sent every 5 seconds.
 * - Uses RadioLib to transmit the message as a binary packet.
 *
 * ## Dependencies
 * - TinyRobotics: https://github.com/pschatzmann/TinyRobotics
 * - RadioLib: https://github.com/jgromes/RadioLib
 *
 * @author Phil Schatzmann
 */

#include <RadioLib.h>
#include <TinyRobotics.h>

// LoRa module pins (adjust for your board/module)
#define LORA_CS 18
#define LORA_RST 14
#define LORA_IRQ 26

Module lora_module(LORA_CS, LORA_IRQ, LORA_RST);
SX1278 lora(&lora_module);
Scheduler scheduler;

void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics LoRa Send Example");

  int state = lora.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("LoRa init failed, code: ");
    Serial.println(state);
    while (true);
  }
  Serial.println("LoRa init succeeded!");
  scheduler.begin(5000, sendMessage, nullptr);  // Send every 5 seconds
}

void sendMessage(void*) {
  Message<float> msg(MessageContent::Throttle, random(0, 100), Unit::Percent,
                     MessageOrigin::RemoteControl);
  lora.transmit((uint8_t*)&msg, sizeof(msg));
}

void loop() { scheduler.run(); }

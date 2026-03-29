/**
 * @file LORA-receive.ino
 * @brief Example: Receive TinyRobotics messages as binary over LoRa (RadioLib) and print as JSON.
 *
 * Demonstrates how to use RadioLib to receive binary-encoded messages over LoRa
 * and TinyRobotics to decode and print them in JSON format.
 *
 * - Uses RadioLib to receive LoRa packets.
 * - Uses MessageDispatcher to decode and dispatch messages from the received buffer.
 * - Uses MessageHandlerPrintJSON to print messages as JSON to Serial.
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
#define LORA_CS   18
#define LORA_RST  14
#define LORA_IRQ  26

Module lora_module(LORA_CS, LORA_IRQ, LORA_RST);
SX1278 lora(&lora_module);
MessageHandlerPrintJSON json(Serial);
MessageParser parser;


void setup() {
  Serial.begin(115200);
  delay(1000);
  TRLogger.info("TinyRobotics LoRa Receive Example");

  int state = lora.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("LoRa init failed, code: ");
    Serial.println(state);
    while (true);
  }
  Serial.println("LoRa init succeeded!");
}

void loop() {
  uint8_t buf[64];
  int len = lora.receive(buf, sizeof(buf));
  if (len > 0) {
    parser.parse(buf, len , json);
  }
}
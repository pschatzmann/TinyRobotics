#pragma once

#ifndef DEFAULT_TYPE
#define DEFAULT_TYPE float
#endif

#ifdef ESP32
#define SUPPORTS_ANALOG_WRITE_FREQ
#define analogWriteFreq(pin, freq) analogWriteFrequency(pin, freq)
#endif

#ifdef ESP8266
#define SUPPORTS_ANALOG_WRITE_FREQ
#define analogWriteFreq(pin, freq) analogWriteFreq(freq)
#endif

#ifdef ARDUINO_ARCH_RP2040
#define SUPPORTS_ANALOG_WRITE_FREQ
#define analogWriteFreq(pin, freq) analogWriteFreq(freq)
#endif

// use servo library for motor control by default, can be overridden by defining
#ifndef USE_SERVO_LIBRARY
#define USE_SERVO_LIBRARY true
#endif
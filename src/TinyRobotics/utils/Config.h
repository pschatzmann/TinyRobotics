#pragma once

// Default type for coordinates, distances, and angles. 
#ifndef DEFAULT_TYPE
#define DEFAULT_TYPE float
#endif

// Automatically include all headers
#ifndef USE_INCLUDE_ALL
#define USE_INCLUDE_ALL true
#endif


// ESP32-specific analog write frequency support
#ifdef ESP32
#define SUPPORTS_ANALOG_WRITE_FREQ
#define analogWriteFreq(pin, freq) analogWriteFrequency(pin, freq)
#endif


// ESP8266-specific analog write frequency support
#ifdef ESP8266
#define SUPPORTS_ANALOG_WRITE_FREQ
#define analogWriteFreq(pin, freq) analogWriteFreq(freq)
#endif


// RP2040-specific analog write frequency support
#ifdef ARDUINO_ARCH_RP2040
#define SUPPORTS_ANALOG_WRITE_FREQ
#define analogWriteFreq(pin, freq) analogWriteFreq(freq)
#endif

// Use external motor libraries (e.g., Servo, FastAccelStepper) if available
#ifndef USE_EXTERNAL_MOTOR_LIBRARIES
#define USE_EXTERNAL_MOTOR_LIBRARIES true
#endif

// use servo library for motor control 
#ifndef USE_SERVO_LIBRARY
#define USE_SERVO_LIBRARY USE_EXTERNAL_MOTOR_LIBRARIES
#endif

// Use FastAccel Stepper Library
#ifndef USE_FASTACCEL_STEPPER
#define USE_FASTACCEL_STEPPER USE_EXTERNAL_MOTOR_LIBRARIES
#endif


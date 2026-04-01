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

#ifndef USE_EXTERNAL_MOTOR_LIBRARIES
#define USE_EXTERNAL_MOTOR_LIBRARIES true
#endif

// use servo library for motor control by default, can be overridden by defining
#ifndef USE_SERVO_LIBRARY
#define USE_SERVO_LIBRARY USE_EXTERNAL_MOTOR_LIBRARIES
#endif

// use the generic MotorCB callback-based motors by default
#ifndef USE_FASTACCEL_STEPPER
#define USE_FASTACCEL_STEPPER USE_EXTERNAL_MOTOR_LIBRARIES
#endif

#ifndef USE_SERVO_MOTOR
#define USE_SERVO_MOTOR USE_EXTERNAL_MOTOR_LIBRARIES
#endif
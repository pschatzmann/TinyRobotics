#pragma once

/**
 * @file ServoLibrary.h
 * @ingroup motors
 * @brief Central selection of the servo implementation used by the motor
 * drivers.
 *
 * All motor drivers (ServoMotor, BrushlessMotor, ...) must include this header
 * instead of including Servo.h or ESP32Servo.h directly. The platform specific
 * library is selected here, so changing the implementation only requires
 * changing this file.
 */

#include "Arduino.h"

#ifdef ESP32
#include "ESP32Servo.h"
#else
#include "Servo.h"
#endif

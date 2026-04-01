#pragma once
/**
 * @defgroup motors Motors
 * @ingroup main
 * @brief Motor drivers and abstractions (DC, stepper, servo)
 */

#include "GenericMotor.h"
#include "BrushedMotor.h"
#if (USE_SERVO_MOTOR)
#include "BrushlessMotor.h"
#include "ServoMotor.h"
#else
namespace tinyrobotics {
using BrushlessMotor = GenericMotor;
using ServoMotor = GenericMotor;
}
#endif
#if (USE_FASTACCEL_STEPPER)
#include "StepperMotor.h"
#else
namespace tinyrobotics {
using StepperMotor = GenericMotor;
}
#endif
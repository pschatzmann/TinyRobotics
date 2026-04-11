#pragma once
/**
 * @defgroup motors Motors
 * @ingroup main
 * @brief Motor drivers and abstractions (DC, stepper, servo)
 */

#include "IMotor.h"
#include "GenericMotor.h"
#include "BrushedMotor.h"
#if (USE_SERVO_LIBRARY)
#include "BrushlessMotor.h"
#include "ServoMotor.h"
#else
namespace tinyrobotics {
    
template <typename T = float>
using BrushlessMotor = GenericMotor<T>;

template <typename T = float>
using ServoMotor = GenericMotor<T>;

}
#endif
#if (USE_FASTACCEL_STEPPER)
#include "StepperMotor.h"
#else
namespace tinyrobotics {
template <typename T = float>
using StepperMotor = GenericMotor<T>;
}
#endif
#pragma once

/**
 * @defgroup main TinyRobotics
 * @brief A powerful Robotics library (not only) for Arduino
 * @file TinyRobotics.h
 * @author Phil Schatzmann
 * @copyright MIT License
 */

/**
 * @defgroup concurrency Concurrency
 * @ingroup main
 * @brief Concurrency and cooperative multitasking utilities
 */

#include "TinyRobotics/utils/Config.h"
#if USE_INCLUDE_ALL
#include "TinyRobotics/communication/Communication.h"
#include "TinyRobotics/control/Control.h"
#include "TinyRobotics/coordinates/Coordinates.h"
#include "TinyRobotics/fusion/Fusion.h"
#include "TinyRobotics/imu/IMU.h"
#include "TinyRobotics/localization/Localization.h"
#include "TinyRobotics/maps/Maps.h"
#include "TinyRobotics/motors/Motors.h"
#include "TinyRobotics/odometry/Odometry.h"
#include "TinyRobotics/planning/Planning.h"
#include "TinyRobotics/sensors/Sensors.h"
#include "TinyRobotics/units/Units.h"
#include "TinyRobotics/vehicles/Vehicles.h"
#endif // USE_INCLUDE_ALL

// Make sure that Nullprint is available
#include "TinyRobotics/utils/NullPrint.h"

#if defined(ARDUINO) || defined(USE_TR_NAMESPACE)
using namespace tinyrobotics;
#endif

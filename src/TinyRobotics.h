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

#include "TinyRobotics/communication/Communication.h"
#include "TinyRobotics/control/Control.h"
#include "TinyRobotics/coordinates/Coordinates.h"
#include "TinyRobotics/imu/IMU.h"
#include "TinyRobotics/maps/Maps.h"
#include "TinyRobotics/motors/Motors.h"
#include "TinyRobotics/planning/Planning.h"
#include "TinyRobotics/sensors/Sensors.h"
#include "TinyRobotics/slam/SLAM.h"
#include "TinyRobotics/units/Units.h"
#include "TinyRobotics/utils/Utils.h"
#include "TinyRobotics/vehicles/Vehicles.h"
#include "TinyRobotics/odometry/Odometry.h"
#include "TinyRobotics/fusion/Fusion.h"

#if defined(ARDUINO) || defined(USE_TR_NAMESPACE)
using namespace tinyrobotics;
#endif



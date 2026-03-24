#pragma once
#include <cstdint>
#include "Config.h"

namespace tinyrobotics {

enum class FrameType : uint8_t {
  WORLD,
  ODOMETRY,
  BASE,
  CAMERA,
  LIDAR,
  WHEEL,
  CUSTOM,
  OBSTACLE,
  TEMP
};

enum class CellState : int8_t { UNKNOWN = -1, FREE = 0, OCCUPIED = 100 };

using DistanceM = DEFAULT_TYPE;
using AngleDeg = DEFAULT_TYPE;

}  // namespace tinyrobotics
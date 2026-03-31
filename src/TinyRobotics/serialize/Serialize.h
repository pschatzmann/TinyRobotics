#pragma once
 /**
 * @defgroup serialize Serialize
 * @ingroup main
 * @brief Serialization and deserialization utilities
 */

#include "Serializable.h"
#include "SerializeSTL.h"
#if defined(ARDUINO)
#include "SerializeArduino.h"
#endif

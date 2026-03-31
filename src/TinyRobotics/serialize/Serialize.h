#pragma once
 /**
 * @defgroup serialize Serialize
 * @ingroup main
 */

#include "Serializable.h"
#include "SerializeSTL.h"
#if defined(ARDUINO)
#include "SerializeArduino.h"
#endif

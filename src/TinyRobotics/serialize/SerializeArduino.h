#pragma once
#include "Stream.h"

namespace tinyrobotics {

/**
 * @class SerializeArduino
 * @ingroup serialize
 * @brief Arduino-compatible serialization utility for Serializable objects.
 *
 * The SerializeArduino class provides methods to serialize and deserialize objects
 * implementing the Serializable interface, using Arduino's Stream and Print APIs.
 *
 * - Supports both input (Stream) and output (Print) for flexible I/O.
 * - print() writes the object's string representation to the output stream.
 * - read() reads a string from the input stream and populates the object using fromString().
 * - Enables saving/loading object state or transmitting data over serial or other stream-based interfaces.
 *
 * Example:
 * @code
 *   SerializeArduino serializer(Serial);
 *   serializer.print(myObject); // Write to Serial
 *   serializer.read(myObject);  // Read from Serial
 * @endcode
 *
 * @note The Serializable interface defines how objects are converted to/from strings.
 *
 * @see Serializable
 */
class SerializeArduino {
 public:
  SerializeArduino(Stream& io) {
    p_io = &io;
    p_out = &io;
  }
  SerializeArduino(Print& out) { p_out = &out; }

  size_t print(Serializable& obj) {
    if (p_out == nullptr) return 0;
    return p_out->println(obj.toString().c_str());
  }

  bool read(Serializable& obj) {
    if (p_io == nullptr) return false;
    StringType str = p_io->readStringUntil('\n');
    return obj.fromString(str);
  }

 protected:
  Stream* p_io = nullptr;
  Print* p_out = nullptr;
};

}  // namespace tinyrobotics
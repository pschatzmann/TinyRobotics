#pragma once
#include "Stream.h"

namespace tinyrobotics {

/**
 * @brief This class provides methods to serialize and deserialize objects that
 * implement the Serializable interface. It can work with both Stream and Print
 * interfaces, allowing for flexible input and output options. The print()
 * method takes a Serializable object and writes its string representation to
 * the output stream, while the read() method reads a string from the input
 * stream and populates a Serializable object using the fromString() method.
 * This class can be used to easily save and load object state, or to transmit
 * object data over a serial connection or other stream-based interface. The
 * design allows for separation of concerns, where the Serializable interface
 * defines how objects can be converted to and from strings, and the Serialize
 * class handles the actual input/output operations, making it reusable for any
 * Serializable object.
 *
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
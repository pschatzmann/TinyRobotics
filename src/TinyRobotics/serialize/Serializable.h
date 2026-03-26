#pragma once
#include "TinyRobotics/utils/Config.h"

namespace tinyrobotics {

/**
 * @brief This class defines an interface for serializable objects that can be
 * converted to and from a string representation. It provides two pure virtual
 * methods: toString() for converting the object to a string, and fromString()
 * for populating the object from a string input. This interface can be
 * implemented by any class that needs to support serialization and
 * deserialization, allowing for easy storage, transmission, or logging of
 * object state in a human-readable
 */
class Serializable {
 public:
  virtual std::string toString() const = 0;
  const char* toCString() const { return toString().c_str(); }
  virtual bool fromString(const std::string& in) = 0;
  virtual bool fromString(const char* in) { return fromString(std::string(in)); }
};

} // namespace tinyrobotics
#pragma once
#include "Stream.h"
#include "TinyRobotics/utils/Config.h"

namespace tinyrobotics {

// Reads a line from the stream into a std::stream. Returns true if a line was
// read, false on EOF.
bool readLine(Stream& s, std::string& out) {
  out.clear();
  while (true) {
    int c = s.read();
    if (c < 0) {
      // If nothing read, return false to signal EOF
      return !out.empty();
    }
    if (c == '\n') break;
    out += static_cast<char>(c);
  }
  return true;
}

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
  virtual bool fromString(const char* in) {
    return fromString(std::string(in));
  }
  size_t writeTo(Print& out) const { return out.println(toCString()); }
  size_t readFrom(Stream& in) {
    std::string inStr;
    if (!readLine(in, inStr)) return 0;
    return fromString(inStr.c_str()) ? inStr.length() + 1 : 0;
  }
};

}  // namespace tinyrobotics
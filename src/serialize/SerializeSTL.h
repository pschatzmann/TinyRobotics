#pragma once
// STL-based serialization for Serializable objects
#include <iostream>
#include <string>

namespace tinyrobotics {

class SerializeSTL {
 public:
  SerializeSTL(std::istream* in, std::ostream* out) : p_in(in), p_out(out) {}
  SerializeSTL(std::ostream* out) : p_in(nullptr), p_out(out) {}
  SerializeSTL(std::istream* in) : p_in(in), p_out(nullptr) {}

  size_t print(Serializable& obj) {
    if (!p_out) return 0;
    std::string str = obj.toString();
    (*p_out) << str << std::endl;
    return str.size();
  }

  bool read(Serializable& obj) {
    if (!p_in) return false;
    std::string str;
    if (!std::getline(*p_in, str)) return false;
    return obj.fromString(str);
  }

 protected:
  std::istream* p_in = nullptr;
  std::ostream* p_out = nullptr;
};

}  // namespace tinyrobotics
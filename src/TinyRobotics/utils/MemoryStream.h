#pragma once
#include "Stream.h"
#include "stdint.h"

namespace tinyrobotics {

/**
 * @class MemoryStream
 * @ingroup utils
 * @brief Read-only memory stream for wrapping a buffer as an Arduino Stream.
 *
 * This class allows you to treat a memory buffer as an input Stream, enabling
 * code that expects a Stream (such as parsers or message dispatchers) to operate
 * directly on in-memory data. The stream is read-only: write() does nothing.
 */
class MemoryStream : public Stream {
  const uint8_t* _data;
  size_t _len, _pos;
 public:
  MemoryStream(const uint8_t* d, size_t l) : _data(d), _len(l), _pos(0) {}
  int available() override { return _len - _pos; }
  int read() override { return (_pos < _len) ? _data[_pos++] : -1; }
  int peek() override { return (_pos < _len) ? _data[_pos] : -1; }
  void flush() override {}
  size_t write(uint8_t) override { return 0; } // Read-only
  size_t readBytes(char* buf, size_t n)  {
    size_t toRead = (_pos + n > _len) ? (_len - _pos) : n;
    memcpy(buf, _data + _pos, toRead);
    _pos += toRead;
    return toRead;
  }
};
}
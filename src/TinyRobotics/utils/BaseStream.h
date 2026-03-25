#pragma once
#include "Stream.h"
#include "TinyRobotics/utils/Buffers.h"

#define STREAM_READ_OVERRIDE 
#define STREAM_READCHAR_OVERRIDE
#define DEFAULT_BUFFER_SIZE 256
#define MAX_SINGLE_CHARS 16

namespace tinyrobotics {

/**
 * @brief Base class for all Streams. It relies on write(const uint8_t *buffer,
 * size_t size) and readBytes(uint8_t *buffer, size_t length).
 * @author Phil Schatzmann
 * @copyright GPLv3
 */
class BaseStream : public Stream {
 public:
  BaseStream() = default;
  virtual ~BaseStream() = default;
  BaseStream(BaseStream &) = default;
  BaseStream &operator=(BaseStream &) = default;

  virtual bool begin(){return true;}
  virtual void end(){}

  virtual size_t readBytes(uint8_t *data,
                           size_t len) STREAM_READ_OVERRIDE = 0;
  virtual size_t write(const uint8_t *data, size_t len) override = 0;

  virtual size_t write(uint8_t ch) override {
    tmp_out.resize(write_buffer_size);
    if (tmp_out.isFull()) {
      flush();
    }
    return tmp_out.write(ch);
  }

  virtual int available() override { return DEFAULT_BUFFER_SIZE; };

  virtual int availableForWrite() override { return DEFAULT_BUFFER_SIZE; }

  virtual void flush() override {
    if (tmp_out.available() > 0) {
      int len = tmp_out.available();
      uint8_t bytes[len];
      tmp_out.readArray((uint8_t *)bytes, len);
      write((const uint8_t *)bytes, len);
    }
  }


  virtual size_t readBytes(char *data, size_t len) STREAM_READCHAR_OVERRIDE {
    return readBytes((uint8_t *)data, len);
  }

  virtual int read() override {
    refillReadBuffer();
    // if it is empty we need to return an int -1
    if (tmp_in.isEmpty()) return -1;
    uint8_t result = 0;
    if (!tmp_in.read(result)) return -1;
    return result;
  }

  virtual int peek() override {
    refillReadBuffer();
    // if it is empty we need to return an int -1
    if (tmp_in.isEmpty()) return -1;
    uint8_t result = 0;
    if (!tmp_in.peek(result)) return -1;
    return result;
  }

  void setWriteBufferSize(int size) { write_buffer_size = size;}

 protected:
  RingBuffer<uint8_t> tmp_in{0};
  RingBuffer<uint8_t> tmp_out{0};
  int write_buffer_size = MAX_SINGLE_CHARS;

  void refillReadBuffer() {
    tmp_in.resize(DEFAULT_BUFFER_SIZE);
    if (tmp_in.isEmpty()) {
      const int len = tmp_in.size();
      uint8_t bytes[len];
      int len_eff = readBytes(bytes, len);
      // LOGD("tmp_in available: %d / size: %d / to be written %d",
      // tmp_in.available(), tmp_in.size(), len_eff);
      tmp_in.writeArray((const uint8_t*)bytes, len_eff);
    }
  }
};


}  // namespace tinyrobotics

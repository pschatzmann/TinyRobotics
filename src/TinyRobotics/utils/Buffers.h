#pragma once

#include <stdint.h>

#include <vector>

#include "TinyRobotics/utils/Config.h"
#include "TinyRobotics/utils/LoggerClass.h"

#ifndef LOGD
#define LOGD TRLogger.debug
#define LOGE TRLogger.error
#define LOGW TRLogger.warn
#define LOGI TRLogger.info
#define TRACED()
#endif

namespace tinyrobotics {

/**
 * @class BaseBuffer
 * @ingroup utils
 * @brief Shared functionality of all buffers
 * @author Phil Schatzmann
 */
template <typename T = int16_t>
class BaseBuffer {
 public:
  BaseBuffer() = default;
  virtual ~BaseBuffer() = default;
  BaseBuffer(BaseBuffer&) = default;
  BaseBuffer& operator=(BaseBuffer&) = default;

  /// reads a single value
  virtual bool read(T& result) = 0;

  /// reads multiple values
  virtual int readArray(T data[], int len) {
    if (data == nullptr) {
      LOGE("NPE");
      return 0;
    }
    int lenResult = min(len, available());
    for (int j = 0; j < lenResult; j++) {
      read(data[j]);
    }
    LOGD("readArray %d -> %d", len, lenResult);
    return lenResult;
  }

  /// Removes the next len entries
  virtual int clearArray(int len) {
    int lenResult = min(len, available());
    T dummy[lenResult];
    readArray(dummy, lenResult);
    return lenResult;
  }

  /// Fills the buffer data
  virtual int writeArray(const T data[], int len) {
    // LOGD("%s: %d", LOG_METHOD, len);
    // CHECK_MEMORY();

    int result = 0;
    for (int j = 0; j < len; j++) {
      if (!write(data[j])) {
        break;
      }
      result = j + 1;
    }
    // CHECK_MEMORY();
    LOGD("writeArray %d -> %d", len, result);
    return result;
  }

  /// Fills the buffer data and overwrites the oldest data if the buffer is full
  virtual int writeArrayOverwrite(const T data[], int len) {
    int to_delete = len - availableForWrite();
    if (to_delete > 0) {
      clearArray(to_delete);
    }
    return writeArray(data, len);
  }

  /// peeks the actual entry from the buffer
  virtual bool peek(T& result) = 0;

  /// checks if the buffer is full
  virtual bool isFull() { return availableForWrite() == 0; }

  bool isEmpty() { return available() == 0; }

  /// write add an entry to the buffer
  virtual bool write(T data) = 0;

  /// clears the buffer
  virtual void reset() = 0;

  ///  same as reset
  void clear() { reset(); }

  /// provides the number of entries that are available to read
  virtual int available() = 0;

  /// provides the number of entries that are available to write
  virtual int availableForWrite() = 0;

  /// returns the address of the start of the physical read buffer
  virtual T* address() = 0;

  virtual size_t size() = 0;

  /// Returns the level of the buffer in %
  virtual float levelPercent() {
    // prevent div by 0.
    if (size() == 0) return 0.0f;
    return 100.0f * static_cast<float>(available()) /
           static_cast<float>(size());
  }

  /// Resizes the buffer if supported: returns false if not supported
  virtual bool resize(int bytes) {
    LOGE("resize not implemented for this buffer");
    return false;
  }
};

/**
 * @brief A simple Buffer implementation which just uses a (dynamically sized)
 * array
 * @ingroup utils
 * @author Phil Schatzmann

 */

template <typename T = int16_t>
class SingleBuffer : public BaseBuffer<T> {
 public:
  /**
   * @brief Construct a new Single Buffer object
   *
   * @param size in entries
   */
  SingleBuffer(int size) {
    buffer.resize(size);
    reset();
  }

  SingleBuffer(SingleBuffer&) = default;
  SingleBuffer& operator=(SingleBuffer&) = default;

  /**
   * @brief Construct a new Single Buffer w/o allocating any memory
   */
  SingleBuffer() { reset(); }

  /// notifies that the external buffer has been refilled
  void onExternalBufferRefilled(void* data, int len) {
    this->owns_buffer = false;
    this->buffer = (uint8_t*)data;
    this->current_read_pos = 0;
    this->current_write_pos = len;
  }

  int writeArray(const T data[], int len) override {
    if (size() == 0) resize(len);
    return BaseBuffer<T>::writeArray(data, len);
  }

  bool write(T sample) override {
    bool result = false;
    if (current_write_pos < buffer.size()) {
      buffer[current_write_pos++] = sample;
      result = true;
    }
    return result;
  }

  bool read(T& result) override {
    bool success = false;
    if (current_read_pos < current_write_pos) {
      result = buffer[current_read_pos++];
      success = true;
    }
    return success;
  }

  bool peek(T& result) override {
    bool success = false;
    if (current_read_pos < current_write_pos) {
      result = buffer[current_read_pos];
      success = true;
    }
    return success;
  }

  int available() override {
    int result = current_write_pos - current_read_pos;
    return max(result, 0);
  }

  int availableForWrite() override { return buffer.size() - current_write_pos; }

  bool isFull() override { return availableForWrite() <= 0; }

  int peekArray(uint8_t* data, int len) {
    int len_available = available();
    if (len > len_available) {
      len = len_available;
    }
    memcpy(data, buffer.data() + current_read_pos, len);
    return len;
  }

  /// consumes len bytes and moves current data to the beginning
  int clearArray(int len) override {
    int len_available = available();
    if (len > available()) {
      reset();
      return len_available;
    }
    current_read_pos += len;
    len_available -= len;
    memmove(buffer.data(), buffer.data() + current_read_pos,
            len_available * sizeof(T));
    current_read_pos = 0;
    current_write_pos = len_available;

    if (is_clear_with_zero) {
      memset(buffer.data() + current_write_pos, 0,
             buffer.size() - current_write_pos);
    }

    return len;
  }

  /// Moves the unprocessed data to the beginning of the buffer
  void trim() {
    int av = available();
    memmove(buffer.data(), buffer.data() + current_read_pos, av * sizeof(T));
    current_write_pos = av;
    current_read_pos = 0;
  }

  /// Provides address to beginning of the buffer
  T* address() override { return buffer.data(); }

  /// Provides address of actual data
  T* data() { return buffer.data() + current_read_pos; }

  void reset() override {
    current_read_pos = 0;
    current_write_pos = 0;
    if (is_clear_with_zero) {
      memset(buffer.data(), 0, buffer.size());
    }
  }

  /// If we load values directly into the address we need to set the avialeble
  /// size
  size_t setAvailable(size_t available_size) {
    size_t result = min(available_size, (size_t)buffer.size());
    current_read_pos = 0;
    current_write_pos = result;
    return result;
  }

  size_t size() override { return buffer.size(); }

  bool resize(int size) {
    if (buffer.size() < size) {
      TRACED();
      buffer.resize(size);
    }
    return true;
  }

  /// Sets the buffer to 0 on clear
  void setClearWithZero(bool flag) { is_clear_with_zero = flag; }

  /// Updates the actual available data size
  void setWritePos(int pos) { current_write_pos = pos; }

  /// Optional ID
  int id = 0;
  /// Optional active/inactive status
  bool active = true;
  /// Optional timestamp
  uint64_t timestamp = 0;

 protected:
  int current_read_pos = 0;
  int current_write_pos = 0;
  bool owns_buffer = true;
  bool is_clear_with_zero = false;
  std::vector<T> buffer{0};
};

/**
 * @brief Implements a typed Ringbuffer
 * @ingroup utils
 * @tparam T
 */
template <typename T = int16_t>
class RingBuffer : public BaseBuffer<T> {
 public:
  RingBuffer(int size) {
    resize(size);
    reset();
  }

  bool read(T& result) override {
    if (isEmpty()) {
      return false;
    }

    result = _aucBuffer[_iTail];
    _iTail = nextIndex(_iTail);
    _numElems--;

    return true;
  }

  // peeks the actual entry from the buffer
  bool peek(T& result) override {
    if (isEmpty()) {
      return false;
    }

    result = _aucBuffer[_iTail];
    return true;
  }

  virtual int peekArray(T* data, int n) {
    if (isEmpty()) return -1;
    int result = 0;
    int count = _numElems;
    int tail = _iTail;
    for (int j = 0; j < n; j++) {
      data[j] = _aucBuffer[tail];
      tail = nextIndex(tail);
      count--;
      result++;
      if (count == 0) break;
    }
    return result;
  }

  // checks if the buffer is full
  virtual bool isFull() override { return available() == max_size; }

  bool isEmpty() { return available() == 0; }

  // write add an entry to the buffer
  virtual bool write(T data) override {
    bool result = false;
    if (!isFull()) {
      _aucBuffer[_iHead] = data;
      _iHead = nextIndex(_iHead);
      _numElems++;
      result = true;
    }
    return result;
  }

  // clears the buffer
  virtual void reset() override {
    _iHead = 0;
    _iTail = 0;
    _numElems = 0;
  }

  // provides the number of entries that are available to read
  virtual int available() override { return _numElems; }

  // provides the number of entries that are available to write
  virtual int availableForWrite() override { return (max_size - _numElems); }

  // returns the address of the start of the physical read buffer
  virtual T* address() override { return _aucBuffer.data(); }

  virtual bool resize(int len) {
    if (max_size != len && len > 0) {
      LOGI("resize: %d", len);
      _aucBuffer.resize(len);
      max_size = len;
    }
    return true;
  }

  /// Returns the maximum capacity of the buffer
  virtual size_t size() override { return max_size; }

 protected:
  std::vector<T> _aucBuffer;
  int _iHead;
  int _iTail;
  int _numElems;
  int max_size = 0;

  int nextIndex(int index) {
    if (max_size == 0) return 0;
    return (uint32_t)(index + 1) % max_size;
  }
};

}  // namespace tinyrobotics
#pragma once

#if __cplusplus >= 201703L

#include <optional>

#else

#include <type_traits>
#include <utility>
#include <stdexcept>

namespace std {

/// nullopt_t and nullopt definition
struct nullopt_t {
  explicit constexpr nullopt_t(int) {}
};
constexpr nullopt_t nullopt{0};

/**
 * @class optional
 * @brief Minimal header-only polyfill for std::optional for pre-C++17 environments.
 *
 * This implementation provides the basic interface of std::optional, allowing code to use
 * std::optional<T> and std::nullopt in environments where the standard library version is not available.
 *
 * Features:
 *  - Value storage and lifetime management
 *  - Copy/move construction and assignment
 *  - has_value(), value(), value_or(), reset(), and conversion to bool
 *  - Construction and assignment from std::nullopt
 *
 * Limitations:
 *  - No emplace or in-place construction
 *  - No support for references or advanced C++17 features
 *
 * Usage:
 *  - Use as a drop-in replacement for std::optional<T> in C++11/C++14 code
 *  - When C++17 or later is available, the standard library version is used
 */
template <typename T>
class optional {
  bool has_value_ = false;
  alignas(T) unsigned char storage_[sizeof(T)];

 public:
  optional() noexcept = default;
  optional(nullopt_t) noexcept {}
  optional(const T& value) { new (storage_) T(value); has_value_ = true; }
  optional(T&& value) { new (storage_) T(std::move(value)); has_value_ = true; }
  optional(const optional& other) {
    if (other.has_value_) {
      new (storage_) T(other.value());
      has_value_ = true;
    }
  }
  optional(optional&& other) noexcept {
    if (other.has_value_) {
      new (storage_) T(std::move(other.value()));
      has_value_ = true;
      other.reset();
    }
  }
  ~optional() { reset(); }

  optional& operator=(nullopt_t) noexcept {
    reset();
    return *this;
  }
  optional& operator=(const optional& other) {
    if (this != &other) {
      reset();
      if (other.has_value_) {
        new (storage_) T(other.value());
        has_value_ = true;
      }
    }
    return *this;
  }
  optional& operator=(optional&& other) noexcept {
    if (this != &other) {
      reset();
      if (other.has_value_) {
        new (storage_) T(std::move(other.value()));
        has_value_ = true;
        other.reset();
      }
    }
    return *this;
  }

  bool has_value() const noexcept { return has_value_; }
  explicit operator bool() const noexcept { return has_value_; }

  T& value() {
    if (!has_value_) throw std::runtime_error("Bad optional access");
    return *reinterpret_cast<T*>(storage_);
  }
  const T& value() const {
    if (!has_value_) throw std::runtime_error("Bad optional access");
    return *reinterpret_cast<const T*>(storage_);
  }
  T value_or(const T& default_value) const {
    return has_value_ ? value() : default_value;
  }
  void reset() {
    if (has_value_) {
      reinterpret_cast<T*>(storage_)->~T();
      has_value_ = false;
    }
  }
};

} // namespace std

#endif

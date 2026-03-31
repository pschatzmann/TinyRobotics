#pragma once
#include <array>
#include <cstddef>
#include <stdexcept>

namespace tinyrobotics {

/**
 * @brief A simple, fixed-capacity vector backed by std::array for embedded and
 * performance-critical use.
 *
 * VectorFromArray provides a subset of the std::vector API, but with no dynamic
 * memory allocation. All storage is allocated at compile time, making it
 * suitable for microcontrollers, real-time systems, or any environment where
 * heap usage is undesirable or unavailable.
 *
 * Features:
 *   - push_back() to add elements up to the fixed capacity N
 *   - operator[] for element access (no bounds checking)
 *   - size() and capacity() for querying current and maximum size
 *   - clear() to reset the vector to empty
 *   - begin()/end() iterators for range-based for loops
 *
 * Limitations:
 *   - Capacity is fixed at compile time (template parameter N)
 *   - Exceeding capacity throws std::out_of_range (can be changed for embedded
 * use)
 *   - No erase, insert, or dynamic resizing
 *
 * @ingroup utils
 * 
 * Example usage:
 * @code
 * VectorFromArray<int, 8> v;
 * v.push_back(1);
 * v.push_back(2);
 * for (auto x : v) {
 *   Serial.println(x);
 * }
 * v.clear();
 * @endcode
 *
 * @tparam T Element type
 * @tparam N Maximum number of elements (capacity)
 */
template <typename T, std::size_t N>
class VectorFromArray {
 public:
  VectorFromArray() : sz(0) {}

  bool push_back(const T& value) {
    if (sz >= N) return false;
    data[sz++] = value;
    return true;
  }

  void clear() { sz = 0; }

  std::size_t size() const { return sz; }
  constexpr std::size_t capacity() const { return N; }

  T& operator[](std::size_t idx) { return data[idx]; }
  const T& operator[](std::size_t idx) const { return data[idx]; }

  T* begin() { return data.begin(); }
  T* end() { return data.begin() + sz; }
  const T* begin() const { return data.begin(); }
  const T* end() const { return data.begin() + sz; }

 protected:
  std::array<T, N> data;
  std::size_t sz;
};

}  // namespace tinyrobotics

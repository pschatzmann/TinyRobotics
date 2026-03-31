#pragma once


#include "TinyRobotics/utils/AllocatorPSRAM.h"
#include "TinyRobotics/concurrency/LockGuard.h"
#include <list>

namespace tinyrobotics {

/**
 * @brief FIFO Queue which is based on a List that is thread 
 * save.
 * @ingroup concurrency
 * @author Phil Schatzmann

 * @tparam T
 * @tparam TMutex
 * @tparam TAllocator Allocator type for the underlying list (default: AllocatorPSRAM<T>)
 */
template <class T, class TMutex, class TAllocator = AllocatorPSRAM<T>>
class SynchronizedQueue {
 public:
  SynchronizedQueue() = default;

  bool enqueue(T& data) {
    LockGuard guard{mutex};;
    return l.push_front(data);
  }

  bool peek(T& data) {
    LockGuard guard{mutex};;
    if (l.end()->prior == nullptr) return false;
    data = *(l.end()->prior);
    return true;
  }

  bool dequeue(T& data) {
    LockGuard guard{mutex};;
    return l.pop_back(data);
  }

  size_t size() {
    LockGuard guard{mutex};;
    return l.size();
  }

  bool clear() {
    LockGuard guard{mutex};;
    return l.clear();
  }

  bool empty() {
    LockGuard guard{mutex};;
    return l.empty();
  }

 protected:
  std::list<T, TAllocator> l;
  TMutex mutex;
};

}  // namespace tinyrobotics
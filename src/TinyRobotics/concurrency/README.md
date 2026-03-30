# Concurrency Module

The Concurrency module in TinyRobotics provides thread-safe data structures and synchronization primitives for multitasking and real-time applications. It is designed to work seamlessly with RTOS environments (such as FreeRTOS) and supports both embedded and desktop platforms.

## Features

- **Thread-Safe Queues:**
  - `SynchronizedQueue`: A mutex-protected FIFO queue for safe data exchange between threads or tasks.
  - `QueueRTOS`: A queue implementation based on the FreeRTOS queue API, supporting real-time task communication.

- **Thread-Safe Buffers:**
  - `SynchronizedBuffer`: A mutex-protected buffer for safe, concurrent access.
  - `BufferRTOS`: A buffer implementation using FreeRTOS stream buffers for efficient, real-time data streaming.

- **Mutexes and Locks:**
  - `Mutex`, `LockGuard`: Abstractions for mutual exclusion and scoped locking, compatible with both RTOS and desktop environments.

- **RTOS Integration:**
  - Classes and utilities designed to leverage FreeRTOS primitives when available, with fallbacks for non-RTOS systems.


## Files

- `SynchronizedQueue.h` — Mutex-protected FIFO queue
- `SynchronizedBuffer.h` — Mutex-protected buffer
- `Mutex.h`, `LockGuard.h` — Mutex and lock abstractions
- `RTOS/QueueRTOS.h` — FreeRTOS-based queue
- `RTOS/Task.h` — Abstraction for FreeRTOS task/thread management
- `RTOS/BufferRTOS.h` — FreeRTOS-based buffer

## When to Use

- For safe data exchange between tasks or threads in multitasking environments
- When integrating with FreeRTOS or other RTOS platforms
- For embedded systems requiring robust, thread-safe communication

## Example Usage

```cpp
#include "concurrency/RTOS.h"

// RTOS example 
QueueRTOS<int> rtosQueue(50);

void source(void* param) {
    static int value = 1;
    rtosQueue.enqueue(value++);
}

void sink(void* param){
    int value
    if (rtosQueue.dequeue(value)){
        Serial.println(value);
    }
}

Task sourceTask(source, "Source", 2048, nullptr, 1);
Task sinkTask(sink,"Sink", 2048, nullptr, 1);
sourceTask.start();
sinkTask.start();

```



## See Examples

- [Task Concurrency Example](../../../examples/others/task/task.ino)

## See Also

- [FreeRTOS Documentation](https://www.freertos.org/)

---

© Phil Schatzmann, GPLv3

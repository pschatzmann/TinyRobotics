#pragma once
#include "Print.h"
#include <cstdint>
#include <cstddef>

namespace tinyrobotics {
/**
 * @brief NullPrint is a Print subclass that discards all output.
 * Useful for disabling output in code that expects a Print interface.
 */
class NullPrintClass : public Print {
 public:
	size_t write(uint8_t) override { return 1; }
	size_t write(uint8_t* buf, size_t, size_t len)  { return len; }
	size_t write(const uint8_t* buf, size_t len)  { return len; }
};

static NullPrintClass NullPrint;

} // namespace tinyrobotics


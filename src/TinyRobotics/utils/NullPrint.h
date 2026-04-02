#pragma once
#include "../arduino/Print.h"

namespace tinyrobotics_arduino {
/**
 * @brief NullPrint is a Print subclass that discards all output.
 * Useful for disabling output in code that expects a Print interface.
 */
class NullPrintClass : public Print {
 public:
	size_t write(uint8_t) override { return 1; }
	size_t write(uint8_t* buf, size_t, size_t len) override { return len; }
	size_t write(const uint8_t* buf, size_t len) override { return len; }
};

static NullPrintClass NullPrint;

} // namespace tinyrobotics_arduino

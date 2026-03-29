#pragma once
#include "Message.h"
#include "Stream.h"
#include "TinyRobotics/utils/LoggerClass.h"
#include "TinyRobotics/vehicles/Vehicle.h"

namespace tinyrobotics {

/**
 * @brief Reads stream and forwards messages to handler.
 *
 * The MessageDispatcher class reads messages from a communication stream
 * (such as Serial, UDP, etc.) and dispatches them to the MessageHandler
 * instance for processing. It handles message framing, validation, and error
 * logging, providing a unified interface for remote control and telemetry.
 *
 * Usage Example:
 * @code
 * MessageDispatcher commMgr(handler, Serial);
 * while (true) {
 *   commMgr.run();
 * }
 * @endcode
 */

class MessageDispatcher {
 public:
  MessageDispatcher(MessageHandler& handler, Stream& io) {
    p_handler = &handler;
    p_stream = &io;
  };

  bool begin() {
    if (p_stream == nullptr || p_handler == nullptr) {
      TRLogger.error("CommMgr: Invalid stream or handler");
      return false;
    }
    is_active = true;
    return true;
  }

  void end() { is_active = false; }

  /// Read messages from the stream and dispatch them to the vehicle for
  /// processing.
  virtual bool run() {
    if (!is_active) return false;
    if (p_stream->available() >= sizeof(Message<float>)) {
      // Read a message from the stream and process it
      Message<float> msg;
      size_t read = p_stream->readBytes((uint8_t*)&msg, sizeof(msg));
      if (read != sizeof(msg)) {
        // Handle invalid message size (e.g., log an error)
        TRLogger.error(
            "CommMgr: Incomplete message received (expected %d bytes, got %d)",
            sizeof(msg), read);
        return false;
      }
      if (memcmp(msg.prefix, "MSG", 3) != 0) {
        // Handle invalid message prefix (e.g., log an error)
        TRLogger.error("CommMgr: Invalid message prefix: %.*s", 3, msg.prefix);
        return false;
      }
      if (!p_handler->onMessage(msg)) {
        // Handle unprocessed message (e.g., log a warning)
        TRLogger.warn("CommMgr: Unhandled message content: %d",
                      static_cast<int>(msg.content));
        return false;
      }
    }
    return true;
  }

 protected:
  Stream* p_stream = nullptr;
  bool is_active = false;
  MessageHandler* p_handler = nullptr;
};

}  // namespace tinyrobotics
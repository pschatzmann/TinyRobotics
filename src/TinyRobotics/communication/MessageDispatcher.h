#pragma once
#include "Message.h"
#include "MessageHandler.h"
#include "MessageParser.h"
#include "Stream.h"
#include "TinyRobotics/utils/LoggerClass.h"
#include "TinyRobotics/vehicles/Vehicle.h"

namespace tinyrobotics {

/**
 * @class MessageDispatcher
 * @ingroup communication
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
  MessageDispatcher() = default;
  MessageDispatcher(MessageHandler& handler) { p_handler = &handler; }
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
    if (!p_stream || !p_handler) return false;
    MessageParser parser;
    // Try to parse and dispatch a message from the stream
    if (!parser.parse(*p_stream, *p_handler)) {
      // Optionally log error or warning here
      TRLogger.error("CommMgr: Failed to parse message");
      return false;
    }
    return true;
  }

  void setStream(Stream& io) { p_stream = &io; }
  void setHandler(MessageHandler& handler) { p_handler = &handler; }

 protected:
  Stream* p_stream = nullptr;
  bool is_active = false;
  MessageHandler* p_handler = nullptr;
};

}  // namespace tinyrobotics
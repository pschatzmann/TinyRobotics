#pragma once
#include "Message.h"
#include "TinyRobotics/utils/LoggerClass.h"
#include "TinyRobotics/vehicles/Vehicle.h"

namespace tinyrobotics {

/**
 * @class MessageBus
 * @brief A message handler that forwards messages to multiple registered
 * handlers.
 *
 * The MessageBus class implements the MessageHandler interface and allows you
 * to broadcast messages to multiple other MessageHandler instances. It is
 * useful for publisher-subscriber or observer patterns, where a single message
 * source needs to notify multiple listeners or output streams.
 *
 * Usage:
 *   - Register handlers using add(MessageHandler&)
 *   - When a message is received, it is forwarded to all registered handlers
 *
 * Supported message types:
 *   - Message<float>
 *   - Message<Coordinate<float>>
 *   - Message<GPSCoordinate> (default: warning, not handled)
 */

class MessageBus : public MessageHandler {
 public:
  /// Default constructor
  MessageBus() = default;

  /// Add a MessageHandler to the bus. All messages received by the bus will be
  /// forwarded to this handler.
  void add(MessageHandler& stream) { p_handlers.push_back(&stream); }

  /// Forward incoming messages to all registered handlers.
  bool onMessage(const Message<float>& msg) override {
    for (auto& stream : p_handlers) {
      stream->onMessage(msg);
    }
    return true;
  }

  /// Forward incoming messages to all registered handlers.
  bool onMessage(const Message<Coordinate<float>>& msg) override {
    for (auto& stream : p_handlers) {
      stream->onMessage(msg);
    }
    return true;
  };

  /// Forward incoming messages to all registered handlers.
  bool onMessage(const Message<GPSCoordinate>& msg) override {
    for (auto& stream : p_handlers) {
      stream->onMessage(msg);
    }
    return true;
  };

  /// Removes all registere handlers from the bus.
  void clear() {
    p_handlers.clear();
  }

   /// Get the number of registered handlers

 protected:
  std::vector<MessageHandler*> p_handlers;
};

}  // namespace tinyrobotics
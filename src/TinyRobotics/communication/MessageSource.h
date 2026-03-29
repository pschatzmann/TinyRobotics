#pragma once
#include <vector>

#include "Message.h"
#include "MessageHandler.h"
#include "TinyRobotics/coordinates/Coordinates.h"
#include "TinyRobotics/coordinates/GPSCoordinate.h"

namespace tinyrobotics {

/**
 * @brief Base class for message sources in the TinyRobotics communication
 * framework.
 *
 * MessageSource manages a list of MessageHandler objects and provides methods
 * to add, clear, and forward messages to all registered handlers. This enables
 * a flexible publish/subscribe or observer pattern for distributing messages
 * (such as sensor data, commands, or events) to multiple consumers in a
 * robotics system.
 *
 * Typical usage:
 * @code
 *   MessageSource source;
 *   source.subscribe(handler1);
 *   source.subscribe(handler2);
 *   source.sendMessage(msg); // Forwards to all handlers
 * @endcode
 *
 * Extend this class to implement custom message-producing components (e.g.,
 * sensors, controllers) that need to broadcast messages to other system parts.
 */
class MessageSource {
 public:
  /**
   * @brief Add a message handler to the chain.
   *
   * Allows forwarding messages to additional handlers.
   * @param handler Reference to a MessageHandler to add.
   */
  void subscribe(MessageHandler& handler) {
    messageHandlers_.push_back(&handler);
  }

  /**
   * @brief Remove all registered message handlers.
   */
  void unsubscribeAll() { messageHandlers_.clear(); }

  /**
   * @brief Publish a message to all registered handlers.
   *
   * Forwards the given message to each handler in the messageHandlers_ list.
   * @param msg The message to publish.
   */
  void sendMessage(Message<float>& msg) {
    for (auto handler : messageHandlers_) {
      handler->onMessage(msg);
    }
  }

  void sendMessage(const Message<Coordinate<float>>& msg) {
    for (auto handler : messageHandlers_) {
      handler->onMessage(msg);
    }
  }

  void sendMessage(const Message<GPSCoordinate>& msg) {
    for (auto handler : messageHandlers_) {
      handler->onMessage(msg);
    }
  }

 protected:
  std::vector<MessageHandler*> messageHandlers_;
};

}  // namespace tinyrobotics

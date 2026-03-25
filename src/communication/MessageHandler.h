#pragma once
#include <vector>
#include "Message.h"
#include "coordinates/Coordinates.h"
#include "coordinates/GPSCoordinate.h"

namespace tinyrobotics {

/**
 * @brief Interface for handling messages in the TinyRobotics framework.
 *
 * MessageHandler provides a base interface for classes that process messages.
 * It also supports chaining multiple handlers for flexible message routing.
 */
class MessageHandler {
 public:
  /**
   * @brief Handle an incoming message (pure virtual).
   *
   * This method should be implemented by derived classes to process messages.
   * @param msg The message to handle.
   * @return true if the message was handled successfully, false otherwise.
   */
  virtual bool onMessage(const Message<float>& msg) = 0;

  virtual bool onMessage(const Message<Coordinate<float>>& msg) {
    TRLogger.warn("MessageHandler: Received unhandled Coordinate message");
    return false;  // Default implementation does not handle Coordinate messages
  };
  virtual bool onMessage(const Message<GPSCoordinate>& msg) {
    TRLogger.warn("MessageHandler: Received unhandled GPSCoordinate message");
    return false;  // Default implementation does not handle Coordinate messages
  };
};

class MessageSource {
 public:
  /**
   * @brief Add a message handler to the chain.
   *
   * Allows forwarding messages to additional handlers.
   * @param handler Reference to a MessageHandler to add.
   */
  void addMessageHandler(MessageHandler& handler) {
    messageHandlers_.push_back(&handler);
  }

  /**
   * @brief Remove all registered message handlers.
   */
  void clearMessageHandlers() { messageHandlers_.clear(); }

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
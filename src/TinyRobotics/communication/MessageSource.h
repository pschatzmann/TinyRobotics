#pragma once
#include <vector>

#include "Message.h"
#include "MessageHandler.h"
#include "TinyRobotics/coordinates/Coordinates.h"
#include "TinyRobotics/coordinates/GPSCoordinate.h"

namespace tinyrobotics {

/**
 * @class MessageSource
 * @ingroup communication
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
   * @brief Subscribe a message handler to this source, with optional filtering.
   *
   * Registers a MessageHandler to receive messages published by this source. Optionally, you can specify
   * a filter for message origin and content. Only messages matching the filter (or with filter set to Undefined)
   * will be delivered to the handler.
   *
   * @param handler Reference to a MessageHandler to add.
   * @param origin  (Optional) Only deliver messages with this origin. Use MessageOrigin::Undefined to accept all origins.
   * @param content (Optional) Only deliver messages with this content type. Use MessageContent::Undefined to accept all content types.
   *
   * Example:
   * @code
   *   source.subscribe(handler); // Receives all messages
   *   source.subscribe(handler, MessageOrigin::RemoteControl); // Only messages from remote control
   *   source.subscribe(handler, MessageOrigin::Undefined, MessageContent::Throttle); // Only throttle messages
   * @endcode
   */
  void subscribe(MessageHandler& handler, MessageOrigin origin = MessageOrigin::Undefined,
                 MessageContent content = MessageContent::Undefined) {
    MessageHandlerEntry entry{handler, origin, content};               
    messageHandlers_.push_back(entry);
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
    for (auto& entry : messageHandlers_) {
      if (!entry.handler) continue;
      if ((entry.origin == MessageOrigin::Undefined || entry.origin == msg.origin) &&
          (entry.content == MessageContent::Undefined || entry.content == msg.content)) {
        entry.handler->onMessage(msg);
      }
    }
  }

  /**
   * @brief Publish a message to all registered handlers.
   *
   * Forwards the given message to each handler in the messageHandlers_ list.
   * @param msg The message to publish.
   */
  void sendMessage(const Message<Coordinate<float>>& msg) {
    for (auto& entry : messageHandlers_) {
      if (!entry.handler) continue;
      if ((entry.origin == MessageOrigin::Undefined || entry.origin == msg.origin) &&
          (entry.content == MessageContent::Undefined || entry.content == msg.content)) {
        entry.handler->onMessage(msg);
      }
    }
  }

  /**
   * @brief Publish a message to all registered handlers.
   *
   * Forwards the given message to each handler in the messageHandlers_ list.
   * @param msg The message to publish.
   */
  void sendMessage(const Message<GPSCoordinate>& msg) {
    for (auto& entry : messageHandlers_) {
      if (!entry.handler) continue;
      if ((entry.origin == MessageOrigin::Undefined || entry.origin == msg.origin) &&
          (entry.content == MessageContent::Undefined || entry.content == msg.content)) {
        entry.handler->onMessage(msg);
      }
    }
  }

protected:
  struct MessageHandlerEntry {
    MessageHandler* handler = nullptr;
    MessageOrigin origin = MessageOrigin::Undefined;  // origin filter
    MessageContent content = MessageContent::Undefined; // content filter

    MessageHandlerEntry() = default;
    MessageHandlerEntry(MessageHandler& h, MessageOrigin o = MessageOrigin::Undefined, MessageContent c = MessageContent::Undefined)
      : handler(&h), origin(o), content(c) {}

    bool operator==(const MessageHandlerEntry& other) const {
      return handler == other.handler && origin == other.origin && content == other.content;
    }
  };
  std::vector<MessageHandlerEntry> messageHandlers_;
};

}  // namespace tinyrobotics

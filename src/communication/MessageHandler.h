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

}  // namespace tinyrobotics
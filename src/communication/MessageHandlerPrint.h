#pragma once
#include "MessageHandler.h"
#include "Print.h"

namespace tinyrobotics {

/**
 * @brief Message handler that prints all received messages to a Print stream.
 *
 * This class implements the MessageHandler interface and outputs the content of
 * all received messages (float, Coordinate<float>, GPSCoordinate) in a
 * human-readable format to the provided Print object (e.g., Serial, file, etc).
 * It uses string arrays to print the message type, unit, and source as readable
 * text.
 *
 * Example usage:
 * @code
 *   MessageHandlerPrint printer(Serial);
 *   source.addMessageHandler(printer);
 * @endcode
 *
 * This is useful for debugging, logging, or monitoring message traffic in a
 * robotics system.
 */
class MessageHandlerPrint : public MessageHandler {
 public:
  MessageHandlerPrint(Print& printer) : printer_(printer) {}
  bool onMessage(const Message<float>& msg) override {
    printer_.print("[Message] Type: ");
    printer_.print(messageTypeStr[static_cast<int>(msg.content)]);
    printer_.print(", Value: ");
    printer_.print(msg.value);
    printer_.print(", Unit: ");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print(", Source: ");
    printer_.println(sourceStr[static_cast<int>(msg.source)]);
    return true;  // Indicate that the message was handled
  }

  bool onMessage(const Message<Coordinate<float>>& msg) override {
    printer_.print("[Message] Type: ");
    printer_.print(messageTypeStr[static_cast<int>(msg.content)]);
    printer_.print(", X: ");
    printer_.print(msg.value.x);
    printer_.print(", Y: ");
    printer_.print(msg.value.y);
    printer_.print(", Unit: ");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print(", Source: ");
    printer_.println(sourceStr[static_cast<int>(msg.source)]);
    return true;  // Indicate that the message was handled
  }

  bool onMessage(const Message<GPSCoordinate>& msg) override {
    printer_.print("[Message] Type: ");
    printer_.print(messageTypeStr[static_cast<int>(msg.content)]);
    printer_.print(", Lat: ");
    printer_.print(msg.value.latitude);
    printer_.print(", Lon: ");
    printer_.print(msg.value.longitude);
    printer_.print(", Alt: ");
    printer_.print(msg.value.altitude);
    printer_.print(", Unit: ");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print(", Source: ");
    printer_.println(sourceStr[static_cast<int>(msg.source)]);
    return true;  // Indicate that the message was handled
  }

 protected:
  Print& printer_;

  // String arrays for enums
  static constexpr const char* messageTypeStr[] = {"Unknown", "Speed", "Heading", "Position", "PositionGPS"};
  static constexpr const char* unitStr[] = {"None", "Meters", "MetersPerSecond", "AngleRadian", "AngleDegree"};
  static constexpr const char* sourceStr[] = {"Unknown", "IMU", "GPS", "Controller", "User", "Other"};
};

}  // namespace tinyrobotics
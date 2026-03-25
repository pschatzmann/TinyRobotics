#pragma once
#include "MessageHandler.h"
#include "Print.h"

namespace tinyrobotics {

// String arrays for enums (shared by all handlers)
constexpr const char* messageTypeStr[] = {"Unknown", "Speed", "Heading",
                                          "Position", "PositionGPS"};
constexpr const char* unitStr[] = {"None", "Meters", "MetersPerSecond",
                                   "AngleRadian", "AngleDegree"};
constexpr const char* sourceStr[] = {"Unknown",    "IMU",  "GPS",
                                     "Controller", "User", "Other"};

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
};

/**
 * @brief Message handler that prints all received messages as XML to a Print
 * stream.
 *
 * This class implements the MessageHandler interface and outputs the content of
 * all received messages (float, Coordinate<float>, GPSCoordinate) in XML format
 * to the provided Print object (e.g., Serial, file, etc). It uses the same
 * string arrays as MessageHandlerPrint to print the message type, unit, and
 * source as readable text.
 *
 * Example usage:
 * @code
 *   MessageHandlerPrintXML printer(Serial);
 *   source.addMessageHandler(printer);
 * @endcode
 *
 * This is useful for logging, exporting, or integrating message traffic with
 * systems that consume XML data.
 */
class MessageHandlerPrintXML : public MessageHandler {
 public:
  MessageHandlerPrintXML(Print& printer) : printer_(printer) {}

  bool onMessage(const Message<float>& msg) override {
    printer_.print("<Message type=\"");
    printer_.print(messageTypeStr[static_cast<int>(msg.content)]);
    printer_.print("\" unit=\"");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print("\" source=\"");
    printer_.print(sourceStr[static_cast<int>(msg.source)]);
    printer_.print("\">");
    printer_.print(msg.value);
    printer_.println("</Message>");
    return true;
  }

  bool onMessage(const Message<Coordinate<float>>& msg) override {
    printer_.print("<Message type=\"");
    printer_.print(messageTypeStr[static_cast<int>(msg.content)]);
    printer_.print("\" unit=\"");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print("\" source=\"");
    printer_.print(sourceStr[static_cast<int>(msg.source)]);
    printer_.print("\"><X>");
    printer_.print(msg.value.x);
    printer_.print("</X><Y>");
    printer_.print(msg.value.y);
    printer_.println("</Y></Message>");
    return true;
  }

  bool onMessage(const Message<GPSCoordinate>& msg) override {
    printer_.print("<Message type=\"");
    printer_.print(messageTypeStr[static_cast<int>(msg.content)]);
    printer_.print("\" unit=\"");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print("\" source=\"");
    printer_.print(sourceStr[static_cast<int>(msg.source)]);
    printer_.print("\"><Lat>");
    printer_.print(msg.value.latitude);
    printer_.print("</Lat><Lon>");
    printer_.print(msg.value.longitude);
    printer_.print("</Lon><Alt>");
    printer_.print(msg.value.altitude);
    printer_.println("</Alt></Message>");
    return true;
  }

 protected:
  Print& printer_;
};

/**
 * @brief Message handler that prints all received messages as JSON to a Print
 * stream.
 *
 * This class implements the MessageHandler interface and outputs the content of
 * all received messages (float, Coordinate<float>, GPSCoordinate) in JSON
 * format to the provided Print object (e.g., Serial, file, etc). It uses the
 * shared string arrays to print the message type, unit, and source as readable
 * text.
 *
 * Example usage:
 * @code
 *   MessageHandlerPrintJSON printer(Serial);
 *   source.addMessageHandler(printer);
 * @endcode
 *
 * This is useful for logging, exporting, or integrating message traffic with
 * systems that consume JSON data.
 */
class MessageHandlerPrintJSON : public MessageHandler {
 public:
  MessageHandlerPrintJSON(Print& printer) : printer_(printer) {}

  bool onMessage(const Message<float>& msg) override {
    printer_.print("{\"type\":\"");
    printer_.print(messageTypeStr[static_cast<int>(msg.content)]);
    printer_.print("\",\"value\":");
    printer_.print(msg.value);
    printer_.print(",\"unit\":\"");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print("\",\"source\":\"");
    printer_.print(sourceStr[static_cast<int>(msg.source)]);
    printer_.println("\"}");
    return true;
  }

  bool onMessage(const Message<Coordinate<float>>& msg) override {
    printer_.print("{\"type\":\"");
    printer_.print(messageTypeStr[static_cast<int>(msg.content)]);
    printer_.print("\",\"x\":");
    printer_.print(msg.value.x);
    printer_.print(",\"y\":");
    printer_.print(msg.value.y);
    printer_.print(",\"unit\":\"");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print("\",\"source\":\"");
    printer_.print(sourceStr[static_cast<int>(msg.source)]);
    printer_.println("\"}");
    return true;
  }

  bool onMessage(const Message<GPSCoordinate>& msg) override {
    printer_.print("{\"type\":\"");
    printer_.print(messageTypeStr[static_cast<int>(msg.content)]);
    printer_.print("\",\"lat\":");
    printer_.print(msg.value.latitude);
    printer_.print(",\"lon\":");
    printer_.print(msg.value.longitude);
    printer_.print(",\"alt\":");
    printer_.print(msg.value.altitude);
    printer_.print(",\"unit\":\"");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print("\",\"source\":\"");
    printer_.print(sourceStr[static_cast<int>(msg.source)]);
    printer_.println("\"}");
    return true;
  }

 protected:
  Print& printer_;
};

}  // namespace tinyrobotics
#pragma once
#include "MessageHandler.h"
#include "Print.h"

namespace tinyrobotics {

// from enum class MessageContent in Message.h - update this array to match all
// values in the MessageContent enum String arrays for enums (shared by all
// handlers) Update this array to match all values in the MessageContent enum
// (see Message.h)
constexpr const char* messageContentStr[] = {
    "Undefined",      // 0
    "Angle",          // 1
    "Pitch",          // 2
    "Roll",           // 3
    "Yaw",            // 4
    "Throttle",       // 5
    "Speed",          // 6
    "SteeringAngle",  // 7
    "Turn",           // 8
    "Heading",        // 9
    "MotorSpeed",     // 10
    "Position",       // 11
    "PositionGPS",    // 12
    "Distance",       // 13
    "Temperature",    // 14
    "Error",          // 15
    "Density"         // 16
    "MotionState"      // 17
};

// from enum class Unit in Common.h - update this array to match all values in
// the Unit enum
constexpr const char* unitStr[] = {
    "Undefined",         // 0
    "Percent",           // 1
    "MetersPerSecond",   // 2
    "RadiansPerSecond",  // 3
    "Meters",            // 4
    "Centimeters",       // 5
    "Millimeters",       // 5
    "AngleDegree",       // 6
    "AngleRadian",       // 7
    "TemperatureC",      // 8
    "TemperatureF",      // 9
    "Pixel"              // 10
};

// from enum class MessageOrigin in Message.h - update this array to match all
// values in the MessageOrigin enum
constexpr const char* originStr[] = {
  "Undefined",      // 0
  "RemoteControl",  // 1
  "Autonomy",       // 2
  "Sensor",         // 3
  "System",         // 4
  "Motor",          // 5
  "Servo",          // 6
  "Rudder",         // 7
  "Aileron",        // 8
  "Elevator",       // 9
  "IMU",            // 10
  "LIDAR",          // 11
  "Camera",         // 12
  "GPS",            // 13
  "Vehicle",        // 14
  "Odometry",       // 15
  "Navigation",     // 16
  "User"            // 17
};

/**
 * @brief Message handler that writes all received messages as raw binary to a
 * Print stream.
 *
 * This class implements the MessageHandler interface and outputs the content of
 * all received messages (float, Coordinate<float>, GPSCoordinate) as raw binary
 * data to the provided Print object (e.g., Serial, file, etc).
 *
 * Example usage:
 * @code
 *   MessageHandlerBinary printer(Serial);
 *   source.subscribe(printer);
 * @endcode
 *
 * This is useful for efficient logging, binary protocols, or communication with
 * systems that expect binary-encoded messages.
 * @ingroup communication
 */
class MessageHandlerBinary : public MessageHandler {
 public:
  MessageHandlerBinary() : printer_(*static_cast<Print*>(nullptr)) {}
  MessageHandlerBinary(Print& printer) : printer_(printer) {}
  bool onMessage(const Message<float>& msg) override {
    size_t written = printer_.write((const uint8_t*)&msg, sizeof(msg));
    return written == sizeof(msg);
  }

  bool onMessage(const Message<Coordinate<float>>& msg) override {
    size_t written = printer_.write((const uint8_t*)&msg, sizeof(msg));
    return written == sizeof(msg);
  }

  bool onMessage(const Message<GPSCoordinate>& msg) override {
    size_t written = printer_.write((const uint8_t*)&msg, sizeof(msg));
    return written == sizeof(msg);
  }

  bool onMessage(const Message<MotionState3D>& msg) override {
    size_t written = printer_.write((const uint8_t*)&msg, sizeof(msg));
    return written == sizeof(msg);
  }

  void setOutput(Print& printer) { printer_ = printer; }

 protected:
  Print& printer_;
};

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
 *   source.subscribe(printer);
 * @endcode
 *
 * This is useful for debugging, logging, or monitoring message traffic in a
 * robotics system.
 * @ingroup communication
 */
class MessageHandlerPrint : public MessageHandler {
 public:
  MessageHandlerPrint() : printer_(*static_cast<Print*>(nullptr)) {}
  MessageHandlerPrint(Print& printer) : printer_(printer) {}
  void setOutput(Print& printer) { printer_ = printer; }
  bool onMessage(const Message<float>& msg) override {
    printer_.print("[Message] Type: ");
    printer_.print(messageContentStr[static_cast<int>(msg.content)]);
    printer_.print(", Value: ");
    printer_.print(msg.value);
    printer_.print(" ");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print(", Source: ");
    printer_.println(originStr[static_cast<int>(msg.origin)]);
    return true;  // Indicate that the message was handled
  }

  bool onMessage(const Message<Coordinate<float>>& msg) override {
    printer_.print("[Message] Type: ");
    printer_.print(messageContentStr[static_cast<int>(msg.content)]);
    printer_.print(", X: ");
    printer_.print(msg.value.x);
    printer_.print(", Y: ");
    printer_.print(msg.value.y);
    printer_.print(", Unit: ");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print(", Source: ");
    printer_.println(originStr[static_cast<int>(msg.origin)]);
    return true;  // Indicate that the message was handled
  }

  bool onMessage(const Message<GPSCoordinate>& msg) override {
    printer_.print("[Message] Type: ");
    printer_.print(messageContentStr[static_cast<int>(msg.content)]);
    printer_.print(", Lat: ");
    printer_.print(msg.value.latitude);
    printer_.print(", Lon: ");
    printer_.print(msg.value.longitude);
    printer_.print(", Alt: ");
    printer_.print(msg.value.altitude);
    printer_.print(", Unit: ");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print(", Source: ");
    printer_.println(originStr[static_cast<int>(msg.origin)]);
    return true;  // Indicate that the message was handled
  }

  bool onMessage(const Message<MotionState3D>& msg) override {
    printer_.print("[Message] Type: MotionState3D");
    printer_.print(", Pos: (");
    printer_.print(msg.value.getPosition().x);
    printer_.print(", ");
    printer_.print(msg.value.getPosition().y);
    printer_.print(", ");
    printer_.print(msg.value.getPosition().z);
    printer_.print(")");
    printer_.print(", Orientation: (");
    printer_.print(msg.value.getOrientation().roll);
    printer_.print(", ");
    printer_.print(msg.value.getOrientation().pitch);
    printer_.print(", ");
    printer_.print(msg.value.getOrientation().yaw);
    printer_.print(")");
    printer_.print(", Speed: (");
    printer_.print(msg.value.getSpeed().x);
    printer_.print(", ");
    printer_.print(msg.value.getSpeed().y);
    printer_.print(", ");
    printer_.print(msg.value.getSpeed().z);
    printer_.print(")");
    printer_.print(", AngularVel: (");
    printer_.print(msg.value.getAngularVelocity().x);
    printer_.print(", ");
    printer_.print(msg.value.getAngularVelocity().y);
    printer_.print(", ");
    printer_.print(msg.value.getAngularVelocity().z);
    printer_.print(")");
    printer_.println("");
    return true;
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
 *   source.subscribe(printer);
 * @endcode
 *
 * This is useful for logging, exporting, or integrating message traffic with
 * systems that consume XML data.
 * @ingroup communication
 */
class MessageHandlerPrintXML : public MessageHandler {
 public:
  MessageHandlerPrintXML() : printer_(*static_cast<Print*>(nullptr)) {}
  MessageHandlerPrintXML(Print& printer) : printer_(printer) {}
  void setOutput(Print& printer) { printer_ = printer; }

  bool onMessage(const Message<float>& msg) override {
    printer_.print("<Message type=\"");
    printer_.print(messageContentStr[static_cast<int>(msg.content)]);
    printer_.print("\" unit=\"");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print("\" source=\"");
    printer_.print(originStr[static_cast<int>(msg.origin)]);
    printer_.print("\" value=\"");
    printer_.print(msg.value);
    printer_.println("\"/>");
    return true;
  }

  bool onMessage(const Message<Coordinate<float>>& msg) override {
    printer_.print("<Message type=\"");
    printer_.print(messageContentStr[static_cast<int>(msg.content)]);
    printer_.print("\" unit=\"");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print("\" source=\"");
    printer_.print(originStr[static_cast<int>(msg.origin)]);
    printer_.print("\"><X>");
    printer_.print(msg.value.x);
    printer_.print("</X><Y>");
    printer_.print(msg.value.y);
    printer_.println("</Y></Message>");
    return true;
  }

  bool onMessage(const Message<GPSCoordinate>& msg) override {
    printer_.print("<Message type=\"");
    printer_.print(messageContentStr[static_cast<int>(msg.content)]);
    printer_.print("\" unit=\"");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print("\" source=\"");
    printer_.print(originStr[static_cast<int>(msg.origin)]);
    printer_.print("\"><Lat>");
    printer_.print(msg.value.latitude);
    printer_.print("</Lat><Lon>");
    printer_.print(msg.value.longitude);
    printer_.print("</Lon><Alt>");
    printer_.print(msg.value.altitude);
    printer_.println("</Alt></Message>");
    return true;
  }

  bool onMessage(const Message<MotionState3D>& msg) override {
    printer_.print("<Message type=\"MotionState3D\"");
    printer_.print(" pos=\"");
    printer_.print(msg.value.getPosition().x);
    printer_.print(",");
    printer_.print(msg.value.getPosition().y);
    printer_.print(",");
    printer_.print(msg.value.getPosition().z);
    printer_.print("\" orientation=\"");
    printer_.print(msg.value.getOrientation().roll);
    printer_.print(",");
    printer_.print(msg.value.getOrientation().pitch);
    printer_.print(",");
    printer_.print(msg.value.getOrientation().yaw);
    printer_.print("\" speed=\"");
    printer_.print(msg.value.getSpeed().x);
    printer_.print(",");
    printer_.print(msg.value.getSpeed().y);
    printer_.print(",");
    printer_.print(msg.value.getSpeed().z);
    printer_.print("\" angularVel=\"");
    printer_.print(msg.value.getAngularVelocity().x);
    printer_.print(",");
    printer_.print(msg.value.getAngularVelocity().y);
    printer_.print(",");
    printer_.print(msg.value.getAngularVelocity().z);
    printer_.println("\"/>");
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
 *   source.subscribe(printer);
 * @endcode
 *
 * This is useful for logging, exporting, or integrating message traffic with
 * systems that consume JSON data.
 * @ingroup communication
 */
class MessageHandlerPrintJSON : public MessageHandler {
 public:
  MessageHandlerPrintJSON() : printer_(*static_cast<Print*>(nullptr)) {}
  MessageHandlerPrintJSON(Print& printer) : printer_(printer) {}
  void setOutput(Print& printer) { printer_ = printer; }

  bool onMessage(const Message<float>& msg) override {
    printer_.print("{\"type\":\"");
    printer_.print(messageContentStr[static_cast<int>(msg.content)]);
    printer_.print("\",\"value\":");
    printer_.print(msg.value);
    printer_.print(",\"unit\":\"");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print("\",\"source\":\"");
    printer_.print(originStr[static_cast<int>(msg.origin)]);
    printer_.println("\"}");
    return true;
  }

  bool onMessage(const Message<Coordinate<float>>& msg) override {
    printer_.print("{\"type\":\"");
    printer_.print(messageContentStr[static_cast<int>(msg.content)]);
    printer_.print("\",\"x\":");
    printer_.print(msg.value.x);
    printer_.print(",\"y\":");
    printer_.print(msg.value.y);
    printer_.print(",\"unit\":\"");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print("\",\"source\":\"");
    printer_.print(originStr[static_cast<int>(msg.origin)]);
    printer_.println("\"}");
    return true;
  }

  bool onMessage(const Message<GPSCoordinate>& msg) override {
    printer_.print("{\"type\":\"");
    printer_.print(messageContentStr[static_cast<int>(msg.content)]);
    printer_.print("\",\"lat\":");
    printer_.print(msg.value.latitude);
    printer_.print(",\"lon\":");
    printer_.print(msg.value.longitude);
    printer_.print(",\"alt\":");
    printer_.print(msg.value.altitude);
    printer_.print(",\"unit\":\"");
    printer_.print(unitStr[static_cast<int>(msg.unit)]);
    printer_.print("\",\"source\":\"");
    printer_.print(originStr[static_cast<int>(msg.origin)]);
    printer_.println("\"}");
    return true;
  }

  bool onMessage(const Message<MotionState3D>& msg) override {
    printer_.print("{\"type\":\"MotionState3D\"");
    printer_.print(",\"pos\":[");
    printer_.print(msg.value.getPosition().x);
    printer_.print(",");
    printer_.print(msg.value.getPosition().y);
    printer_.print(",");
    printer_.print(msg.value.getPosition().z);
    printer_.print("],\"orientation\":[");
    printer_.print(msg.value.getOrientation().roll);
    printer_.print(",");
    printer_.print(msg.value.getOrientation().pitch);
    printer_.print(",");
    printer_.print(msg.value.getOrientation().yaw);
    printer_.print("],\"speed\":[");
    printer_.print(msg.value.getSpeed().x);
    printer_.print(",");
    printer_.print(msg.value.getSpeed().y);
    printer_.print(",");
    printer_.print(msg.value.getSpeed().z);
    printer_.print("],\"angularVel\":[");
    printer_.print(msg.value.getAngularVelocity().x);
    printer_.print(",");
    printer_.print(msg.value.getAngularVelocity().y);
    printer_.print(",");
    printer_.print(msg.value.getAngularVelocity().z);
    printer_.println("]}");
    return true;
  }

 protected:
  Print& printer_;
};

}  // namespace tinyrobotics
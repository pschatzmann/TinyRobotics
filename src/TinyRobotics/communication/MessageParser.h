#pragma once
#include "Message.h"
#include "TinyRobotics/utils/MemoryStream.h"


namespace tinyrobotics {

/**
 * @enum MessageValueType
 * @ingroup communications
 * @brief Enum to indicate the type of parsed TinyRobotics message.
 *
 * Used by MessageParser to specify which variant of the union in ParsedMessage
 * is valid.
 */
enum class MessageValueType {
  Float,          ///< Message<float>
  Coordinate,     ///< Message<Coordinate<float>>
  GPSCoordinate,  ///< Message<GPSCoordinate>
  Unknown         ///< Unknown or parse error
};

/**
 * @brief Holds a parsed TinyRobotics message of the correct type.
 *
 * The type field indicates which member of the union is valid after parsing.
 * Used as an output parameter for MessageParser::parse().
 * @ingroup communication
 */
struct ParsedMessage {
  MessageValueType type;  ///< Indicates which union member is valid. @see MessageValueType
  union {
    Message<float> floatMsg;
    Message<Coordinate<float>> coordMsg;
    Message<GPSCoordinate> gpsMsg;
  };
  ParsedMessage() : type(MessageValueType::Unknown) {}
  ~ParsedMessage() {}  // Add proper destructors if needed
};

/**
 * @brief Parses binary TinyRobotics messages from a Stream and dispatches them
 * by type.
 *
 * MessageParser provides two main interfaces:
 *
 * 1. parse(Stream&, ParsedMessage&):
 *    - Parses a message from the stream and fills a ParsedMessage union with
 * the correct type.
 *    - The caller can inspect parsed.type and access the appropriate union
 * member.
 *
 * 2. parse(Stream&, MessageHandler&):
 *    - Parses a message from the stream and automatically dispatches it to the
 * correct onMessage() overload of the provided MessageHandler instance.
 *    - This is convenient for direct message handling without manual type
 * inspection.
 *
 * The parser reads the message header (prefix, size, origin_id), then inspects
 * the content field to determine the value type (float, Coordinate<float>,
 * GPSCoordinate). It then reads the value and remaining fields accordingly.
 *
 * Usage (manual type inspection):
 * @code
 *   MessageParser parser;
 *   ParsedMessage parsed;
 *   if (parser.parse(stream, parsed)) {
 *     switch (parsed.type) { ... }
 *   }
 * @endcode
 * 
 * Usage (automatic dispatch):
 * @code
 *   MessageParser parser;
 *   parser.parse(stream, handler); // handler is a MessageHandler subclass
 * @endcode
 * @ingroup communication
 */
class MessageParser {
 public:
  // Parses a message from the stream and fills out the correct type in result
  bool parse(Stream& io, ParsedMessage& result) {
    // 1. Read prefix, size, and origin_id
    char prefix[4] = {0};
    if (io.readBytes(prefix, 3) != 3) return false;
    if (strncmp(prefix, "MSG", 3) != 0) return false;

    uint8_t size = io.read();
    uint8_t origin_id = io.read();

    // 2. Read content and unit fields (assuming they are next)
    MessageContent content;
    if (io.readBytes((char*)&content, sizeof(content)) != sizeof(content))
      return false;
    Unit unit;
    if (io.readBytes((char*)&unit, sizeof(unit)) != sizeof(unit)) return false;

    // 3. Decide type and read value accordingly
    if (content == MessageContent::Position) {
      result.type = MessageValueType::Coordinate;
      // Read Coordinate<float> value
      if (io.readBytes((char*)&result.coordMsg.value,
                       sizeof(Coordinate<float>)) != sizeof(Coordinate<float>))
        return false;
      result.coordMsg.prefix = "MSG";
      result.coordMsg.size = size;
      result.coordMsg.origin_id = origin_id;
      result.coordMsg.content = content;
      result.coordMsg.unit = unit;
      // Read source
      io.readBytes((char*)&result.coordMsg.origin, sizeof(MessageOrigin));
    } else if (content == MessageContent::PositionGPS) {
      result.type = MessageValueType::GPSCoordinate;
      if (io.readBytes((char*)&result.gpsMsg.value, sizeof(GPSCoordinate)) !=
          sizeof(GPSCoordinate))
        return false;
      result.gpsMsg.prefix = "MSG";
      result.gpsMsg.size = size;
      result.gpsMsg.origin_id = origin_id;
      result.gpsMsg.content = content;
      result.gpsMsg.unit = unit;
      io.readBytes((char*)&result.gpsMsg.origin, sizeof(MessageOrigin));
    } else {
      result.type = MessageValueType::Float;
      if (io.readBytes((char*)&result.floatMsg.value, sizeof(float)) !=
          sizeof(float))
        return false;
      result.floatMsg.prefix = "MSG";
      result.floatMsg.size = size;
      result.floatMsg.origin_id = origin_id;
      result.floatMsg.content = content;
      result.floatMsg.unit = unit;
      io.readBytes((char*)&result.floatMsg.origin, sizeof(MessageOrigin));
    }
    return true;
  }

  /**
   * @brief Parses a message from the stream and dispatches it to the given
   * handler.
   *
   * This method parses the message type and calls the appropriate onMessage
   * overload on the provided MessageHandler instance.
   *
   * @param io      The input stream to parse from.
   * @param handler The MessageHandler to dispatch the parsed message to.
   * @return true if a message was successfully parsed and dispatched, false
   * otherwise.
   */
  bool parse(Stream& io, MessageHandler& handler) {
    ParsedMessage parsed;
    if (!parse(io, parsed)) return false;
    switch (parsed.type) {
      case MessageValueType::Float:
        handler.onMessage(parsed.floatMsg);
        break;
      case MessageValueType::Coordinate:
        handler.onMessage(parsed.coordMsg);
        break;
      case MessageValueType::GPSCoordinate:
        handler.onMessage(parsed.gpsMsg);
        break;
      default:
        return false;
    }
    return true;
  }

  /**
   * @brief Parses a message from a raw buffer and dispatches it to the given handler.
   *
   * This overload wraps the buffer in a MemoryStream and calls the Stream-based parse.
   *
   * @param data    Pointer to the message buffer.
   * @param len     Length of the buffer.
   * @param handler The MessageHandler to dispatch the parsed message to.
   * @return true if a message was successfully parsed and dispatched, false otherwise.
   */
  bool parse(uint8_t* data, size_t len, MessageHandler& handler) {
    MemoryStream stream(data, len);
    return parse(stream, handler);
  }
};

}  // namespace tinyrobotics
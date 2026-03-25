#pragma once
#include <sstream>
#include <string>
#include <vector>

#include "TinyRobotics/utils/LoggerClass.h"
#include "GPSCoordinate.h"

namespace tinyrobotics {

enum GPSFormat {
  GGA,  // Fix data
  RMC   // Recommended minimum
};

/**
 * @brief Parses NMEA sentences from GPS modules and extracts GPS data.
 *
 * The NMEAParser class supports parsing of common NMEA sentences such as GGA
 * (fix data) and RMC (recommended minimum), converting them into structured
 * GPSCoordinate objects.
 *
 * Features:
 *   - Supports GGA and RMC NMEA sentence formats
 *   - Converts latitude/longitude from NMEA (degrees/minutes) to decimal
 * degrees
 *   - Extracts altitude, fix quality, and accuracy metrics
 *   - Handles both std::string and Arduino String (if ARDUINO is defined)
 *   - Can be used for navigation, mapping, logging, or any application
 * requiring GPS data
 *
 * Example usage:
 * @code
 * tinyrobotics::NMEAParser parser(tinyrobotics::GPSFormat::GGA);
 * GPSCoordinate gps;
 * if
 * (parser.parse("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
 * gps)) {
 *   // gps now contains parsed data
 * }
 * @endcode
 */

class NMEAParser {
 public:
  NMEAParser(GPSFormat fmt) : format(fmt) {}

  bool parse(const char* sentence, GPSCoordinate& gps) const {
    return parse(std::string(sentence), gps);
  }

  bool parse(const std::string& sentence, GPSCoordinate& gps) const {
    switch (format) {
      case GPSFormat::GGA:
        return parseGGA(sentence, gps);
      case GPSFormat::RMC:
        return parseRMC(sentence, gps);
      default:
        return false;
    }
  }

#ifdef ARDUINO
  bool parse(const String& sentence, GPSCoordinate& gps) const {
    return parse(std::string(sentence.c_str()), gps);
  }
#endif

 protected:
  GPSFormat format;

  // Helper: safe string to float
  static bool safe_stof(const std::string& s, float& out) {
    if (s.empty()) return false;
    char* endptr = nullptr;
    out = strtof(s.c_str(), &endptr);
    return endptr != nullptr && *endptr == '\0';
  }
  // Helper: safe string to int
  static bool safe_stoi(const std::string& s, int& out) {
    if (s.empty()) return false;
    char* endptr = nullptr;
    out = strtol(s.c_str(), &endptr, 10);
    return endptr != nullptr && *endptr == '\0';
  }
  // Parse GGA sentence (fix data)
  bool parseGGA(const std::string& sentence, GPSCoordinate& gps) const {
    // Format: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    std::vector<std::string> fields;
    size_t start = 0;
    size_t end = sentence.find(',');

    while (end != std::string::npos) {
      fields.push_back(sentence.substr(start, end - start));
      start = end + 1;
      end = sentence.find(',', start);
    }
    fields.push_back(sentence.substr(start));

    if (fields.size() < 15) {
      TRLogger.error("NMEAParser: GGA sentence has too few fields: %d" +
                     fields.size());
      return false;
    }

    // // Time
    // if (!fields[1].empty()) {
    //   float time = 0;
    //   if (!safe_stof(fields[1], time)) {
    //     TRLogger.error("NMEAParser: Invalid GGA time: %s", fields[1]);
    //     return false;
    //   }
    //   gps.timestamp = time;  // HHMMSS.SS
    // }

    // Latitude
    if (!fields[2].empty() && !fields[3].empty()) {
      float lat_deg = 0, lat_min = 0;
      if (fields[2].size() < 4 || !safe_stof(fields[2].substr(0, 2), lat_deg) ||
          !safe_stof(fields[2].substr(2), lat_min)) {
        TRLogger.error("NMEAParser: Invalid GGA latitude: %s, %s", fields[2],
                       fields[3]);
        return false;
      }
      gps.latitude = lat_deg + lat_min / 60.0;
      if (fields[3] == "S") gps.latitude = -gps.latitude;
    }

    // Longitude
    if (!fields[4].empty() && !fields[5].empty()) {
      float lon_deg = 0, lon_min = 0;
      if (fields[4].size() < 5 || !safe_stof(fields[4].substr(0, 3), lon_deg) ||
          !safe_stof(fields[4].substr(3), lon_min)) {
        TRLogger.error("NMEAParser: Invalid GGA longitude: %s, %s", fields[4],
                       fields[5]);
        return false;
      }
      gps.longitude = lon_deg + lon_min / 60.0;
      if (fields[5] == "W") gps.longitude = -gps.longitude;
    }

    // // Fix quality
    // if (!fields[6].empty()) {
    //   int fixq = 0;
    //   if (!safe_stoi(fields[6], fixq)) {
    //     TRLogger.error("NMEAParser: Invalid GGA fix quality: %s", fields[6]);
    //     return false;
    //   }
    //   gps.fix_quality = fixq;
    // }

    // // HDOP (Horizontal Dilution of Precision)
    // if (!fields[8].empty()) {
    //   float hdop = 0;
    //   if (!safe_stof(fields[8], hdop)) {
    //     TRLogger.error("NMEAParser: Invalid GGA HDOP: %s", fields[8]);
    //     return false;
    //   }
    //   gps.horizontal_accuracy = hdop * 5.0;  // Approximate 1-sigma
    // }

    // Altitude
    if (!fields[9].empty()) {
      float alt = 0;
      if (!safe_stof(fields[9], alt)) {
        TRLogger.error("NMEAParser: Invalid GGA altitude: %s", fields[9]);
        return false;
      }
      gps.altitude = alt;
    }

    return true;
  }

  // Parse RMC sentence (recommended minimum)
  bool parseRMC(const std::string& sentence, GPSCoordinate& gps) const {
    // Format:
    // $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
    std::vector<std::string> fields;
    size_t start = 0;
    size_t end = sentence.find(',');

    while (end != std::string::npos) {
      fields.push_back(sentence.substr(start, end - start));
      start = end + 1;
      end = sentence.find(',', start);
    }
    fields.push_back(sentence.substr(start));

    if (fields.size() < 12) {
      TRLogger.error("NMEAParser: RMC sentence has too few fields: %d",
                     fields.size());
      return false;
    }

    // // Status
    // if (fields[2] == "A") {
    //   gps.fix_quality = 1;  // Active fix
    // } else {
    //   gps.fix_quality = 0;  // Invalid
    //   TRLogger.error("NMEAParser: RMC status not active: %s", fields[2]);
    //   return false;
    // }

    // Latitude
    if (!fields[3].empty() && !fields[4].empty()) {
      float lat_deg = 0, lat_min = 0;
      if (fields[3].size() < 4 || !safe_stof(fields[3].substr(0, 2), lat_deg) ||
          !safe_stof(fields[3].substr(2), lat_min)) {
        TRLogger.error("NMEAParser: Invalid RMC latitude: %s, %s", fields[3],
                       fields[4]);
        return false;
      }
      gps.latitude = lat_deg + lat_min / 60.0;
      if (fields[4] == "S") gps.latitude = -gps.latitude;
    }

    // Longitude
    if (!fields[5].empty() && !fields[6].empty()) {
      float lon_deg = 0, lon_min = 0;
      if (fields[5].size() < 5 || !safe_stof(fields[5].substr(0, 3), lon_deg) ||
          !safe_stof(fields[5].substr(3), lon_min)) {
        TRLogger.error("NMEAParser: Invalid RMC longitude: %s/%s", fields[5],
                       fields[6]);
        return false;
      }
      gps.longitude = lon_deg + lon_min / 60.0;
      if (fields[6] == "W") gps.longitude = -gps.longitude;
    }

    // Speed over ground (knots)
    if (!fields[7].empty()) {
      float speed_knots = 0;
      if (!safe_stof(fields[7], speed_knots)) {
        TRLogger.error("NMEAParser: Invalid RMC speed: %s", fields[7]);
        return false;
      }
      // speed_ms = speed_knots * 0.514444;
    }

    return true;
  }

  std::vector<std::string> split(const std::string& str, char delim) {
    std::vector<std::string> tokens;
    std::stringstream ss{str};
    std::string token;
    while (std::getline(ss, token, delim)) {
      tokens.push_back(token);
    }
    return tokens;
  }
};

}  // namespace tinyrobotics
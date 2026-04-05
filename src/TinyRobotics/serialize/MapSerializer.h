
#pragma once

#include "TinyRobotics/maps/PathSegment.h"
#include "TinyRobotics/utils/Common.h"

namespace tinyrobotics {

// Reads a line from the stream into 'out'. Returns true if a line was read,
// false on EOF.
// Reads a line from the stream into 'out'. Returns true if a line was read,
// false on EOF.
static bool readLine(Stream& s, std::string& out) {
  out.clear();
  while (true) {
    int c = s.read();
    if (c < 0) {
      // If nothing read, return false to signal EOF
      return !out.empty();
    }
    if (c == '\n') break;
    out += static_cast<char>(c);
  }
  return true;
}
/**
 * @class GridMapSerializer
 * @brief Utility class for serializing and deserializing grid maps in CSV
 * format.
 *
 * The GridMapSerializer class provides methods to write a grid map to a Print
 * object (such as Serial) and to read a grid map from a Stream object. It is
 * designed to work with map types that provide the following interface:
 *   - getXCount(), getYCount(), getResolution(), cellToWorld(), getCell(),
 * setCell(), resize(), setResolution()
 *
 * The serialization format is:
 *   - Header: "GridMap x:<xCount>, y:<yCount>, resolution=<res>"
 *   - One line per cell: "<wx>,<wy>,<value>" (CSV: world x, world y, value)
 *     - If the value is missing, the line is "<wx>,<wy>,"
 *
 * This format is compatible with external tools and spreadsheets for easy
 * inspection and editing.
 *
 * Example:
 *   GridMap x:10, y:10, resolution=0.1
 *   0,0,0
 *   0,0.1,1
 *   ...
 *
 * @tparam Map The map type to serialize/deserialize (e.g., GridMap)
 * @tparam StateT The cell state type (e.g., float, int, CellState)
 * @tparam T   The numeric type for coordinates (e.g., float)
 */

template <typename Map, typename StateT = CellState, typename T = float>
class GridMapSerializer {
 public:
  // write map to output
  size_t write(Map& map, Print& out) {
    size_t total = 0;
    char buf[128];
    int n =
        snprintf(buf, sizeof(buf), "GridMap x:%d, y:%d, resolution=%g\n",
                 map.getXCount(), map.getYCount(), (double)map.getResolution());
    if (n > 0) total += out.print(buf);
    for (int x = 0; x < map.getXCount(); ++x) {
      for (int y = 0; y < map.getYCount(); ++y) {
        Coordinate<T> coord;
        map.cellToWorld(x, y, coord.x, coord.y);
        StateT state;
        if (map.getCell(x, y, state)) {
          int n = snprintf(buf, sizeof(buf), "%g,%g,%g\n", (double)coord.x,
                           (double)coord.y, (double)state);
          if (n > 0) total += out.print(buf);
        } else {
          int n = snprintf(buf, sizeof(buf), "%g,%g,\n", (double)coord.x,
                           (double)coord.y);
          if (n > 0) total += out.print(buf);
        }
      }
    }
    out.flush();
    return total;
  }

  // read map from input
  size_t readFrom(Map& map, Stream& in) {
    size_t total = 0;
    int x, y;
    float res;

    // Read header
    std::string header;
    if (!readLine(in, header)) return 0;
    total += header.length() + 1;
    if (sscanf(header.c_str(), "GridMap x:%d, y:%d, resolution=%f", &x, &y,
               &res) != 3)
      return 0;
    if (x <= 0 || y <= 0 || res <= 0) return 0;
    map.resize(x, y);
    map.setResolution(res);

    // Read cell values
    int cellCount = x * y;
    int filled = 0;
    while (filled < cellCount) {
      std::string line;
      if (!readLine(in, line)) return 0;
      total += line.length() + 1;
      float wx, wy, value;
      int parsed = sscanf(line.c_str(), "%f,%f,%f", &wx, &wy, &value);
      if (parsed < 2) return 0;  // must have at least x and y
      Coordinate<T> coord(static_cast<T>(wx), static_cast<T>(wy));
      if (parsed == 3) {
        map.setCell(coord, (StateT)value);
      }
      ++filled;
    }
    return total;
  }
};

/**
 * @class PathMapSerializer
 * @brief Utility class for serializing and deserializing path maps (sequences
 * of segments) in CSV format.
 *
 * The PathMapSerializer class provides methods to write a path map (a
 * sequence of segments, each with a 'from' and 'to' coordinate, a cost, and a
 * directed flag) to a Print object (such as Serial) and to read a path map
 * from a Stream object. It is designed to work with map types that provide:
 *   - size(), operator[](size_t), and addSegment(from, to, cost, directed)
 *   - Each segment must have 'from' and 'to' members of type CoordinateT,
 * plus 'cost' (double/float) and 'directed' (bool)
 *
 * The serialization format is:
 *   - One line per segment:
 * "<from.x>,<from.y>,<to.x>,<to.y>,<cost>,<directed>" (CSV)
 *     - <cost> is a floating-point value, <directed> is 0 (false) or 1 (true)
 *     - If <cost> and <directed> are missing, defaults are used (cost=1.0,
 * directed=false)
 *
 * This format is compatible with external tools and spreadsheets for easy
 * inspection and editing.
 *
 * Example:
 *   0,0,1,1,2.5,1
 *   1,1,2,2,1.0,0
 *   ...
 *
 * @tparam MapT The map type to serialize/deserialize (must support segment
 * access and addSegment)
 * @tparam CoordinateT The coordinate type for segment endpoints (e.g.,
 * Coordinate<float>)
 */

template <typename MapT, typename CoordinateT = Coordinate<float>>
class PathMapSerializer {
  using T = typename CoordinateT::value_type;

  size_t write(MapT& map, Print& out) {
    char buf[160];
    size_t total = 0;
    size_t len = map.size();
    for (size_t i = 0; i < len; ++i) {
      auto& segment = map[i];
      auto& from = segment.from;
      auto& to = segment.to;
      double cost = segment.cost;
      bool directed = segment.directed;
      int n = snprintf(buf, sizeof(buf), "%g,%g,%g,%g,%g,%d\n", (double)from.x,
                       (double)from.y, (double)to.x, (double)to.y, cost,
                       directed ? 1 : 0);
      if (n > 0) total += out.print(buf);
    }
    out.flush();
    return total;
  }

  size_t readFrom(MapT& map, Stream& in) {
    size_t total = 0;
    std::string line;
    while (readLine(in, line)) {
      total += line.length() + 1;
      float fx, fy, tx, ty, cost = 1.0f;
      int directedInt = 0;
      int parsed = sscanf(line.c_str(), "%f,%f,%f,%f,%f,%d", &fx, &fy, &tx, &ty,
                          &cost, &directedInt);
      if (parsed < 4) return 0;  // must have at least coordinates
      CoordinateT from(static_cast<T>(fx), static_cast<T>(fy));
      CoordinateT to(static_cast<T>(tx), static_cast<T>(ty));
      double segCost = (parsed >= 5) ? cost : 1.0;
      bool directed = (parsed == 6) ? (directedInt != 0) : false;
      PathSegment<CoordinateT> segment{from, to, segCost, directed};
      map.addSegment(segment);
    }
    return total;
  }
};

/**
 * @class PointCloudSerializer
 * @brief Utility class for serializing and deserializing PointCloud objects,
 * including points, voxels, and attributes.
 *
 * The PointCloudSerializer class provides methods to write a PointCloud to a
 * Print object (such as Serial) and to read a PointCloud from a Stream object.
 * The format includes a header with attributes, points, and voxels.
 *
 * Format:
 *   #PointCloud voxelSize:<float> is3d:<0|1>
 * bounds:<minx>,<miny>,<minz>,<maxx>,<maxy>,<maxz> #Points x1,y1,z1
 *   ...
 *   #Voxels
 *   vx1,vy1,vz1
 *   ...
 *
 * @tparam PointCloudT The PointCloud type to serialize/deserialize (e.g.,
 * PointCloud<float>)
 * @tparam T The numeric type for coordinates (e.g., float)
 */
template <typename PointCloudT>
class PointCloudSerializer {
  using T = typename PointCloudT::value_type;

 public:
  // Write all attributes, points, and voxels
  size_t write(const PointCloudT& cloud, Print& out) {
    size_t total = 0;
    char buf[128];
    // Header
    const auto& b = cloud.bounds();
    sprintf(buf, "#PointCloud voxelSize:%g is3d:%d bounds:%g,%g,%g,%g,%g,%g\n",
            (double)cloud.voxelSize(), cloud.is3D() ? 1 : 0, (double)b.min.x,
            (double)b.min.y, (double)b.min.z, (double)b.max.x, (double)b.max.y,
            (double)b.max.z);
    total += out.print(buf);
    // Points
    total += out.print("#Points\n");
    for (const auto& pt : cloud.points()) {
      sprintf(buf, "%g,%g,%g\n", (double)pt.x, (double)pt.y, (double)pt.z);
      total += out.print(buf);
    }
    // Voxels
    total += out.print("#Voxels\n");
    for (auto it = cloud.beginVoxels(); it != cloud.endVoxels(); ++it) {
      sprintf(buf, "%d,%d,%d\n", it->x, it->y, it->z);
      total += out.print(buf);
    }
    out.flush();
    return total;
  }

  // Read all attributes, points, and voxels
  size_t readFrom(PointCloudT& cloud, Stream& in) {
    size_t total = 0;
    std::string line;
    // Header
    if (!readLine(in, line)) return 0;
    total += line.length() + 1;
    float voxelSize = 0.0f, minx, miny, minz, maxx, maxy, maxz;
    int is3d = 0;
    if (sscanf(line.c_str(),
               "#PointCloud voxelSize:%f is3d:%d bounds:%f,%f,%f,%f,%f,%f",
               &voxelSize, &is3d, &minx, &miny, &minz, &maxx, &maxy,
               &maxz) != 8)
      return 0;
    cloud.set3D(is3d != 0);
    cloud.setBounds(minx, miny, minz, maxx, maxy, maxz);
    cloud.buildVoxelGrid(voxelSize);
    // Expect #Points
    if (!readLine(in, line)) return 0;
    total += line.length() + 1;
    if (line.find("#Points") != 0) return 0;
    // Read points
    while (readLine(in, line)) {
      total += line.length() + 1;
      if (line.find("#Voxels") == 0) break;
      float x, y, z = 0.0f;
      int parsed = sscanf(line.c_str(), "%f,%f,%f", &x, &y, &z);
      if (parsed < 2) return 0;
      cloud.add(static_cast<T>(x), static_cast<T>(y), static_cast<T>(z));
    }
    // Read voxels
    while (readLine(in, line)) {
      total += line.length() + 1;
      int vx, vy, vz;
      if (sscanf(line.c_str(), "%d,%d,%d", &vx, &vy, &vz) == 3) {
        cloud.addVoxel((float)vx * voxelSize, (float)vy * voxelSize,
                       (float)vz * voxelSize);
      }
    }
    return total;
  }
};

}  // namespace tinyrobotics

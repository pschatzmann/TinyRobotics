#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "TinyRobotics/coordinates/Coordinate.h"
#include "TinyRobotics/utils/Common.h"
#include "TinyRobotics/utils/LoggerClass.h"

namespace tinyrobotics {
/**
 * @struct Point3D
 * @ingroup maps
 * @brief Represents a 3D point with x, y, z coordinates (in meters).
 *
 * Used throughout the mapping and robotics modules for point cloud processing,
 * spatial queries, and geometric computations. Each coordinate is stored as a T
 * (meters).
 */
template <typename T = DistanceM>
struct Point3D {
  T x;
  T y;
  T z;
};

/**
 * @class PointCloud
 * @ingroup maps
 * @brief Represents a 3D point cloud for robotics and mapping applications.
 *
 * The PointCloud class manages a collection of 3D points and provides:
 * - Efficient addition of points (individually, from arrays, or from
 * Coordinate).
 * - Automatic bounding box tracking for fast spatial queries.
 * - Voxel grid downsampling to reduce point density while preserving structure.
 * - Fast occupancy queries via a voxel grid (for collision checking or
 * mapping).
 * - Optional "live" voxel grid mode for real-time occupancy updates as points
 * are added.
 *
 * Usage scenarios include 3D mapping, obstacle detection, SLAM, and environment
 * representation. The class is designed for embedded and robotics use,
 * balancing efficiency and flexibility.
 *
 * Key features:
 * - Add, clear, and access points.
 * - Retrieve the bounding box (min/max x, y, z).
 * - Downsample using a voxel grid (keep one point per voxel).
 * - Build and query a voxel occupancy grid for fast lookups.
 * - Optionally enable live voxel grid updates for real-time occupancy.
 *
 * Example:
 * @code
 * PointCloud cloud;
 * cloud.add(1.0, 2.0, 0.5);
 * cloud.buildVoxelGrid(0.1f); // 10cm voxels
 * bool occ = cloud.isOccupied(1.0, 2.0, 0.5);
 * @endcode
 */
template <typename T = DistanceM>
class PointCloud : public IMapNeighbors<T> {
  /// Bounding box
  struct Bounds {
    Point3D<T> min;
    Point3D<T> max;
  };

  struct Key {
    int x, y, z;
    bool operator==(const Key& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct KeyHash {
    size_t operator()(const Key& k) const {
      return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^
             (std::hash<int>()(k.z) << 1);
    }
  };

 public:
  using Container = std::vector<Point3D<T>, AllocatorPSRAM<Point3D<T>>>;

  PointCloud(bool isliveVoxel = false) {
    resetBounds();
    setLiveVoxelGrid(isliveVoxel);
  }

  /// Update the voxel grid as points are added (for real-time occupancy
  /// updates).
  void setLiveVoxelGrid(bool live) { liveVocelGrid_ = live; }

  /// Add a single point
  void add(T x, T y, T z = 0.0f) {
    Point3D<T> p{x, y, z};
    points_.push_back(p);
    updateBounds(p);
    if (liveVocelGrid_) addVoxel(x, y, z);
  }

  /// Add from array
  void add(const Point3D<T>* pts, size_t count) {
    for (size_t i = 0; i < count; ++i) add(pts[i].x, pts[i].y, pts[i].z);
  }

  /// Add from Coordinate
  void add(const Coordinate<T>& coord) { add(coord.x, coord.y, coord.z); }

  /// Clear all data
  void clear() {
    points_.clear();
    voxelGrid_.clear();
    resetBounds();
  }

  /// Provides access to the points in the point cloud.
  const Container& points() const { return points_; }

  /// Provides access to the points in the point cloud.
  Container& points() { return points_; }

  /// Provide the number of points in the point cloud.
  size_t size() const { return points_.size(); }

  /// Checks if the point cloud is empty (i.e., contains no points).
  bool isEmpty() const { return points_.empty(); }

  /// @brief  Returns the bounding box of the point cloud, which is defined by
  /// the minimum and maximum coordinates along each axis (x, y, z). The
  /// bounding box is updated whenever new points are added to the cloud,
  /// allowing for efficient retrieval of the spatial extent of the point cloud.
  Bounds bounds() const { return bounds_; }

  /// Simple voxel downsampling: e.g. 0.5 m to reduce number of points while
  /// keeping overall structure.
  PointCloud voxelDownsample(float voxelSize) const {
    PointCloud out;
    if (voxelSize <= 0.0f) return out;

    std::unordered_map<Key, Point3D<T>, KeyHash> voxels;

    for (const auto& p : points_) {
      Key key{int(std::floor(p.x / voxelSize)),
              int(std::floor(p.y / voxelSize)),
              int(std::floor(p.z / voxelSize))};

      // Keep first point per voxel (simple version)
      if (voxels.find(key) == voxels.end()) voxels[key] = p;
    }

    for (const auto& kv : voxels)
      out.add(kv.second.x, kv.second.y, kv.second.z);

    return out;
  }

  /// Builds a voxel grid for fast occupancy queries: A voxel is a grid cell
  void buildVoxelGrid(float voxelSize) {
    voxelGrid_.clear();
    voxelSize_ = voxelSize;

    for (const auto& p : points_) {
      addVoxel(p.x, p.y, p.z);
    }
    TRLogger.debug("Voxel grid built with voxel size: %f, total voxels: %d",
                   voxelSize_, (int)voxelGrid_.size());
    for (const auto& key : voxelGrid_) {
      TRLogger.debug("Voxel: (%d, %d, %d)", key.x, key.y, key.z);
    }
  }

  /// Adds a point to the voxel grid if voxelSize_ > 0
  void addVoxel(float x, float y, float z) {
    if (voxelSize_ > 0.0f) {
      Key key{int(std::floor(x / voxelSize_)), int(std::floor(y / voxelSize_)),
              int(std::floor(z / voxelSize_))};
      voxelGrid_.insert(key);
    } else {
      TRLogger.warn("Voxel size must be > 0 to add voxels");
    }
  }

  /// Checks if a given point is occupied based on the voxel grid.
  bool isOccupied(float x, float y, float z) const {
    if (voxelSize_ <= 0.0f) return false;

    Key key{int(std::floor(x / voxelSize_)), int(std::floor(y / voxelSize_)),
            int(std::floor(z / voxelSize_))};

    return voxelGrid_.find(key) != voxelGrid_.end();
  }

  /**
   * @brief Returns the free (unoccupied) neighboring coordinates of a given
   * point in the voxel grid.
   *
   * If the voxel grid is built (voxelSize_ > 0), this method returns all
   * adjacent unoccupied voxels (6-connectivity in 3D, or 4/8-connectivity in 2D
   * if z=0). Only neighbors that are not present in the voxel grid are
   * returned.
   *
   * @param from The coordinate to find neighbors for.
   * @return std::vector<Coordinate<T>> List of free neighboring
   * coordinates.
   */
  std::vector<Coordinate<T>> getNeighbors(Coordinate<T> from) const override {
    std::vector<Coordinate<T>> neighbors;
    if (voxelSize_ <= 0.0f) return neighbors;
    TRLogger.debug("-------------------");
    TRLogger.debug(
        "Finding neighbors for: %s with bounding box min:(%f,%f,%f) / "
        "max:(%f,%f,%f)",
        from.toCString(), bounds_.min.x, bounds_.min.y, bounds_.min.z,
        bounds_.max.x, bounds_.max.y, bounds_.max.z);

    // Compute voxel index of 'from'
    int x0 = int(std::floor(from.x / voxelSize_));
    int y0 = int(std::floor(from.y / voxelSize_));
    int z0 = int(std::floor(from.z / voxelSize_));

    if (is_3d) {
      // 6-connectivity in 3D (face neighbors)
      const int dx[6] = {1, -1, 0, 0, 0, 0};
      const int dy[6] = {0, 0, 1, -1, 0, 0};
      const int dz[6] = {0, 0, 0, 0, 1, -1};
      for (int i = 0; i < 6; ++i) {
        int nx = x0 + dx[i];
        int ny = y0 + dy[i];
        int nz = z0 + dz[i];
        // Convert voxel index back to coordinate (center of voxel)
        T cx = (nx + 0.5f) * voxelSize_;
        T cy = (ny + 0.5f) * voxelSize_;
        T cz = (nz + 0.5f) * voxelSize_;
        // Check bounding box
        if (cx < bounds_.min.x || cx > bounds_.max.x || cy < bounds_.min.y ||
            cy > bounds_.max.y || cz < bounds_.min.z || cz > bounds_.max.z) {
          continue;
        }
        Key nkey{nx, ny, nz};
        // Only add if NOT occupied (free neighbor)
        if (voxelGrid_.find(nkey) == voxelGrid_.end()) {
          neighbors.emplace_back(cx, cy, cz);
        }
      }
    } else {
      // 4-connectivity in 2D (z=0 plane)
      const int dx[4] = {1, -1, 0, 0};
      const int dy[4] = {0, 0, 1, -1};
      for (int i = 0; i < 4; ++i) {
        int nx = x0 + dx[i];
        int ny = y0 + dy[i];
        int nz = z0;  // keep z the same
        T cx = (nx + 0.5f) * voxelSize_;
        T cy = (ny + 0.5f) * voxelSize_;
        T cz = 0.0f;  // Always use z=0 for 2D
        // Check bounding box (z=0 plane)
        if (cx < bounds_.min.x || cx > bounds_.max.x || cy < bounds_.min.y ||
            cy > bounds_.max.y) {
          continue;
        }
        Key nkey{nx, ny, nz};
        if (voxelGrid_.find(nkey) == voxelGrid_.end()) {
          neighbors.emplace_back(cx, cy, cz);
        }
      }
    }
    for (auto& n : neighbors) {
      TRLogger.debug("Neighbor: %s", n.toCString());
    }
    TRLogger.debug("-------------------");

    return neighbors;
  }

  /// Convert voxel grid index (cx, cy) to world coordinate (center of voxel,
  /// z=0)
  Coordinate<T> toVoxel(int cx, int cy) const {
    // This matches the logic in getNeighbors for 2D
    T wx = (cx + 0.5f) * voxelSize_;
    T wy = (cy + 0.5f) * voxelSize_;
    T wz = 0.0f;
    return Coordinate<T>(wx, wy, wz);
  }

  /// Use 3D in findNeighbors (if false, only consider z=0 plane for 2D
  /// occupancy)
  void set3D(bool is3d) { is_3d = is3d; }

  /// Manually set the bounding box (if known in advance, can save time instead
  /// of
  void setBounds(float minX, float minY, float minZ, float maxX, float maxY,
                 float maxZ) {
    bounds_.min = {minX, minY, minZ};
    bounds_.max = {maxX, maxY, maxZ};
  }

  /// Checks if a coordinate is valid (within bounds and not occupied)
  bool isValid(const Coordinate<T>& coord) const override {
    // Check bounding box
    if (coord.x < bounds_.min.x || coord.x > bounds_.max.x ||
        coord.y < bounds_.min.y || coord.y > bounds_.max.y ||
        coord.z < bounds_.min.z || coord.z > bounds_.max.z) {
      return false;
    }
    // If voxel grid is enabled, check occupancy
    if (voxelSize_ > 0.0f && isOccupied(coord.x, coord.y, coord.z)) {
      return false;
    }
    return true;
  }

 protected:
  bool is_3d = false;
  Container points_;
  Bounds bounds_;
  T voxelSize_ = 0.0f;
  bool liveVocelGrid_ = false;
  // voxel grid preferrably is psram
  using PSRAMKeyAllocator = AllocatorPSRAM<Key>;
  std::unordered_set<Key, KeyHash, std::equal_to<Key>, PSRAMKeyAllocator>
      voxelGrid_;

  void resetBounds() {
    bounds_.min = {std::numeric_limits<float>::max(),
                   std::numeric_limits<float>::max(),
                   std::numeric_limits<float>::max()};

    bounds_.max = {std::numeric_limits<float>::lowest(),
                   std::numeric_limits<float>::lowest(),
                   std::numeric_limits<float>::lowest()};
  }

  void updateBounds(const Point3D<T>& p) {
    if (p.x < bounds_.min.x) bounds_.min.x = p.x;
    if (p.y < bounds_.min.y) bounds_.min.y = p.y;
    if (p.z < bounds_.min.z) bounds_.min.z = p.z;

    if (p.x > bounds_.max.x) bounds_.max.x = p.x;
    if (p.y > bounds_.max.y) bounds_.max.y = p.y;
    if (p.z > bounds_.max.z) bounds_.max.z = p.z;
  }
};

}  // namespace tinyrobotics
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

struct Point3D {
  DistanceM x;
  DistanceM y;
  DistanceM z;
};

/**
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
class PointCloud {
  /// Bounding box
  struct Bounds {
    Point3D min;
    Point3D max;
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
  using Container = std::vector<Point3D, tinyrobotics::AllocatorPSRAM<Point3D>>;

  PointCloud(bool isliveVoxel = false) {
    resetBounds();
    setLiveVoxelGrid(isliveVoxel);
  }

  /// Update the voxel grid as points are added (for real-time occupancy
  /// updates).
  void setLiveVoxelGrid(bool live) { liveVocelGrid_ = live; }

  /// Add a single point
  void add(DistanceM x, DistanceM y, DistanceM z) {
    Point3D p{x, y, z};
    points_.push_back(p);
    updateBounds(p);
    if (liveVocelGrid_) addVoxel(x, y, z);
  }

  /// Add from array
  void add(const Point3D* pts, size_t count) {
    for (size_t i = 0; i < count; ++i) add(pts[i].x, pts[i].y, pts[i].z);
  }

  /// Add from Coordinate
  void add(const Coordinate<DistanceM>& coord) {
    add(coord.x, coord.y, coord.z);
  }

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

    std::unordered_map<Key, Point3D, KeyHash> voxels;

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

 protected:
  Container points_;
  Bounds bounds_;
  DistanceM voxelSize_ = 0.0f;
  bool liveVocelGrid_ = false;
  // voxel grid preferrably is psram
  using PSRAMKeyAllocator = tinyrobotics::AllocatorPSRAM<Key>;
  std::unordered_set<Key, KeyHash, std::equal_to<Key>, PSRAMKeyAllocator> voxelGrid_;

  void resetBounds() {
    bounds_.min = {std::numeric_limits<float>::max(),
                   std::numeric_limits<float>::max(),
                   std::numeric_limits<float>::max()};

    bounds_.max = {std::numeric_limits<float>::lowest(),
                   std::numeric_limits<float>::lowest(),
                   std::numeric_limits<float>::lowest()};
  }

  void updateBounds(const Point3D& p) {
    if (p.x < bounds_.min.x) bounds_.min.x = p.x;
    if (p.y < bounds_.min.y) bounds_.min.y = p.y;
    if (p.z < bounds_.min.z) bounds_.min.z = p.z;

    if (p.x > bounds_.max.x) bounds_.max.x = p.x;
    if (p.y > bounds_.max.y) bounds_.max.y = p.y;
    if (p.z > bounds_.max.z) bounds_.max.z = p.z;
  }
};

}  // namespace tinyrobotics
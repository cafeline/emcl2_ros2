#ifndef EMCL2__COMPRESSED_VOXEL_MAP_H_
#define EMCL2__COMPRESSED_VOXEL_MAP_H_

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace emcl2 {

  class CompressedVoxelMap
  {
public:
    CompressedVoxelMap() = default;

    bool loadFromFile(const std::string & path);

    bool isOccupied(double x, double y, double z) const;
    bool isOccupied(const Eigen::Vector3d & position) const;

    double voxelSize() const;

private:
    struct BlockCoord
    {
      int32_t x;
      int32_t y;
      int32_t z;

      bool operator == (const BlockCoord & other) const
      {
        return x == other.x && y == other.y && z == other.z;
      }
    };

    struct BlockCoordHash
    {
      std::size_t operator()(const BlockCoord & coord) const noexcept
      {
        std::size_t hx = static_cast < std::size_t > (coord.x) * 73856093u;
        std::size_t hy = static_cast < std::size_t > (coord.y) * 19349663u;
        std::size_t hz = static_cast < std::size_t > (coord.z) * 83492791u;
        return hx ^ hy ^ hz;
      }
    };

    double voxel_size_ {0.0};
    int block_size_ {0};
    Eigen::Vector3d origin_ {Eigen::Vector3d::Zero()};
    int pattern_length_ {0};
    int pattern_bytes_ {0};
    std::vector < std::uint8_t > dictionary_patterns_;
    std::unordered_map < BlockCoord, std::uint16_t, BlockCoordHash > block_to_pattern_;
  };

}  // namespace emcl2

#endif  // EMCL2__COMPRESSED_VOXEL_MAP_H_

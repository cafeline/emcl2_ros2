// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__COMPRESSED_VOXEL_MAP_H_
#define EMCL2__COMPRESSED_VOXEL_MAP_H_

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <string>
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
    double voxel_size_ {0.0};
    double inv_voxel_size_ {0.0};
    int block_size_ {0};
    Eigen::Vector3d origin_ {Eigen::Vector3d::Zero()};
    Eigen::Vector3i block_dims_ {Eigen::Vector3i::Zero()};
    std::size_t pattern_bits_ {0};
    std::size_t pattern_bytes_ {0};
    std::vector < std::uint8_t > dictionary_patterns_;
    std::vector < std::uint32_t > block_indices_;
    std::size_t dictionary_size_ {0};
    std::uint8_t block_index_bit_width_ {1};
    std::size_t stride_y_ {0};
    std::size_t stride_z_ {0};
  };

}  // namespace emcl2

#endif  // EMCL2__COMPRESSED_VOXEL_MAP_H_

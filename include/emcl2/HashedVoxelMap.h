// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__HASHED_VOXEL_MAP_H_
#define EMCL2__HASHED_VOXEL_MAP_H_

#include <Eigen/Core>

#include <memory>
#include <string>

namespace emcl2 {

  class HashedVoxelMap
  {
public:
    HashedVoxelMap();
    ~HashedVoxelMap();

    HashedVoxelMap(const HashedVoxelMap &);
    HashedVoxelMap & operator = (const HashedVoxelMap &);
    HashedVoxelMap(HashedVoxelMap &&) noexcept;
    HashedVoxelMap & operator = (HashedVoxelMap &&)noexcept;

    bool loadFromFile(const std::string & path);

    bool isOccupied(double x, double y, double z) const;
    bool isOccupied(const Eigen::Vector3d & position) const;

    double voxelSize() const;

private:
    class Impl;
    std::shared_ptr < Impl > impl_;
  };

}  // namespace emcl2

#endif  // EMCL2__HASHED_VOXEL_MAP_H_

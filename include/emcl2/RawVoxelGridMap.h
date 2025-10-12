#ifndef EMCL2__RAW_VOXEL_GRID_MAP_H_
#define EMCL2__RAW_VOXEL_GRID_MAP_H_

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace emcl2
{

  class RawVoxelGridMap
  {
public:
    RawVoxelGridMap() = default;

    bool loadFromFile(const std::string & path);

    bool isOccupied(double x, double y, double z) const;
    bool isOccupied(const Eigen::Vector3d & position) const;

    double voxelSize() const;

private:
    double voxel_size_ {0.0};
    double inv_voxel_size_ {0.0};
    Eigen::Vector3d origin_ {Eigen::Vector3d::Zero()};
    Eigen::Vector3i dims_ {Eigen::Vector3i::Zero()};
    std::size_t stride_y_ {0};
    std::size_t stride_z_ {0};
    std::vector < uint8_t > voxel_values_;
  };

}  // namespace emcl2

#endif  // EMCL2__RAW_VOXEL_GRID_MAP_H_

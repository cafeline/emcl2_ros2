#include "emcl2/RawVoxelGridMap.h"

#include <hdf5.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace emcl2
{
namespace
{

bool datasetExists(hid_t location, const char * name)
{
  return H5Lexists(location, name, H5P_DEFAULT) > 0;
}

std::vector<hsize_t> datasetShape(hid_t dataset)
{
  hid_t space = H5Dget_space(dataset);
  if (space < 0) {
    return {};
  }
  const int ndims = H5Sget_simple_extent_ndims(space);
  std::vector<hsize_t> dims(static_cast<std::size_t>(ndims));
  H5Sget_simple_extent_dims(space, dims.data(), nullptr);
  H5Sclose(space);
  return dims;
}

void resetState(
  double & voxel_size, double & inv_voxel_size,
  Eigen::Vector3d & origin, Eigen::Vector3i & dims,
  std::size_t & stride_y, std::size_t & stride_z,
  std::vector<uint8_t> & values)
{
  voxel_size = 0.0;
  inv_voxel_size = 0.0;
  origin = Eigen::Vector3d::Zero();
  dims = Eigen::Vector3i::Zero();
  stride_y = 0;
  stride_z = 0;
  values.clear();
}

}  // namespace

bool RawVoxelGridMap::loadFromFile(const std::string & path)
{
  resetState(voxel_size_, inv_voxel_size_, origin_, dims_, stride_y_, stride_z_, voxel_values_);

  hid_t file = H5Fopen(path.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
  if (file < 0) {
    return false;
  }

  bool ok = true;

  try {
    hid_t group = H5Gopen2(file, "/raw_voxel_grid", H5P_DEFAULT);
    if (group < 0) {
      throw std::runtime_error("raw_voxel_grid group missing");
    }

    if (!datasetExists(group, "dimensions")) {
      H5Gclose(group);
      throw std::runtime_error("dimensions dataset missing");
    }
    hid_t dims_dataset = H5Dopen2(group, "dimensions", H5P_DEFAULT);
    if (dims_dataset < 0) {
      H5Gclose(group);
      throw std::runtime_error("failed to open dimensions dataset");
    }
    std::array<uint32_t, 3> dims_buf{{0U, 0U, 0U}};
    herr_t status = H5Dread(
      dims_dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, dims_buf.data());
    H5Dclose(dims_dataset);
    if (status < 0) {
      H5Gclose(group);
      throw std::runtime_error("failed to read dimensions dataset");
    }
    if (std::any_of(dims_buf.begin(), dims_buf.end(), [](uint32_t v) {return v == 0U;})) {
      H5Gclose(group);
      throw std::runtime_error("invalid zero dimension");
    }
    dims_ = Eigen::Vector3i(
      static_cast<int>(dims_buf[0]),
      static_cast<int>(dims_buf[1]),
      static_cast<int>(dims_buf[2]));

    if (!datasetExists(group, "voxel_size")) {
      H5Gclose(group);
      throw std::runtime_error("voxel_size dataset missing");
    }
    hid_t voxel_size_dataset = H5Dopen2(group, "voxel_size", H5P_DEFAULT);
    if (voxel_size_dataset < 0) {
      H5Gclose(group);
      throw std::runtime_error("failed to open voxel_size dataset");
    }
    float voxel_size_f = 0.0f;
    status = H5Dread(
      voxel_size_dataset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT,
      &voxel_size_f);
    H5Dclose(voxel_size_dataset);
    if (status < 0 || voxel_size_f <= 0.0f) {
      H5Gclose(group);
      throw std::runtime_error("invalid voxel size");
    }
    voxel_size_ = static_cast<double>(voxel_size_f);
    inv_voxel_size_ = 1.0 / voxel_size_;

    if (!datasetExists(group, "origin")) {
      H5Gclose(group);
      throw std::runtime_error("origin dataset missing");
    }
    hid_t origin_dataset = H5Dopen2(group, "origin", H5P_DEFAULT);
    if (origin_dataset < 0) {
      H5Gclose(group);
      throw std::runtime_error("failed to open origin dataset");
    }
    std::array<float, 3> origin_buf{{0.0f, 0.0f, 0.0f}};
    status = H5Dread(
      origin_dataset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, origin_buf.data());
    H5Dclose(origin_dataset);
    if (status < 0) {
      H5Gclose(group);
      throw std::runtime_error("failed to read origin dataset");
    }
    origin_ = Eigen::Vector3d(
      static_cast<double>(origin_buf[0]),
      static_cast<double>(origin_buf[1]),
      static_cast<double>(origin_buf[2]));

    if (!datasetExists(group, "voxel_values")) {
      H5Gclose(group);
      throw std::runtime_error("voxel_values dataset missing");
    }
    hid_t voxel_values_dataset = H5Dopen2(group, "voxel_values", H5P_DEFAULT);
    if (voxel_values_dataset < 0) {
      H5Gclose(group);
      throw std::runtime_error("failed to open voxel_values dataset");
    }
    const std::vector<hsize_t> shape = datasetShape(voxel_values_dataset);
    if (shape.size() != 3U) {
      H5Dclose(voxel_values_dataset);
      H5Gclose(group);
      throw std::runtime_error("voxel_values dataset must be 3D");
    }

    const auto expected_x = static_cast<hsize_t>(dims_.x());
    const auto expected_y = static_cast<hsize_t>(dims_.y());
    const auto expected_z = static_cast<hsize_t>(dims_.z());

    // Dataset stored as [z, y, x]
    if (shape[2] != expected_x || shape[1] != expected_y || shape[0] != expected_z) {
      H5Dclose(voxel_values_dataset);
      H5Gclose(group);
      throw std::runtime_error("voxel_values shape mismatch");
    }

    const std::size_t total_voxels =
      static_cast<std::size_t>(dims_.x()) *
      static_cast<std::size_t>(dims_.y()) *
      static_cast<std::size_t>(dims_.z());
    voxel_values_.assign(total_voxels, 0U);
    status = H5Dread(
      voxel_values_dataset, H5T_NATIVE_UCHAR, H5S_ALL, H5S_ALL, H5P_DEFAULT, voxel_values_.data());
    H5Dclose(voxel_values_dataset);
    if (status < 0) {
      H5Gclose(group);
      throw std::runtime_error("failed to read voxel_values dataset");
    }

    stride_y_ = static_cast<std::size_t>(dims_.x());
    stride_z_ = stride_y_ * static_cast<std::size_t>(dims_.y());

    H5Gclose(group);
  } catch (const std::exception &) {
    ok = false;
  }

  H5Fclose(file);

  if (!ok) {
    resetState(voxel_size_, inv_voxel_size_, origin_, dims_, stride_y_, stride_z_, voxel_values_);
  }

  return ok && voxel_size_ > 0.0 && dims_.x() > 0 && dims_.y() > 0 && dims_.z() > 0 &&
         !voxel_values_.empty();
}

bool RawVoxelGridMap::isOccupied(double x, double y, double z) const
{
  if (voxel_size_ <= 0.0 || dims_.x() <= 0 || dims_.y() <= 0 || dims_.z() <= 0 ||
    voxel_values_.empty())
  {
    return false;
  }

  const Eigen::Vector3d rel(x - origin_.x(), y - origin_.y(), z - origin_.z());

  const auto to_index = [](double value, double inv_size) -> int {
      const double scaled = value * inv_size;
      return static_cast<int>(std::floor(scaled));
    };

  const int vx = to_index(rel.x(), inv_voxel_size_);
  const int vy = to_index(rel.y(), inv_voxel_size_);
  const int vz = to_index(rel.z(), inv_voxel_size_);

  if (vx < 0 || vy < 0 || vz < 0 ||
    vx >= dims_.x() || vy >= dims_.y() || vz >= dims_.z())
  {
    return false;
  }

  const std::size_t ux = static_cast<std::size_t>(vx);
  const std::size_t uy = static_cast<std::size_t>(vy);
  const std::size_t uz = static_cast<std::size_t>(vz);

  const std::size_t linear_index = ux + stride_y_ * uy + stride_z_ * uz;
  if (linear_index >= voxel_values_.size()) {
    return false;
  }

  return voxel_values_[linear_index] > 0U;
}

bool RawVoxelGridMap::isOccupied(const Eigen::Vector3d & position) const
{
  return isOccupied(position.x(), position.y(), position.z());
}

double RawVoxelGridMap::voxelSize() const
{
  return voxel_size_;
}

}  // namespace emcl2

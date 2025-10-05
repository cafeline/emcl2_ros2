#include "emcl2/CompressedVoxelMap.h"

#include <hdf5.h>

#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace emcl2
{
namespace
{

bool datasetExists(hid_t location, const char * name)
{
  return H5Lexists(location, name, H5P_DEFAULT) > 0;
}

bool readScalar(hid_t group, const char * name, hid_t type, void * out)
{
  if (!datasetExists(group, name)) {
    return false;
  }
  hid_t dataset = H5Dopen2(group, name, H5P_DEFAULT);
  if (dataset < 0) {
    return false;
  }
  herr_t status = H5Dread(dataset, type, H5S_ALL, H5S_ALL, H5P_DEFAULT, out);
  H5Dclose(dataset);
  return status >= 0;
}

std::vector<hsize_t> datasetShape(hid_t dataset)
{
  hid_t space = H5Dget_space(dataset);
  if (space < 0) {
    return {};
  }
  int ndims = H5Sget_simple_extent_ndims(space);
  std::vector<hsize_t> dims(static_cast<std::size_t>(ndims));
  H5Sget_simple_extent_dims(space, dims.data(), nullptr);
  H5Sclose(space);
  return dims;
}

bool readDataset(
  hid_t group, const char * name, hid_t type, std::vector<uint8_t> & buffer,
  std::vector<hsize_t> & shape)
{
  if (!datasetExists(group, name)) {
    return false;
  }
  hid_t dataset = H5Dopen2(group, name, H5P_DEFAULT);
  if (dataset < 0) {
    return false;
  }
  shape = datasetShape(dataset);
  if (shape.empty()) {
    H5Dclose(dataset);
    return false;
  }
  std::size_t total = 1;
  for (hsize_t d : shape) {
    total *= static_cast<std::size_t>(d);
  }
  buffer.resize(total * H5Tget_size(type));
  herr_t status = H5Dread(dataset, type, H5S_ALL, H5S_ALL, H5P_DEFAULT, buffer.data());
  H5Dclose(dataset);
  return status >= 0;
}

}  // namespace

bool CompressedVoxelMap::loadFromFile(const std::string & path)
{
  const auto reset_state = [&]() {
      block_indices_.clear();
      voxel_size_ = 0.0;
      inv_voxel_size_ = 0.0;
      block_size_ = 0;
      block_dims_ = Eigen::Vector3i::Zero();
      stride_y_ = 0;
      stride_z_ = 0;
    };

  reset_state();

  hid_t file = H5Fopen(path.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
  if (file < 0) {
    return false;
  }

  bool ok = true;

  try {
    // compression parameters
    hid_t compression = H5Gopen2(file, "/compression_params", H5P_DEFAULT);
    if (compression < 0) {
      throw std::runtime_error("compression_params group missing");
    }

    float voxel_size_f = 0.0f;
    if (!readScalar(compression, "voxel_size", H5T_NATIVE_FLOAT, &voxel_size_f)) {
      H5Gclose(compression);
      throw std::runtime_error("voxel_size missing");
    }
    voxel_size_ = static_cast<double>(voxel_size_f);
    inv_voxel_size_ = voxel_size_ > 0.0 ? 1.0 / voxel_size_ : 0.0;

    uint32_t block_size_u = 0;
    if (!readScalar(compression, "block_size", H5T_NATIVE_UINT32, &block_size_u)) {
      H5Gclose(compression);
      throw std::runtime_error("block_size missing");
    }
    block_size_ = static_cast<int>(block_size_u);

    std::vector<uint8_t> origin_buffer;
    std::vector<hsize_t> origin_shape;
    if (!readDataset(
        compression, "grid_origin", H5T_NATIVE_FLOAT, origin_buffer,
        origin_shape))
    {
      H5Gclose(compression);
      throw std::runtime_error("grid_origin missing");
    }
    if (origin_shape.size() != 1 || origin_shape[0] < 3) {
      H5Gclose(compression);
      throw std::runtime_error("grid_origin invalid shape");
    }
    const float * origin_ptr = reinterpret_cast<const float *>(origin_buffer.data());
    origin_ = Eigen::Vector3d(
      static_cast<double>(origin_ptr[0]),
      static_cast<double>(origin_ptr[1]),
      static_cast<double>(origin_ptr[2]));

    H5Gclose(compression);

    // compressed data
    hid_t compressed = H5Gopen2(file, "/compressed_data", H5P_DEFAULT);
    if (compressed < 0) {
      throw std::runtime_error("compressed_data group missing");
    }

    std::vector<uint8_t> dims_buffer;
    std::vector<hsize_t> dims_shape_vec;
    if (!readDataset(compressed, "block_dims", H5T_NATIVE_INT32, dims_buffer, dims_shape_vec)) {
      H5Gclose(compressed);
      throw std::runtime_error("block_dims dataset missing");
    }
    if (dims_shape_vec.size() != 1 || dims_shape_vec[0] != 3) {
      H5Gclose(compressed);
      throw std::runtime_error("block_dims invalid shape");
    }
    const int32_t * dims_ptr = reinterpret_cast<const int32_t *>(dims_buffer.data());
    block_dims_ = Eigen::Vector3i(dims_ptr[0], dims_ptr[1], dims_ptr[2]);
    if (block_dims_.x() <= 0 || block_dims_.y() <= 0 || block_dims_.z() <= 0) {
      H5Gclose(compressed);
      throw std::runtime_error("block_dims must be positive");
    }

    stride_y_ = static_cast<std::size_t>(block_dims_.x());
    stride_z_ = static_cast<std::size_t>(block_dims_.x()) *
      static_cast<std::size_t>(block_dims_.y());
    const std::size_t expected_total = static_cast<std::size_t>(block_dims_.x()) *
      static_cast<std::size_t>(block_dims_.y()) *
      static_cast<std::size_t>(block_dims_.z());

    hid_t indices_dataset = H5Dopen2(compressed, "block_indices", H5P_DEFAULT);
    if (indices_dataset < 0) {
      H5Gclose(compressed);
      throw std::runtime_error("block_indices dataset missing");
    }
    hid_t indices_space = H5Dget_space(indices_dataset);
    if (indices_space < 0) {
      H5Dclose(indices_dataset);
      H5Gclose(compressed);
      throw std::runtime_error("block_indices dataspace missing");
    }
    int ndims = H5Sget_simple_extent_ndims(indices_space);
    if (ndims != 1) {
      H5Sclose(indices_space);
      H5Dclose(indices_dataset);
      H5Gclose(compressed);
      throw std::runtime_error("block_indices invalid rank");
    }
    hsize_t dims[1];
    H5Sget_simple_extent_dims(indices_space, dims, nullptr);
    const std::size_t total_blocks = static_cast<std::size_t>(dims[0]);
    if (total_blocks == 0 || total_blocks != expected_total) {
      H5Sclose(indices_space);
      H5Dclose(indices_dataset);
      H5Gclose(compressed);
      throw std::runtime_error("block_indices size mismatch");
    }

    block_indices_.assign(total_blocks, 0);
    herr_t status = H5Dread(
      indices_dataset, H5T_NATIVE_UINT8, H5S_ALL, H5S_ALL, H5P_DEFAULT,
      block_indices_.data());

    H5Sclose(indices_space);
    H5Dclose(indices_dataset);

    if (status < 0) {
      H5Gclose(compressed);
      throw std::runtime_error("failed to read block_indices");
    }

    H5Gclose(compressed);

  } catch (const std::exception &) {
    ok = false;
  }

  H5Fclose(file);

  if (!ok) {
    reset_state();
  }

  return ok && voxel_size_ > 0.0 && block_size_ > 0 && !block_indices_.empty();
}

bool CompressedVoxelMap::isOccupied(double x, double y, double z) const
{
  if (voxel_size_ <= 0.0 || block_size_ <= 0 || block_indices_.empty()) {
    return false;
  }

  const Eigen::Vector3d rel(x - origin_.x(), y - origin_.y(), z - origin_.z());

  const auto fast_floor = [](double value) -> int64_t {
      int64_t truncated = static_cast<int64_t>(value);
      if (value < static_cast<double>(truncated)) {
        --truncated;
      }
      return truncated;
    };

  const double inv = inv_voxel_size_;
  const int64_t voxel_x = fast_floor(rel.x() * inv);
  const int64_t voxel_y = fast_floor(rel.y() * inv);
  const int64_t voxel_z = fast_floor(rel.z() * inv);

  const int64_t block_size_ll = static_cast<int64_t>(block_size_);
  const auto floor_div = [](int64_t numerator, int64_t denominator) -> int64_t {
      int64_t quotient = numerator / denominator;
      int64_t remainder = numerator % denominator;
      if (remainder < 0) {
        --quotient;
      }
      return quotient;
    };

  const int64_t block_x = floor_div(voxel_x, block_size_ll);
  const int64_t block_y = floor_div(voxel_y, block_size_ll);
  const int64_t block_z = floor_div(voxel_z, block_size_ll);

  const std::size_t ux = static_cast<std::size_t>(block_x);
  const std::size_t uy = static_cast<std::size_t>(block_y);
  const std::size_t uz = static_cast<std::size_t>(block_z);

  const std::size_t linear_index = ux + stride_y_ * uy + stride_z_ * uz;

  if (linear_index >= block_indices_.size()) {
    return false;
  }

  return block_indices_[linear_index];
}

bool CompressedVoxelMap::isOccupied(const Eigen::Vector3d & position) const
{
  return isOccupied(position.x(), position.y(), position.z());
}

double CompressedVoxelMap::voxelSize() const
{
  return voxel_size_;
}

}  // namespace emcl2

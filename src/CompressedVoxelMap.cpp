#include "emcl2/CompressedVoxelMap.h"

#include <hdf5.h>

#include <cmath>
#include <limits>
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
      dictionary_patterns_.clear();
      block_indices_.clear();
      voxel_size_ = 0.0;
      block_size_ = 0;
      pattern_length_ = 0;
      pattern_bytes_ = 0;
      block_offset_ = Eigen::Vector3i::Zero();
      block_dims_ = Eigen::Vector3i::Zero();
      block_index_sentinel_ = 0;
      block_index_bit_width_ = 0;
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

    uint32_t block_size_u = 0;
    if (!readScalar(compression, "block_size", H5T_NATIVE_UINT32, &block_size_u)) {
      H5Gclose(compression);
      throw std::runtime_error("block_size missing");
    }
    block_size_ = static_cast<int>(block_size_u);

    uint32_t pattern_bits_u = 0;
    if (!readScalar(compression, "pattern_bits", H5T_NATIVE_UINT32, &pattern_bits_u)) {
      H5Gclose(compression);
      throw std::runtime_error("pattern_bits missing");
    }
    pattern_length_ = static_cast<int>(pattern_bits_u);
    pattern_bytes_ = (pattern_length_ + 7) / 8;

    uint32_t block_index_bits_u = 0;
    if (readScalar(compression, "block_index_bit_width", H5T_NATIVE_UINT32, &block_index_bits_u)) {
      block_index_bit_width_ = static_cast<uint8_t>(block_index_bits_u);
    }

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

    // dictionary
    hid_t dictionary = H5Gopen2(file, "/dictionary", H5P_DEFAULT);
    if (dictionary < 0) {
      throw std::runtime_error("dictionary group missing");
    }

    uint32_t pattern_length_u = 0;
    if (!readScalar(dictionary, "pattern_length", H5T_NATIVE_UINT32, &pattern_length_u)) {
      H5Gclose(dictionary);
      throw std::runtime_error("pattern_length missing");
    }
    pattern_length_ = static_cast<int>(pattern_length_u);
    pattern_bytes_ = (pattern_length_ + 7) / 8;

    std::vector<uint8_t> pattern_buffer;
    std::vector<hsize_t> pattern_shape;
    if (!readDataset(dictionary, "patterns", H5T_NATIVE_UCHAR, pattern_buffer, pattern_shape)) {
      H5Gclose(dictionary);
      throw std::runtime_error("patterns dataset missing");
    }
    if (pattern_bytes_ <= 0) {
      H5Gclose(dictionary);
      throw std::runtime_error("pattern bytes invalid");
    }
    if (pattern_buffer.empty() ||
      pattern_buffer.size() % static_cast<std::size_t>(pattern_bytes_) != 0)
    {
      H5Gclose(dictionary);
      throw std::runtime_error("patterns size mismatch");
    }
    dictionary_patterns_.assign(pattern_buffer.begin(), pattern_buffer.end());
    H5Gclose(dictionary);

    // compressed data
    hid_t compressed = H5Gopen2(file, "/compressed_data", H5P_DEFAULT);
    if (compressed < 0) {
      throw std::runtime_error("compressed_data group missing");
    }

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
    if (total_blocks == 0) {
      H5Sclose(indices_space);
      H5Dclose(indices_dataset);
      H5Gclose(compressed);
      throw std::runtime_error("block_indices empty");
    }

    hid_t indices_type = H5Dget_type(indices_dataset);
    const std::size_t element_size = static_cast<std::size_t>(H5Tget_size(indices_type));

    block_indices_.assign(total_blocks, 0);
    herr_t status = -1;
    if (element_size == sizeof(uint8_t)) {
      std::vector<uint8_t> tmp(total_blocks, 0);
      status =
        H5Dread(indices_dataset, H5T_NATIVE_UINT8, H5S_ALL, H5S_ALL, H5P_DEFAULT, tmp.data());
      if (status >= 0) {
        for (std::size_t i = 0; i < total_blocks; ++i) {
          block_indices_[i] = static_cast<uint64_t>(tmp[i]);
        }
        if (block_index_bit_width_ == 0) {
          block_index_bit_width_ = 8;
        }
      }
    } else if (element_size == sizeof(uint16_t)) {
      std::vector<uint16_t> tmp(total_blocks, 0);
      status =
        H5Dread(indices_dataset, H5T_NATIVE_UINT16, H5S_ALL, H5S_ALL, H5P_DEFAULT, tmp.data());
      if (status >= 0) {
        for (std::size_t i = 0; i < total_blocks; ++i) {
          block_indices_[i] = static_cast<uint64_t>(tmp[i]);
        }
        if (block_index_bit_width_ == 0) {
          block_index_bit_width_ = 16;
        }
      }
    } else if (element_size == sizeof(uint32_t)) {
      std::vector<uint32_t> tmp(total_blocks, 0);
      status =
        H5Dread(indices_dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, tmp.data());
      if (status >= 0) {
        for (std::size_t i = 0; i < total_blocks; ++i) {
          block_indices_[i] = static_cast<uint64_t>(tmp[i]);
        }
        if (block_index_bit_width_ == 0) {
          block_index_bit_width_ = 32;
        }
      }
    } else if (element_size == sizeof(uint64_t)) {
      status = H5Dread(
        indices_dataset, H5T_NATIVE_UINT64, H5S_ALL, H5S_ALL, H5P_DEFAULT,
        block_indices_.data());
      if (status >= 0 && block_index_bit_width_ == 0) {
        block_index_bit_width_ = 64;
      }
    }

    H5Tclose(indices_type);
    H5Sclose(indices_space);
    H5Dclose(indices_dataset);

    if (status < 0) {
      H5Gclose(compressed);
      throw std::runtime_error("failed to read block_indices");
    }

    std::vector<uint8_t> offset_buffer;
    std::vector<hsize_t> offset_shape;
    if (!readDataset(compressed, "block_offset", H5T_NATIVE_INT32, offset_buffer, offset_shape)) {
      H5Gclose(compressed);
      throw std::runtime_error("block_offset dataset missing");
    }
    if (offset_shape.size() != 1 || offset_shape[0] != 3) {
      H5Gclose(compressed);
      throw std::runtime_error("block_offset invalid shape");
    }
    const int32_t * offset_ptr = reinterpret_cast<const int32_t *>(offset_buffer.data());
    block_offset_ = Eigen::Vector3i(offset_ptr[0], offset_ptr[1], offset_ptr[2]);

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

    H5Gclose(compressed);

    if (block_dims_.x() <= 0 || block_dims_.y() <= 0 || block_dims_.z() <= 0) {
      throw std::runtime_error("block_dims must be positive");
    }

    const std::size_t expected_total = static_cast<std::size_t>(block_dims_.x()) *
      static_cast<std::size_t>(block_dims_.y()) *
      static_cast<std::size_t>(block_dims_.z());

    if (expected_total != block_indices_.size()) {
      throw std::runtime_error("block_indices size mismatch");
    }

    stride_y_ = static_cast<int64_t>(block_dims_.x());
    stride_z_ = static_cast<int64_t>(block_dims_.x()) * static_cast<int64_t>(block_dims_.y());

    if (block_index_bit_width_ != 8 && block_index_bit_width_ != 16 &&
      block_index_bit_width_ != 32 && block_index_bit_width_ != 64)
    {
      throw std::runtime_error("unsupported block index bit width");
    }

    if (block_index_bit_width_ == 64) {
      block_index_sentinel_ = std::numeric_limits<uint64_t>::max();
    } else {
      block_index_sentinel_ = (1ULL << block_index_bit_width_) - 1ULL;
    }

  } catch (const std::exception &) {
    ok = false;
  }

  H5Fclose(file);

  if (!ok) {
    reset_state();
  }

  return ok && voxel_size_ > 0.0 && block_size_ > 0 && pattern_bytes_ > 0 &&
         !dictionary_patterns_.empty() && !block_indices_.empty();
}

bool CompressedVoxelMap::isOccupied(double x, double y, double z) const
{
  if (voxel_size_ <= 0.0 || block_size_ <= 0 || dictionary_patterns_.empty() ||
    block_indices_.empty())
  {
    return false;
  }

  Eigen::Vector3d rel(x - origin_.x(), y - origin_.y(), z - origin_.z());
  const double inv = 1.0 / voxel_size_;

  int64_t voxel_x = static_cast<int64_t>(std::floor(rel.x() * inv));
  int64_t voxel_y = static_cast<int64_t>(std::floor(rel.y() * inv));
  int64_t voxel_z = static_cast<int64_t>(std::floor(rel.z() * inv));

  const int64_t block_size_ll = static_cast<int64_t>(block_size_);
  const double block_size_d = static_cast<double>(block_size_ll);

  int64_t block_x = static_cast<int64_t>(std::floor(static_cast<double>(voxel_x) / block_size_d));
  int64_t block_y = static_cast<int64_t>(std::floor(static_cast<double>(voxel_y) / block_size_d));
  int64_t block_z = static_cast<int64_t>(std::floor(static_cast<double>(voxel_z) / block_size_d));

  const int64_t idx_x = block_x - static_cast<int64_t>(block_offset_.x());
  const int64_t idx_y = block_y - static_cast<int64_t>(block_offset_.y());
  const int64_t idx_z = block_z - static_cast<int64_t>(block_offset_.z());

  if (idx_x < 0 || idx_y < 0 || idx_z < 0 ||
    idx_x >= block_dims_.x() || idx_y >= block_dims_.y() || idx_z >= block_dims_.z())
  {
    return false;
  }

  const std::size_t linear_index = static_cast<std::size_t>(idx_x) +
    static_cast<std::size_t>(stride_y_) * static_cast<std::size_t>(idx_y) +
    static_cast<std::size_t>(stride_z_) * static_cast<std::size_t>(idx_z);

  if (linear_index >= block_indices_.size()) {
    return false;
  }

  const uint64_t pattern_index = block_indices_[linear_index];
  if (pattern_index == block_index_sentinel_) {
    return false;
  }

  const std::size_t offset = static_cast<std::size_t>(pattern_index) *
    static_cast<std::size_t>(pattern_bytes_);
  if (offset + static_cast<std::size_t>(pattern_bytes_) > dictionary_patterns_.size()) {
    return false;
  }

  const int local_x = static_cast<int>(voxel_x - block_x * block_size_ll);
  const int local_y = static_cast<int>(voxel_y - block_y * block_size_ll);
  const int local_z = static_cast<int>(voxel_z - block_z * block_size_ll);

  if (local_x < 0 || local_x >= block_size_ || local_y < 0 || local_y >= block_size_ ||
    local_z < 0 || local_z >= block_size_)
  {
    return false;
  }

  const int bit_index = local_z * block_size_ * block_size_ + local_y * block_size_ + local_x;
  const int byte_index = bit_index / 8;
  const int bit_in_byte = bit_index % 8;

  if (byte_index < 0 || byte_index >= pattern_bytes_) {
    return false;
  }

  const std::uint8_t * pattern = dictionary_patterns_.data() + offset;
  return (pattern[static_cast<std::size_t>(byte_index)] >> bit_in_byte) & 0x1;
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

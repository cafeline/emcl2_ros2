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
      dictionary_patterns_.clear();
      voxel_size_ = 0.0;
      inv_voxel_size_ = 0.0;
      block_size_ = 0;
      block_dims_ = Eigen::Vector3i::Zero();
      pattern_bits_ = 0;
      pattern_bytes_ = 0;
      dictionary_size_ = 0;
      block_index_bit_width_ = 1;
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

    uint32_t pattern_bits_u = 0;
    if (!readScalar(compression, "pattern_bits", H5T_NATIVE_UINT32, &pattern_bits_u)) {
      H5Gclose(compression);
      throw std::runtime_error("pattern_bits missing");
    }
    pattern_bits_ = static_cast<std::size_t>(pattern_bits_u);
    pattern_bytes_ = (pattern_bits_ + 7U) / 8U;
    if (pattern_bits_ == 0 || pattern_bytes_ == 0) {
      H5Gclose(compression);
      throw std::runtime_error("pattern_bits must be positive");
    }

    uint32_t dictionary_size_u = 0;
    if (readScalar(compression, "dictionary_size", H5T_NATIVE_UINT32, &dictionary_size_u)) {
      dictionary_size_ = static_cast<std::size_t>(dictionary_size_u);
    } else {
      dictionary_size_ = 0;
    }

    uint32_t bit_width_u = 1;
    if (readScalar(compression, "block_index_bit_width", H5T_NATIVE_UINT32, &bit_width_u)) {
      block_index_bit_width_ = static_cast<std::uint8_t>(bit_width_u);
    } else {
      block_index_bit_width_ = 1;
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

    hid_t dictionary = H5Gopen2(file, "/dictionary", H5P_DEFAULT);
    if (dictionary < 0) {
      throw std::runtime_error("dictionary group missing");
    }

    uint32_t pattern_length_u = 0;
    if (!readScalar(dictionary, "pattern_length", H5T_NATIVE_UINT32, &pattern_length_u)) {
      H5Gclose(dictionary);
      throw std::runtime_error("pattern_length missing");
    }
    const std::size_t pattern_length_sz = static_cast<std::size_t>(pattern_length_u);
    if (pattern_length_sz == 0) {
      H5Gclose(dictionary);
      throw std::runtime_error("pattern_length must be positive");
    }
    if (pattern_bits_ != 0 && pattern_bits_ != pattern_length_sz) {
      H5Gclose(dictionary);
      throw std::runtime_error("pattern_bits mismatch between compression params and dictionary");
    }
    pattern_bits_ = pattern_length_sz;
    pattern_bytes_ = (pattern_bits_ + 7U) / 8U;

    std::vector<uint8_t> pattern_buffer;
    std::vector<hsize_t> pattern_shape;
    if (!readDataset(dictionary, "patterns", H5T_NATIVE_UINT8, pattern_buffer, pattern_shape)) {
      H5Gclose(dictionary);
      throw std::runtime_error("patterns dataset missing");
    }
    H5Gclose(dictionary);

    if (pattern_bytes_ == 0) {
      throw std::runtime_error("pattern_bytes must be positive");
    }
    if (pattern_buffer.size() % pattern_bytes_ != 0) {
      throw std::runtime_error("patterns dataset size mismatch");
    }

    const std::size_t computed_dictionary_size = pattern_buffer.size() / pattern_bytes_;
    if (computed_dictionary_size == 0) {
      throw std::runtime_error("dictionary must contain at least one pattern");
    }
    if (dictionary_size_ != 0 && dictionary_size_ != computed_dictionary_size) {
      throw std::runtime_error("dictionary_size metadata mismatch");
    }
    dictionary_size_ = computed_dictionary_size;
    dictionary_patterns_ = std::move(pattern_buffer);

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
    const std::size_t packed_size = static_cast<std::size_t>(dims[0]);
    if (block_index_bit_width_ == 0) {
      H5Gclose(compressed);
      throw std::runtime_error("block_index_bit_width must be positive");
    }

    const std::size_t bits_per_index = static_cast<std::size_t>(block_index_bit_width_);
    if (bits_per_index > 32U) {
      H5Sclose(indices_space);
      H5Dclose(indices_dataset);
      H5Gclose(compressed);
      throw std::runtime_error("block_index_bit_width greater than 32 is unsupported");
    }

    const std::size_t expected_packed = (expected_total * bits_per_index + 7) / 8;
    if (packed_size == 0 || packed_size != expected_packed) {
      H5Sclose(indices_space);
      H5Dclose(indices_dataset);
      H5Gclose(compressed);
      throw std::runtime_error("block_indices size mismatch");
    }

    std::vector<uint8_t> packed(packed_size, 0);
    herr_t status = H5Dread(
      indices_dataset, H5T_NATIVE_UINT8, H5S_ALL, H5S_ALL, H5P_DEFAULT,
      packed.data());

    H5Sclose(indices_space);
    H5Dclose(indices_dataset);

    if (status < 0) {
      H5Gclose(compressed);
      throw std::runtime_error("failed to read block_indices");
    }

    block_indices_.assign(expected_total, 0U);
    const std::size_t total_bits = bits_per_index * expected_total;
    for (std::size_t index = 0; index < expected_total; ++index) {
      std::uint64_t value = 0;
      const std::size_t base_bit = index * bits_per_index;
      for (std::size_t bit = 0; bit < bits_per_index; ++bit) {
        const std::size_t absolute_bit = base_bit + bit;
        if (absolute_bit >= total_bits) {
          break;
        }
        const std::size_t byte_index = absolute_bit >> 3U;
        const std::size_t bit_index = absolute_bit & 7U;
        const uint8_t byte = packed[byte_index];
        const std::uint64_t bit_value = (static_cast<std::uint64_t>(byte) >> bit_index) & 0x1ULL;
        value |= (bit_value << bit);
      }
      block_indices_[index] = static_cast<std::uint32_t>(value);
    }

    H5Gclose(compressed);

  } catch (const std::exception &) {
    ok = false;
  }

  H5Fclose(file);

  if (!ok) {
    reset_state();
  }

  return ok && voxel_size_ > 0.0 && block_size_ > 0 &&
         pattern_bytes_ > 0 && dictionary_size_ > 0 &&
         !dictionary_patterns_.empty() && !block_indices_.empty();
}

bool CompressedVoxelMap::isOccupied(double x, double y, double z) const
{
  if (voxel_size_ <= 0.0 || block_size_ <= 0 || block_indices_.empty() ||
    dictionary_patterns_.empty() || pattern_bytes_ == 0)
  {
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

  if (block_x < 0 || block_y < 0 || block_z < 0 ||
    block_x >= block_dims_.x() || block_y >= block_dims_.y() ||
    block_z >= block_dims_.z())
  {
    return false;
  }

  const std::size_t ux = static_cast<std::size_t>(block_x);
  const std::size_t uy = static_cast<std::size_t>(block_y);
  const std::size_t uz = static_cast<std::size_t>(block_z);

  const std::size_t linear_index = ux + stride_y_ * uy + stride_z_ * uz;

  if (linear_index >= block_indices_.size()) {
    return false;
  }

  const std::uint32_t pattern_id = block_indices_[linear_index];
  if (pattern_id >= dictionary_size_) {
    return false;
  }

  const int64_t local_x = voxel_x - block_x * block_size_ll;
  const int64_t local_y = voxel_y - block_y * block_size_ll;
  const int64_t local_z = voxel_z - block_z * block_size_ll;

  if (local_x < 0 || local_y < 0 || local_z < 0 ||
    local_x >= block_size_ll || local_y >= block_size_ll || local_z >= block_size_ll)
  {
    return false;
  }

  const std::size_t lx = static_cast<std::size_t>(local_x);
  const std::size_t ly = static_cast<std::size_t>(local_y);
  const std::size_t lz = static_cast<std::size_t>(local_z);

  const std::size_t bit_index = lx + static_cast<std::size_t>(block_size_) *
    (ly + static_cast<std::size_t>(block_size_) * lz);
  const std::size_t byte_offset = static_cast<std::size_t>(pattern_id) * pattern_bytes_ +
    (bit_index >> 3);
  const std::uint8_t mask = static_cast<std::uint8_t>(1u << (bit_index & 7));

  if (byte_offset >= dictionary_patterns_.size()) {
    return false;
  }

  return (dictionary_patterns_[byte_offset] & mask) != 0;
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

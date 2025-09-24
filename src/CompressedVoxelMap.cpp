#include "emcl2/CompressedVoxelMap.h"

#include <hdf5.h>

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
  block_to_pattern_.clear();
  dictionary_patterns_.clear();
  voxel_size_ = 0.0;
  block_size_ = 0;
  pattern_length_ = 0;
  pattern_bytes_ = 0;

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

    std::vector<uint8_t> indices_buffer;
    std::vector<hsize_t> indices_shape;
    if (!readDataset(compressed, "indices", H5T_NATIVE_UINT16, indices_buffer, indices_shape)) {
      H5Gclose(compressed);
      throw std::runtime_error("indices dataset missing");
    }
    if (indices_shape.size() != 1) {
      H5Gclose(compressed);
      throw std::runtime_error("indices invalid shape");
    }
    std::size_t num_blocks = indices_shape[0];
    block_to_pattern_.clear();
    block_to_pattern_.reserve(num_blocks);

    const uint16_t * indices_ptr = reinterpret_cast<const uint16_t *>(indices_buffer.data());

    std::vector<uint8_t> positions_buffer;
    std::vector<hsize_t> positions_shape;
    if (!readDataset(
        compressed, "voxel_positions", H5T_NATIVE_INT32, positions_buffer,
        positions_shape))
    {
      H5Gclose(compressed);
      throw std::runtime_error("voxel_positions dataset missing");
    }
    if (positions_shape.size() != 2 || positions_shape[0] != num_blocks ||
      positions_shape[1] != 3)
    {
      H5Gclose(compressed);
      throw std::runtime_error("voxel_positions shape mismatch");
    }
    const int32_t * positions_ptr = reinterpret_cast<const int32_t *>(positions_buffer.data());

    for (std::size_t i = 0; i < num_blocks; ++i) {
      BlockCoord coord{positions_ptr[i * 3 + 0], positions_ptr[i * 3 + 1],
        positions_ptr[i * 3 + 2]};
      block_to_pattern_[coord] = indices_ptr[i];
    }
    H5Gclose(compressed);
  } catch (const std::exception &) {
    ok = false;
  }

  H5Fclose(file);

  if (!ok) {
    block_to_pattern_.clear();
    dictionary_patterns_.clear();
    voxel_size_ = 0.0;
    block_size_ = 0;
    pattern_length_ = 0;
    pattern_bytes_ = 0;
  }

  return ok && voxel_size_ > 0.0 && block_size_ > 0 && !dictionary_patterns_.empty();
}

bool CompressedVoxelMap::isOccupied(double x, double y, double z) const
{
  if (voxel_size_ <= 0.0 || block_size_ <= 0 || dictionary_patterns_.empty()) {
    return false;
  }

  Eigen::Vector3d rel(x - origin_.x(), y - origin_.y(), z - origin_.z());
  double inv = 1.0 / voxel_size_;
  double vx_d = rel.x() * inv;
  double vy_d = rel.y() * inv;
  double vz_d = rel.z() * inv;

  int64_t voxel_x = static_cast<int64_t>(std::floor(vx_d));
  int64_t voxel_y = static_cast<int64_t>(std::floor(vy_d));
  int64_t voxel_z = static_cast<int64_t>(std::floor(vz_d));

  int64_t block_size_ll = static_cast<int64_t>(block_size_);

  int64_t block_x = static_cast<int64_t>(std::floor(static_cast<double>(voxel_x) / block_size_ll));
  int64_t block_y = static_cast<int64_t>(std::floor(static_cast<double>(voxel_y) / block_size_ll));
  int64_t block_z = static_cast<int64_t>(std::floor(static_cast<double>(voxel_z) / block_size_ll));

  BlockCoord coord{static_cast<int32_t>(block_x), static_cast<int32_t>(block_y),
    static_cast<int32_t>(block_z)};
  auto it = block_to_pattern_.find(coord);
  if (it == block_to_pattern_.end()) {
    return false;
  }

  std::uint16_t pattern_index = it->second;
  std::size_t offset = static_cast<std::size_t>(pattern_index) *
    static_cast<std::size_t>(pattern_bytes_);
  if (offset + static_cast<std::size_t>(pattern_bytes_) > dictionary_patterns_.size()) {
    return false;
  }

  int local_x = static_cast<int>(voxel_x - block_x * block_size_ll);
  int local_y = static_cast<int>(voxel_y - block_y * block_size_ll);
  int local_z = static_cast<int>(voxel_z - block_z * block_size_ll);

  if (local_x < 0 || local_x >= block_size_ || local_y < 0 || local_y >= block_size_ ||
    local_z < 0 ||
    local_z >= block_size_)
  {
    return false;
  }

  int bit_index = local_z * block_size_ * block_size_ + local_y * block_size_ + local_x;
  int byte_index = bit_index / 8;
  int bit_in_byte = bit_index % 8;

  const std::uint8_t * pattern = dictionary_patterns_.data() + offset;
  if (byte_index < 0 || byte_index >= pattern_bytes_) {
    return false;
  }

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

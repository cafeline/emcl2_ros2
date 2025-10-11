#include "emcl2/HashedVoxelMap.h"

#include "emcl2/detail/cnpy.h"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace emcl2
{
namespace
{

constexpr int kSdfBlockSize = 8;
constexpr int kVoxelsPerBlock = kSdfBlockSize * kSdfBlockSize * kSdfBlockSize;
constexpr int kFreeEntry = -2;

constexpr int kHashPrime0 = 73856093;
constexpr int kHashPrime1 = 19349669;
constexpr int kHashPrime2 = 83492791;

int computeHash(int x, int y, int z, std::uint32_t hash_size)
{
  if (hash_size == 0U) {
    return 0;
  }
  int value = ((x * kHashPrime0) ^ (y * kHashPrime1) ^ (z * kHashPrime2)) %
    static_cast<int>(hash_size);
  if (value < 0) {
    value += static_cast<int>(hash_size);
  }
  return value;
}

template<typename T>
const T * asTypedData(const cnpy::NpyArray & array)
{
  if (array.word_size != sizeof(T)) {
    throw std::runtime_error("dtype size mismatch");
  }
  if (!array.data.data()) {
    throw std::runtime_error("array data is null");
  }
  return reinterpret_cast<const T *>(array.data.data());
}

bool toInt32(int64_t value, int32_t & out)
{
  if (value < std::numeric_limits<int32_t>::min() ||
    value > std::numeric_limits<int32_t>::max())
  {
    return false;
  }
  out = static_cast<int32_t>(value);
  return true;
}

}  // namespace

class HashedVoxelMap::Impl
{
public:
  void reset()
  {
    voxel_size_ = 0.0;
    inv_voxel_size_ = 0.0;
    origin_ = Eigen::Vector3d::Zero();
    hash_size_ = 0U;
    hash_bucket_size_ = 0U;
    total_entries_ = 0U;
    block_count_ = 0U;
    hash_pos_.clear();
    hash_ptr_.clear();
    hash_offset_.clear();
    weights_.clear();
  }

  bool load(const std::string & path)
  {
    reset();

    cnpy::npz_t arrays;
    try {
      arrays = cnpy::npz_load_all(path);
    } catch (const std::exception &) {
      return false;
    }

    try {
      const int64_t hash_size_i64 = loadScalar<int64_t>(arrays, "hash_size.npy");
      const int64_t hash_bucket_size_i64 = loadScalar<int64_t>(arrays, "hash_bucket_size.npy");
      if (hash_size_i64 <= 0 || hash_bucket_size_i64 <= 0) {
        throw std::runtime_error("hash_size or hash_bucket_size must be positive");
      }
      if (hash_size_i64 > static_cast<int64_t>(std::numeric_limits<std::uint32_t>::max()) ||
        hash_bucket_size_i64 > static_cast<int64_t>(std::numeric_limits<std::uint32_t>::max()))
      {
        throw std::runtime_error("hash parameters exceed supported range");
      }
      hash_size_ = static_cast<std::uint32_t>(hash_size_i64);
      hash_bucket_size_ = static_cast<std::uint32_t>(hash_bucket_size_i64);

      if (hash_size_ == 0U || hash_bucket_size_ == 0U) {
        throw std::runtime_error("hash_size or hash_bucket_size must be positive");
      }

      total_entries_ = hash_size_ * hash_bucket_size_;

      loadVector(arrays, "hash_entries_pos.npy", hash_pos_, 3U);
      loadVector(arrays, "hash_entries_ptr.npy", hash_ptr_, 1U);
      loadVector(arrays, "hash_entries_offset.npy", hash_offset_, 1U);

      if (hash_pos_.size() != total_entries_ * 3U ||
        hash_ptr_.size() != total_entries_ ||
        hash_offset_.size() != total_entries_)
      {
        throw std::runtime_error("hash entry array sizes mismatch");
      }

      loadWeights(arrays);

      Eigen::Vector3f origin_f = loadVec3<float>(arrays, "origin.npy");
      origin_ = origin_f.cast<double>();
      float voxel_size_f = loadScalar<float>(arrays, "voxel_size.npy");
      if (!(voxel_size_f > 0.0f)) {
        throw std::runtime_error("voxel_size must be positive");
      }
      voxel_size_ = static_cast<double>(voxel_size_f);
      inv_voxel_size_ = 1.0 / voxel_size_;

      // optional metadata
      if (arrays.find("dimensions.npy") != arrays.end()) {
        loadVec3<int64_t>(arrays, "dimensions.npy");
      }

      return true;
    } catch (const std::exception & ex) {
      reset();
      std::cerr << "HashedVoxelMap: " << ex.what() << std::endl;
      return false;
    }
  }

  bool isOccupied(double x, double y, double z) const
  {
    if (voxel_size_ <= 0.0 || hash_size_ == 0U || hash_bucket_size_ == 0U ||
      total_entries_ == 0U || block_count_ == 0U)
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

    const int64_t voxel_x = fast_floor(rel.x() * inv_voxel_size_);
    const int64_t voxel_y = fast_floor(rel.y() * inv_voxel_size_);
    const int64_t voxel_z = fast_floor(rel.z() * inv_voxel_size_);

    const auto floor_div = [](int64_t numerator, int64_t denominator) -> int64_t {
        int64_t quotient = numerator / denominator;
        int64_t remainder = numerator % denominator;
        if (remainder < 0) {
          --quotient;
        }
        return quotient;
      };

    const int64_t block_x_64 = floor_div(voxel_x, kSdfBlockSize);
    const int64_t block_y_64 = floor_div(voxel_y, kSdfBlockSize);
    const int64_t block_z_64 = floor_div(voxel_z, kSdfBlockSize);

    int32_t block_x = 0;
    int32_t block_y = 0;
    int32_t block_z = 0;
    if (!toInt32(block_x_64, block_x) ||
      !toInt32(block_y_64, block_y) ||
      !toInt32(block_z_64, block_z))
    {
      return false;
    }

    std::size_t ptr_index = 0;
    if (!findBlock(block_x, block_y, block_z, ptr_index)) {
      return false;
    }
    const std::size_t block_index = ptr_index / kVoxelsPerBlock;
    if (block_index >= block_count_) {
      return false;
    }

    const int64_t local_x = voxel_x - static_cast<int64_t>(block_x) * kSdfBlockSize;
    const int64_t local_y = voxel_y - static_cast<int64_t>(block_y) * kSdfBlockSize;
    const int64_t local_z = voxel_z - static_cast<int64_t>(block_z) * kSdfBlockSize;

    if (local_x < 0 || local_y < 0 || local_z < 0 ||
      local_x >= kSdfBlockSize || local_y >= kSdfBlockSize || local_z >= kSdfBlockSize)
    {
      return false;
    }

    const std::size_t lx = static_cast<std::size_t>(local_x);
    const std::size_t ly = static_cast<std::size_t>(local_y);
    const std::size_t lz = static_cast<std::size_t>(local_z);

    const std::size_t voxel_index = block_index * kVoxelsPerBlock +
      lx + static_cast<std::size_t>(kSdfBlockSize) *
      (ly + static_cast<std::size_t>(kSdfBlockSize) * lz);

    if (voxel_index >= weights_.size()) {
      return false;
    }

    return weights_[voxel_index] > 0U;
  }

  double voxelSize() const
  {
    return voxel_size_;
  }

private:
  template<typename T>
  void loadScalar(
    const cnpy::npz_t & arrays, const std::string & name,
    T & out)
  {
    auto it = arrays.find(name);
    if (it == arrays.end()) {
      throw std::runtime_error("missing dataset: " + name);
    }
    const auto & array = it->second;
    if (array.shape.empty()) {
      throw std::runtime_error("scalar dataset has empty shape: " + name);
    }
    std::size_t total = 1U;
    for (std::size_t dim : array.shape) {
      total *= dim;
    }
    if (total != 1U) {
      throw std::runtime_error("scalar dataset has multiple entries: " + name);
    }
    const T * ptr = asTypedData<T>(array);
    out = ptr[0];
  }

  template<typename T>
  T loadScalar(
    const cnpy::npz_t & arrays, const std::string & name)
  {
    T value{};
    loadScalar(arrays, name, value);
    return value;
  }

  template<typename T>
  Eigen::Matrix<T, 3, 1> loadVec3(
    const cnpy::npz_t & arrays, const std::string & name)
  {
    auto it = arrays.find(name);
    if (it == arrays.end()) {
      throw std::runtime_error("missing dataset: " + name);
    }
    const auto & array = it->second;
    if (array.shape.size() != 1U || array.shape[0] < 3U) {
      throw std::runtime_error("vector dataset invalid shape: " + name);
    }
    const T * ptr = asTypedData<T>(array);
    Eigen::Matrix<T, 3, 1> vec;
    vec.x() = ptr[0];
    vec.y() = ptr[1];
    vec.z() = ptr[2];
    return vec;
  }

  template<typename T>
  void loadVector(
    const cnpy::npz_t & arrays, const std::string & name,
    std::vector<T> & out, std::size_t tuple)
  {
    auto it = arrays.find(name);
    if (it == arrays.end()) {
      throw std::runtime_error("missing dataset: " + name);
    }
    const auto & array = it->second;
    if (array.shape.empty()) {
      throw std::runtime_error("array dataset invalid shape: " + name);
    }
    std::size_t total = 1U;
    for (std::size_t dim : array.shape) {
      total *= dim;
    }
    const T * ptr = asTypedData<T>(array);
    out.assign(ptr, ptr + total);

    if (tuple > 1U && (total % tuple) != 0U) {
      throw std::runtime_error("array dataset tuple mismatch: " + name);
    }
  }

  void loadWeights(const cnpy::npz_t & arrays)
  {
    auto it = arrays.find("sdf_blocks.npy");
    if (it == arrays.end()) {
      throw std::runtime_error("missing dataset: sdf_blocks");
    }
    const auto & array = it->second;
    if (array.shape.size() != 3U ||
      array.shape[1] != static_cast<std::size_t>(kVoxelsPerBlock) ||
      array.shape[2] != 2U)
    {
      throw std::runtime_error("sdf_blocks has unexpected shape");
    }
    const int32_t * ptr = asTypedData<int32_t>(array);
    block_count_ = static_cast<std::uint32_t>(array.shape[0]);
    if (block_count_ == 0U) {
      throw std::runtime_error("sdf_blocks must contain at least one block");
    }
    weights_.assign(static_cast<std::size_t>(block_count_) * kVoxelsPerBlock, 0U);

    const std::size_t block_stride = static_cast<std::size_t>(kVoxelsPerBlock) * 2U;
    for (std::size_t block = 0; block < block_count_; ++block) {
      const int32_t * base = ptr + block * block_stride;
      for (int voxel = 0; voxel < kVoxelsPerBlock; ++voxel) {
        const int32_t packed_int = base[voxel * 2 + 1];
        const std::uint32_t packed = static_cast<std::uint32_t>(packed_int);
        const std::uint8_t weight = static_cast<std::uint8_t>(packed & 0xFFu);
        weights_[block * kVoxelsPerBlock + static_cast<std::size_t>(voxel)] = weight;
      }
    }
  }

  bool findBlock(
    int32_t block_x, int32_t block_y, int32_t block_z,
    std::size_t & ptr_index) const
  {
    const int hash = computeHash(block_x, block_y, block_z, hash_size_);
    const std::size_t base = static_cast<std::size_t>(hash) * hash_bucket_size_;

    for (std::size_t offset = 0; offset < hash_bucket_size_; ++offset) {
      const std::size_t idx = base + offset;
      const int32_t ptr = hash_ptr_[idx];
      if (ptr == kFreeEntry) {
        if (offset < hash_bucket_size_ - 1U) {
          continue;
        }
      }
      const int32_t * pos = &hash_pos_[idx * 3U];
      if (ptr != kFreeEntry &&
        pos[0] == block_x && pos[1] == block_y && pos[2] == block_z)
      {
        ptr_index = static_cast<std::size_t>(ptr);
        return true;
      }
    }

    std::size_t idx_last = base + hash_bucket_size_ - 1U;
    std::size_t visited = 0U;
    std::size_t total_slots = hash_ptr_.size();
    std::uint32_t offset = hash_offset_[idx_last];

    while (offset != 0U && visited < total_slots) {
      idx_last = (idx_last + offset) % total_slots;
      const int32_t ptr = hash_ptr_[idx_last];
      const int32_t * pos = &hash_pos_[idx_last * 3U];
      if (ptr != kFreeEntry &&
        pos[0] == block_x && pos[1] == block_y && pos[2] == block_z)
      {
        ptr_index = static_cast<std::size_t>(ptr);
        return true;
      }
      offset = hash_offset_[idx_last];
      ++visited;
    }

    return false;
  }

  double voxel_size_ {0.0};
  double inv_voxel_size_ {0.0};
  Eigen::Vector3d origin_ {Eigen::Vector3d::Zero()};

  std::uint32_t hash_size_ {0U};
  std::uint32_t hash_bucket_size_ {0U};
  std::size_t total_entries_ {0U};
  std::uint32_t block_count_ {0U};

  std::vector<int32_t> hash_pos_;
  std::vector<int32_t> hash_ptr_;
  std::vector<std::uint32_t> hash_offset_;
  std::vector<std::uint8_t> weights_;
};

HashedVoxelMap::HashedVoxelMap()
: impl_(std::make_shared<Impl>())
{
}

HashedVoxelMap::~HashedVoxelMap() = default;

HashedVoxelMap::HashedVoxelMap(const HashedVoxelMap &) = default;
HashedVoxelMap & HashedVoxelMap::operator=(const HashedVoxelMap &) = default;
HashedVoxelMap::HashedVoxelMap(HashedVoxelMap &&) noexcept = default;
HashedVoxelMap & HashedVoxelMap::operator=(HashedVoxelMap &&) noexcept = default;

bool HashedVoxelMap::loadFromFile(const std::string & path)
{
  return impl_->load(path);
}

bool HashedVoxelMap::isOccupied(double x, double y, double z) const
{
  return impl_->isOccupied(x, y, z);
}

bool HashedVoxelMap::isOccupied(const Eigen::Vector3d & position) const
{
  return isOccupied(position.x(), position.y(), position.z());
}

double HashedVoxelMap::voxelSize() const
{
  return impl_->voxelSize();
}

}  // namespace emcl2

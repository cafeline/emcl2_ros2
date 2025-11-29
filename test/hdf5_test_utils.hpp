// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2_TEST_HDF5_TEST_UTILS_HPP_
#define EMCL2_TEST_HDF5_TEST_UTILS_HPP_

#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <hdf5.h>

namespace emcl2_test
{

inline void write_scalar_uint32(hid_t group, const char * name, uint32_t value)
{
  hsize_t dims[1] = {1};
  hid_t space = H5Screate_simple(1, dims, nullptr);
  hid_t dataset = H5Dcreate2(
    group, name, H5T_NATIVE_UINT32, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  H5Dwrite(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &value);
  H5Dclose(dataset);
  H5Sclose(space);
}

inline void write_scalar_float(hid_t group, const char * name, float value)
{
  hsize_t dims[1] = {1};
  hid_t space = H5Screate_simple(1, dims, nullptr);
  hid_t dataset = H5Dcreate2(
    group, name, H5T_NATIVE_FLOAT, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  H5Dwrite(dataset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, &value);
  H5Dclose(dataset);
  H5Sclose(space);
}

inline void write_array_float(hid_t group, const char * name, const float * data, hsize_t length)
{
  hsize_t dims[1] = {length};
  hid_t space = H5Screate_simple(1, dims, nullptr);
  hid_t dataset = H5Dcreate2(
    group, name, H5T_NATIVE_FLOAT, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  H5Dwrite(dataset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
  H5Dclose(dataset);
  H5Sclose(space);
}

inline void write_array_uint8(
  hid_t group, const char * name, const unsigned char * data,
  hsize_t length)
{
  hsize_t dims[1] = {length};
  hid_t space = H5Screate_simple(1, dims, nullptr);
  hid_t dataset = H5Dcreate2(
    group, name, H5T_NATIVE_UCHAR, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  H5Dwrite(dataset, H5T_NATIVE_UCHAR, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
  H5Dclose(dataset);
  H5Sclose(space);
}

inline void write_array_uint16(
  hid_t group, const char * name, const uint16_t * data,
  hsize_t length)
{
  hsize_t dims[1] = {length};
  hid_t space = H5Screate_simple(1, dims, nullptr);
  hid_t dataset = H5Dcreate2(
    group, name, H5T_NATIVE_UINT16, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  H5Dwrite(dataset, H5T_NATIVE_UINT16, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
  H5Dclose(dataset);
  H5Sclose(space);
}

inline void write_array_int32(
  hid_t group, const char * name, const int32_t * data,
  hsize_t length)
{
  hsize_t dims[1] = {length};
  hid_t space = H5Screate_simple(1, dims, nullptr);
  hid_t dataset = H5Dcreate2(
    group, name, H5T_NATIVE_INT32, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  H5Dwrite(dataset, H5T_NATIVE_INT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
  H5Dclose(dataset);
  H5Sclose(space);
}

inline void write_matrix_int32(
  hid_t group, const char * name, const int32_t * data, hsize_t rows,
  hsize_t cols)
{
  hsize_t dims[2] = {rows, cols};
  hid_t space = H5Screate_simple(2, dims, nullptr);
  hid_t dataset = H5Dcreate2(
    group, name, H5T_NATIVE_INT32, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  H5Dwrite(dataset, H5T_NATIVE_INT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
  H5Dclose(dataset);
  H5Sclose(space);
}

inline std::vector<uint8_t> pack_block_indices(
  const std::vector<uint32_t> & indices,
  uint32_t bit_width)
{
  const std::size_t total_bits = static_cast<std::size_t>(bit_width) * indices.size();
  std::vector<uint8_t> packed((total_bits + 7U) / 8U, 0);

  for (std::size_t i = 0; i < indices.size(); ++i) {
    const uint32_t value = indices[i];
    for (uint32_t bit = 0; bit < bit_width; ++bit) {
      const std::size_t absolute_bit = i * static_cast<std::size_t>(bit_width) + bit;
      const std::size_t byte_index = absolute_bit >> 3U;
      const std::size_t bit_index = absolute_bit & 7U;
      const uint8_t bit_value = static_cast<uint8_t>((value >> bit) & 0x1U);
      packed[byte_index] = static_cast<uint8_t>(packed[byte_index] | (bit_value << bit_index));
    }
  }

  return packed;
}

inline std::string create_basic_hdf5_map(const std::string & filename)
{
  const auto path = (std::filesystem::temp_directory_path() / filename).string();

  hid_t file = H5Fcreate(path.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);

  hid_t compression =
    H5Gcreate2(file, "/compression_params", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  write_scalar_float(compression, "voxel_size", 1.0f);
  write_scalar_uint32(compression, "block_size", 2U);
  write_scalar_uint32(compression, "pattern_bits", 8U);
  write_scalar_uint32(compression, "dictionary_size", 3U);
  write_scalar_uint32(compression, "block_index_bit_width", 2U);
  const float origin[3] = {0.0f, 0.0f, 0.0f};
  write_array_float(compression, "grid_origin", origin, 3);
  H5Gclose(compression);

  hid_t dictionary = H5Gcreate2(file, "/dictionary", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  write_scalar_uint32(dictionary, "pattern_length", 8U);
  const unsigned char patterns[3] = {0x00, 0x01, 0x08};
  write_array_uint8(dictionary, "patterns", patterns, 3);
  H5Gclose(dictionary);

  hid_t compressed = H5Gcreate2(file, "/compressed_data", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  const auto block_indices = pack_block_indices({1U}, 2U);
  write_array_uint8(compressed, "block_indices", block_indices.data(), block_indices.size());
  const int32_t block_dims[3] = {1, 1, 1};
  write_array_int32(compressed, "block_dims", block_dims, 3);
  H5Gclose(compressed);

  H5Fclose(file);
  return path;
}

inline std::string create_offset_hdf5_map(const std::string & filename)
{
  const auto path = (std::filesystem::temp_directory_path() / filename).string();

  hid_t file = H5Fcreate(path.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);

  hid_t compression =
    H5Gcreate2(file, "/compression_params", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  write_scalar_float(compression, "voxel_size", 1.0f);
  write_scalar_uint32(compression, "block_size", 2U);
  write_scalar_uint32(compression, "pattern_bits", 8U);
  write_scalar_uint32(compression, "dictionary_size", 3U);
  write_scalar_uint32(compression, "block_index_bit_width", 2U);
  const float origin[3] = {0.0f, 0.0f, 0.0f};
  write_array_float(compression, "grid_origin", origin, 3);
  H5Gclose(compression);

  hid_t dictionary = H5Gcreate2(file, "/dictionary", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  write_scalar_uint32(dictionary, "pattern_length", 8U);
  const unsigned char patterns[3] = {0x00, 0x01, 0x08};
  write_array_uint8(dictionary, "patterns", patterns, 3);
  H5Gclose(dictionary);

  hid_t compressed = H5Gcreate2(file, "/compressed_data", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  const auto block_indices = pack_block_indices({0U, 2U}, 2U);
  write_array_uint8(compressed, "block_indices", block_indices.data(), block_indices.size());
  const int32_t block_dims[3] = {2, 1, 1};
  write_array_int32(compressed, "block_dims", block_dims, 3);
  H5Gclose(compressed);

  H5Fclose(file);
  return path;
}

}  // namespace emcl2_test

#endif  // EMCL2_TEST_HDF5_TEST_UTILS_HPP_

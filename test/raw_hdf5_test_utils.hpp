// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2_TEST_RAW_HDF5_TEST_UTILS_HPP_
#define EMCL2_TEST_RAW_HDF5_TEST_UTILS_HPP_

#include <array>
#include <cstdint>
#include <filesystem>
#include <iterator>
#include <string>
#include <vector>

#include <hdf5.h>

namespace emcl2_test
{

inline void write_scalar_float(hid_t group, const char * name, float value)
{
  const hsize_t dims[1] = {1};
  hid_t space = H5Screate_simple(1, dims, nullptr);
  hid_t dataset = H5Dcreate2(
    group, name, H5T_NATIVE_FLOAT, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  H5Dwrite(dataset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, &value);
  H5Dclose(dataset);
  H5Sclose(space);
}

inline void write_array_uint32(hid_t group, const char * name, const uint32_t * data, hsize_t len)
{
  const hsize_t dims[1] = {len};
  hid_t space = H5Screate_simple(1, dims, nullptr);
  hid_t dataset = H5Dcreate2(
    group, name, H5T_NATIVE_UINT32, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  H5Dwrite(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
  H5Dclose(dataset);
  H5Sclose(space);
}

inline void write_array_float(hid_t group, const char * name, const float * data, hsize_t len)
{
  const hsize_t dims[1] = {len};
  hid_t space = H5Screate_simple(1, dims, nullptr);
  hid_t dataset = H5Dcreate2(
    group, name, H5T_NATIVE_FLOAT, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  H5Dwrite(dataset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
  H5Dclose(dataset);
  H5Sclose(space);
}

inline void write_matrix_int32(
  hid_t group, const char * name, const int32_t * data, hsize_t rows, hsize_t cols)
{
  const hsize_t dims[2] = {rows, cols};
  hid_t space = H5Screate_simple(2, dims, nullptr);
  hid_t dataset = H5Dcreate2(
    group, name, H5T_NATIVE_INT32, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  H5Dwrite(dataset, H5T_NATIVE_INT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
  H5Dclose(dataset);
  H5Sclose(space);
}

inline void write_voxel_values(
  hid_t group, const char * name,
  const std::vector<uint8_t> & values,
  const std::array<uint32_t, 3> & dims)
{
  const hsize_t hdims[3] = {
    static_cast<hsize_t>(dims[2]),
    static_cast<hsize_t>(dims[1]),
    static_cast<hsize_t>(dims[0])
  };
  hid_t space = H5Screate_simple(3, hdims, nullptr);
  hid_t dataset = H5Dcreate2(
    group, name, H5T_NATIVE_UCHAR, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  if (!values.empty()) {
    H5Dwrite(dataset, H5T_NATIVE_UCHAR, H5S_ALL, H5S_ALL, H5P_DEFAULT, values.data());
  }
  H5Dclose(dataset);
  H5Sclose(space);
}

inline std::string create_basic_raw_hdf5_map(const std::string & filename)
{
  const auto path = (std::filesystem::temp_directory_path() / filename).string();

  hid_t file = H5Fcreate(path.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
  hid_t raw_group = H5Gcreate2(file, "/raw_voxel_grid", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

  const std::array<uint32_t, 3> dims = {2U, 2U, 1U};  // x, y, z
  write_array_uint32(raw_group, "dimensions", dims.data(), 3);
  write_scalar_float(raw_group, "voxel_size", 1.0f);
  const float origin[3] = {0.0f, 0.0f, 0.0f};
  write_array_float(raw_group, "origin", origin, 3);

  // Occupied voxels: use dictionary-style compatibility (x,y,z rows)
  const int32_t occupied[][3] = {
    {0, 0, 0},
    {1, 1, 0}
  };
  write_matrix_int32(
    raw_group, "occupied_voxels", &occupied[0][0],
    static_cast<hsize_t>(std::size(occupied)), 3);

  // voxel_values flattened: iterate z, y, x (matching producer)
  // Layout for dims (x=2,y=2,z=1):
  // z=0:
  //   y=0: [255,   0]
  //   y=1: [  0, 255]
  std::vector<uint8_t> voxel_values = {
    255U, 0U,
    0U, 255U
  };
  write_voxel_values(raw_group, "voxel_values", voxel_values, dims);

  H5Gclose(raw_group);
  H5Fclose(file);
  return path;
}

inline std::string create_offset_raw_hdf5_map(const std::string & filename)
{
  const auto path = (std::filesystem::temp_directory_path() / filename).string();

  hid_t file = H5Fcreate(path.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
  hid_t raw_group = H5Gcreate2(file, "/raw_voxel_grid", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

  const std::array<uint32_t, 3> dims = {3U, 1U, 1U};  // x, y, z
  write_array_uint32(raw_group, "dimensions", dims.data(), 3);
  write_scalar_float(raw_group, "voxel_size", 0.5f);
  const float origin[3] = {1.0f, -1.0f, 0.0f};
  write_array_float(raw_group, "origin", origin, 3);

  const int32_t occupied[][3] = {
    {1, 0, 0}
  };
  write_matrix_int32(
    raw_group, "occupied_voxels", &occupied[0][0],
    static_cast<hsize_t>(std::size(occupied)), 3);

  std::vector<uint8_t> voxel_values = {
    0U, 255U, 0U
  };
  write_voxel_values(raw_group, "voxel_values", voxel_values, dims);

  H5Gclose(raw_group);
  H5Fclose(file);
  return path;
}

}  // namespace emcl2_test

#endif  // EMCL2_TEST_RAW_HDF5_TEST_UTILS_HPP_

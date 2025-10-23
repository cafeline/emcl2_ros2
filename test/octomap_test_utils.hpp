// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2_TEST_OCTOMAP_TEST_UTILS_HPP_
#define EMCL2_TEST_OCTOMAP_TEST_UTILS_HPP_

#include <Eigen/Core>

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <octomap/OcTree.h>

#include "emcl2/OctomapMap.h"

namespace emcl2_test
{

inline std::string create_octomap_file(
  const std::string & filename, const std::vector<Eigen::Vector3d> & occupied_points,
  double resolution = 0.5)
{
  const auto path = (std::filesystem::temp_directory_path() / filename).string();

  octomap::OcTree tree(resolution);
  for (const auto & point : occupied_points) {
    tree.updateNode(
      octomap::point3d(
        static_cast<float>(point.x()),
        static_cast<float>(point.y()),
        static_cast<float>(point.z())),
      true);
  }
  tree.updateInnerOccupancy();
  tree.writeBinary(path);

  return path;
}

inline emcl2::OctomapMap create_octomap_map(
  const std::string & filename, const std::vector<Eigen::Vector3d> & occupied_points,
  double resolution = 0.5)
{
  emcl2::OctomapMap map;
  const auto path = create_octomap_file(filename, occupied_points, resolution);
  if (!map.loadFromFile(path)) {
    throw std::runtime_error("failed to load test octomap");
  }
  return map;
}

}  // namespace emcl2_test

#endif  // EMCL2_TEST_OCTOMAP_TEST_UTILS_HPP_

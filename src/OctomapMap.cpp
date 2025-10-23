// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "emcl2/OctomapMap.h"

#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>

#include <filesystem>
#include <memory>

namespace emcl2
{

OctomapMap::OctomapMap() = default;

OctomapMap::~OctomapMap() = default;

bool OctomapMap::loadFromFile(const std::string & path)
{
  tree_.reset();

  if (path.empty()) {
    return false;
  }

  std::unique_ptr<octomap::OcTree> occupancy_tree;

  const auto ext = std::filesystem::path(path).extension().string();
  if (ext == ".bt") {
    auto binary_tree = std::make_unique<octomap::OcTree>(0.1);
    if (!binary_tree->readBinary(path)) {
      return false;
    }
    occupancy_tree = std::move(binary_tree);
  } else {
    std::unique_ptr<octomap::OcTree> from_factory(
      dynamic_cast<octomap::OcTree *>(octomap::AbstractOcTree::read(path)));
    if (!from_factory) {
      return false;
    }
    occupancy_tree = std::move(from_factory);
  }

  tree_ = std::move(occupancy_tree);
  return static_cast<bool>(tree_);
}

bool OctomapMap::isOccupied(const Eigen::Vector3d & position) const
{
  if (!tree_) {
    return false;
  }
  const auto * node = tree_->search(position.x(), position.y(), position.z());
  if (!node) {
    return false;
  }
  return tree_->isNodeOccupied(node);
}

bool OctomapMap::isOccupied(double x, double y, double z) const
{
  return isOccupied(Eigen::Vector3d(x, y, z));
}

double OctomapMap::resolution() const
{
  if (!tree_) {
    return 0.0;
  }
  return tree_->getResolution();
}

}  // namespace emcl2

// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#include <gtest/gtest.h>

#include <octomap/OcTree.h>

#include <filesystem>

#include "emcl2/OctomapMap.h"

namespace
{

std::string create_test_octomap()
{
  const auto temp_path = std::filesystem::temp_directory_path() / "emcl2_test_map.bt";
  octomap::OcTree tree(0.5);
  tree.updateNode(octomap::point3d(1.0f, 2.0f, 0.5f), true);
  tree.updateNode(octomap::point3d(-1.0f, -2.0f, 0.5f), true);
  tree.writeBinary(temp_path.string());
  return temp_path.string();
}

}  // namespace

TEST(OctomapMapTest, LoadAndQuery)
{
  const auto path = create_test_octomap();

  emcl2::OctomapMap map;
  ASSERT_TRUE(map.loadFromFile(path));
  EXPECT_NEAR(0.5, map.resolution(), 1e-9);
  EXPECT_TRUE(map.isOccupied(1.0, 2.0, 0.5));
  EXPECT_FALSE(map.isOccupied(1.0, 2.5, 0.5));
}

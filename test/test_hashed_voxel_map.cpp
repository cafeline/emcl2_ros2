#include <gtest/gtest.h>

#include <Eigen/Core>

#include <filesystem>
#include <string>

#include "emcl2/HashedVoxelMap.h"

namespace
{

std::string testDataPath()
{
  const std::filesystem::path current_file(__FILE__);
  return (current_file.parent_path() / "data" / "simple_hash_map.npz").string();
}

}  // namespace

TEST(HashedVoxelMapTest, LoadAndQuery)
{
  emcl2::HashedVoxelMap map;
  ASSERT_TRUE(map.loadFromFile(testDataPath()));

  EXPECT_TRUE(map.isOccupied(0.05, 0.05, 0.05));
  EXPECT_TRUE(map.isOccupied(1.85, 0.05, 0.05));
  EXPECT_TRUE(map.isOccupied(0.05, 1.85, 0.05));
  EXPECT_TRUE(map.isOccupied(-0.05, 0.05, 0.05));
  EXPECT_TRUE(map.isOccupied(0.05, 0.05, 1.65));
  EXPECT_FALSE(map.isOccupied(3.0, 0.0, 0.0));
  EXPECT_NEAR(map.voxelSize(), 0.2, 1e-6);
}

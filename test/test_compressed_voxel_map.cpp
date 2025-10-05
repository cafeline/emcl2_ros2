#include <gtest/gtest.h>

#include <cstdint>

#include "emcl2/CompressedVoxelMap.h"

#include "hdf5_test_utils.hpp"

TEST(CompressedVoxelMapTest, LoadAndQuery)
{
  const auto path = emcl2_test::create_basic_hdf5_map("emcl2_test_map.h5");

  emcl2::CompressedVoxelMap map;
  ASSERT_TRUE(map.loadFromFile(path));

  EXPECT_TRUE(map.isOccupied(0.5, 0.5, 0.5));
  EXPECT_FALSE(map.isOccupied(3.0, 0.5, 0.5));
}

TEST(CompressedVoxelMapTest, LoadWithOffset)
{
  const auto path = emcl2_test::create_offset_hdf5_map("emcl2_test_map_offset.h5");

  emcl2::CompressedVoxelMap map;
  ASSERT_TRUE(map.loadFromFile(path));

  EXPECT_FALSE(map.isOccupied(-1.5, 0.5, 0.5));  // オフセット側ブロックは非占有
  EXPECT_TRUE(map.isOccupied(0.5, 0.5, 0.5));   // 右隣ブロックは占有
  EXPECT_FALSE(map.isOccupied(5.0, 0.5, 0.5));  // Outside block grid
}

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

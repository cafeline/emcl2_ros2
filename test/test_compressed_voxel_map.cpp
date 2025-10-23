// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

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
  EXPECT_FALSE(map.isOccupied(0.5, 1.5, 0.5));
  EXPECT_FALSE(map.isOccupied(1.5, 0.5, 0.5));
  EXPECT_FALSE(map.isOccupied(3.0, 0.5, 0.5));
}

TEST(CompressedVoxelMapTest, LoadWithOffset)
{
  const auto path = emcl2_test::create_offset_hdf5_map("emcl2_test_map_offset.h5");

  emcl2::CompressedVoxelMap map;
  ASSERT_TRUE(map.loadFromFile(path));

  EXPECT_FALSE(map.isOccupied(-1.5, 0.5, 0.5));  // マップ外は非占有
  EXPECT_FALSE(map.isOccupied(2.5, 0.5, 0.5));   // ブロック ID は 1 だがパターン的に空
  EXPECT_TRUE(map.isOccupied(3.5, 1.5, 0.5));    // 同ブロック内の別ボクセルは占有
  EXPECT_FALSE(map.isOccupied(5.0, 0.5, 0.5));  // Outside block grid
}

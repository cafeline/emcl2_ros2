// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#include <gtest/gtest.h>

#include <cstdint>

#include "emcl2/RawVoxelGridMap.h"

#include "raw_hdf5_test_utils.hpp"

TEST(RawVoxelGridMapTest, LoadAndQuery)
{
  const auto path = emcl2_test::create_basic_raw_hdf5_map("emcl2_test_raw_map.h5");

  emcl2::RawVoxelGridMap map;
  ASSERT_TRUE(map.loadFromFile(path));
  EXPECT_TRUE(map.isOccupied(0.1, 0.1, 0.1));
  EXPECT_FALSE(map.isOccupied(1.2, 0.2, 0.1));
  EXPECT_TRUE(map.isOccupied(1.6, 1.6, 0.1));
  EXPECT_FALSE(map.isOccupied(3.0, 0.5, 0.5));
}

TEST(RawVoxelGridMapTest, LoadWithOffset)
{
  const auto path = emcl2_test::create_offset_raw_hdf5_map("emcl2_test_raw_map_offset.h5");

  emcl2::RawVoxelGridMap map;
  ASSERT_TRUE(map.loadFromFile(path));

  EXPECT_FALSE(map.isOccupied(0.0, -0.5, 0.0));   // マップ外は非占有
  EXPECT_TRUE(map.isOccupied(1.75, -0.75, 0.0));  // オフセット適用後の占有点
  EXPECT_FALSE(map.isOccupied(2.5, -0.5, 0.0));   // 同じスライス内の空き領域
  EXPECT_FALSE(map.isOccupied(4.0, -0.5, 0.0));   // Outside block grid
}

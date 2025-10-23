// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <filesystem>

#include "emcl2/HashedVoxelMap.h"
#include "emcl2/PointCloudObservation.h"
#include "emcl2/Particle.h"

namespace
{

std::string testDataPath()
{
  const std::filesystem::path current_file(__FILE__);
  return (current_file.parent_path() / "data" / "simple_hash_map.npz").string();
}

emcl2::HashedVoxelMap createMap()
{
  emcl2::HashedVoxelMap map;
  EXPECT_TRUE(map.loadFromFile(testDataPath()));
  return map;
}

emcl2::PointCloudObservation createObservation(const Eigen::Vector3d & point)
{
  emcl2::PointCloudObservation obs;
  obs.sensor_offset = Eigen::Vector3d::Zero();
  obs.sensor_rotation = Eigen::Quaterniond::Identity();
  obs.points.push_back(point);
  return obs;
}

}  // namespace

TEST(ParticleTest, UpdateWeightWithHit)
{
  auto map = createMap();
  auto obs = createObservation(Eigen::Vector3d(0.05, 0.05, 0.05));

  emcl2::Particle particle(0.0, 0.0, 0.0, 1.0);
  auto new_weight = particle.updateWeight(map, obs);

  EXPECT_DOUBLE_EQ(new_weight, 1.0);
  EXPECT_DOUBLE_EQ(particle.weight(), 1.0);
}

TEST(ParticleTest, UpdateWeightMiss)
{
  auto map = createMap();
  auto obs = createObservation(Eigen::Vector3d(3.0, 3.0, 3.0));

  emcl2::Particle particle(0.0, 0.0, 0.0, 1.0);
  auto new_weight = particle.updateWeight(map, obs);

  EXPECT_DOUBLE_EQ(new_weight, 0.0);
  EXPECT_DOUBLE_EQ(particle.weight(), 0.0);
}

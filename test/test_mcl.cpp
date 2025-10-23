// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <random>

#include "emcl2/RawVoxelGridMap.h"
#include "emcl2/Mcl.h"
#include "emcl2/PointCloudObservation.h"
#include "emcl2/Particle.h"

#include "raw_hdf5_test_utils.hpp"

namespace
{

emcl2::RawVoxelGridMap createMap()
{
  emcl2::RawVoxelGridMap map;
  EXPECT_TRUE(map.loadFromFile(emcl2_test::create_basic_raw_hdf5_map("mcl_map.h5")));
  return map;
}

emcl2::PointCloudObservation observationAt(const Eigen::Vector3d & point)
{
  emcl2::PointCloudObservation obs;
  obs.sensor_offset = Eigen::Vector3d::Zero();
  obs.sensor_rotation = Eigen::Quaterniond::Identity();
  obs.points.push_back(point);
  return obs;
}

}  // namespace

TEST(MclTest, SensorUpdateFavoursConsistentParticles)
{
  auto map = createMap();
  auto obs = observationAt(Eigen::Vector3d(0.1, 0.1, 0.0));

  std::vector<emcl2::Particle> particles;
  particles.emplace_back(0.0, 0.0, 0.0, 1.0);
  particles.emplace_back(5.0, 5.0, 0.0, 1.0);

  emcl2::Mcl filter(std::move(particles));
  filter.sensorUpdate(map, obs);

  const auto & updated = filter.particles();
  ASSERT_EQ(updated.size(), 2u);
  EXPECT_GT(updated[0].weight(), updated[1].weight());
}

TEST(MclTest, ResampleProducesUniformWeights)
{
  std::vector<emcl2::Particle> particles;
  particles.emplace_back(0.0, 0.0, 0.0, 0.7);
  particles.emplace_back(1.0, 0.0, 0.0, 0.2);
  particles.emplace_back(2.0, 0.0, 0.0, 0.1);

  emcl2::Mcl filter(std::move(particles));
  std::mt19937 rng(42);
  filter.resample(rng);

  const double expected = 1.0 / static_cast<double>(filter.particles().size());
  for (const auto & particle : filter.particles()) {
    EXPECT_NEAR(particle.weight(), expected, 1e-12);
  }
}

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <cmath>

#include "emcl2/CompressedVoxelMap.h"
#include "emcl2/PointCloudObservation.h"
#include "emcl2/SensorModel.h"
#include "emcl2/Pose.h"

#include "hdf5_test_utils.hpp"

namespace
{

emcl2::CompressedVoxelMap createMap()
{
  emcl2::CompressedVoxelMap map;
  EXPECT_TRUE(map.loadFromFile(emcl2_test::create_basic_hdf5_map("emcl2_test_map_sensor.h5")));
  return map;
}

}  // namespace

TEST(SensorModelTest, ReturnsHitCount)
{
  auto map = createMap();

  emcl2::PointCloudObservation obs;
  obs.sensor_offset = Eigen::Vector3d::Zero();
  obs.sensor_rotation = Eigen::Quaterniond::Identity();
  obs.points.push_back(Eigen::Vector3d(0.5, 0.5, 0.5));

  emcl2::Pose pose(0.0, 0.0, 0.0);

  const auto score = emcl2::evaluatePointCloudLikelihood(pose, obs, map);
  EXPECT_DOUBLE_EQ(score, 1.0);
}

TEST(SensorModelTest, MissReturnsZero)
{
  auto map = createMap();

  emcl2::PointCloudObservation obs;
  obs.sensor_offset = Eigen::Vector3d::Zero();
  obs.sensor_rotation = Eigen::Quaterniond::Identity();
  obs.points.push_back(Eigen::Vector3d(3.0, 3.0, 3.0));

  emcl2::Pose pose(0.0, 0.0, 0.0);

  const auto score = emcl2::evaluatePointCloudLikelihood(pose, obs, map);
  EXPECT_DOUBLE_EQ(score, 0.0);
}

TEST(SensorModelTest, YawTransformationApplied)
{
  auto map = createMap();

  emcl2::PointCloudObservation obs;
  obs.sensor_offset = Eigen::Vector3d::Zero();
  obs.sensor_rotation = Eigen::Quaterniond::Identity();
  obs.points.push_back(Eigen::Vector3d(0.5, 0.0, 0.5));

  emcl2::Pose pose(0.0, 0.0, M_PI_2);

  const auto score = emcl2::evaluatePointCloudLikelihood(pose, obs, map);
  EXPECT_DOUBLE_EQ(score, 1.0);
}

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>

#include "emcl2/HashedVoxelMap.h"
#include "emcl2/PointCloudObservation.h"
#include "emcl2/SensorModel.h"
#include "emcl2/Pose.h"

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

}  // namespace

TEST(SensorModelTest, ReturnsHitCount)
{
  auto map = createMap();

  emcl2::PointCloudObservation obs;
  obs.sensor_offset = Eigen::Vector3d::Zero();
  obs.sensor_rotation = Eigen::Quaterniond::Identity();
  obs.points.push_back(Eigen::Vector3d(0.05, 0.05, 0.05));

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
  obs.points.push_back(Eigen::Vector3d(0.05, 0.0, 0.05));

  emcl2::Pose pose(0.05, 0.0, M_PI_2);

  const auto score = emcl2::evaluatePointCloudLikelihood(pose, obs, map);
  EXPECT_DOUBLE_EQ(score, 1.0);
}

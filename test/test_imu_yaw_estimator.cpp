// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "emcl2/ImuYawEstimator.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>

using emcl2::ImuYawEstimator;
using emcl2::ImuYawParams;

namespace
{

rclcpp::Time timeFromSec(double sec)
{
  const int64_t nsec = static_cast<int64_t>(sec * 1e9);
  return rclcpp::Time(nsec);
}

}  // namespace

TEST(ImuYawEstimatorTest, BiasRemovalAndYawIntegration)
{
  ImuYawParams params;
  params.bias_sample_count = 2;
  params.clip_enable = false;
  params.lpf_enable = false;
  params.dt_check_enable = false;

  ImuYawEstimator estimator(params);

  ASSERT_TRUE(estimator.process(timeFromSec(0.0), Eigen::Vector3d(0.0, 0.0, 0.1)));
  ASSERT_TRUE(estimator.process(timeFromSec(0.01), Eigen::Vector3d(0.0, 0.0, 0.1)));
  EXPECT_TRUE(estimator.biasReady());
  EXPECT_NEAR(estimator.yaw(), 0.0, 1e-6);

  ASSERT_TRUE(estimator.process(timeFromSec(0.02), Eigen::Vector3d(0.0, 0.0, 1.1)));
  EXPECT_NEAR(estimator.yaw(), 0.01, 1e-4);
}

TEST(ImuYawEstimatorTest, DtCheckSkipsOutliers)
{
  ImuYawParams params;
  params.bias_sample_count = 1;
  params.clip_enable = false;
  params.lpf_enable = false;
  params.dt_check_enable = true;
  params.dt_min = 0.001;
  params.dt_max = 0.1;

  ImuYawEstimator estimator(params);
  ASSERT_TRUE(estimator.process(timeFromSec(0.0), Eigen::Vector3d::Zero()));
  EXPECT_TRUE(estimator.biasReady());

  EXPECT_FALSE(estimator.process(timeFromSec(0.2), Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_NEAR(estimator.yaw(), 0.0, 1e-9);

  EXPECT_TRUE(estimator.process(timeFromSec(0.3), Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_NEAR(estimator.yaw(), 0.1, 1e-6);
}

TEST(ImuYawEstimatorTest, LowPassFilterIsApplied)
{
  ImuYawParams params;
  params.bias_sample_count = 1;
  params.clip_enable = false;
  params.lpf_enable = true;
  params.lpf_alpha = 0.5;
  params.dt_check_enable = false;

  ImuYawEstimator estimator(params);

  ASSERT_TRUE(estimator.process(timeFromSec(0.0), Eigen::Vector3d::Zero()));
  ASSERT_TRUE(estimator.process(timeFromSec(0.01), Eigen::Vector3d(0.0, 0.0, 1.0)));

  // First filtered value: 0.5 * 1.0 + 0.5 * 0.0 = 0.5, dt = 0.01 -> yaw += 0.005
  EXPECT_NEAR(estimator.yaw(), 0.005, 1e-4);
}

TEST(ImuYawEstimatorTest, ResetOrientationAlignsAndPausesDt)
{
  ImuYawParams params;
  params.bias_sample_count = 1;
  params.clip_enable = false;
  params.lpf_enable = false;
  params.dt_check_enable = false;

  ImuYawEstimator estimator(params);
  ASSERT_TRUE(estimator.process(timeFromSec(0.0), Eigen::Vector3d::Zero()));
  ASSERT_TRUE(estimator.process(timeFromSec(0.01), Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_NEAR(estimator.yaw(), 0.01, 1e-6);

  estimator.resetOrientation(M_PI_2);
  EXPECT_NEAR(estimator.yaw(), M_PI_2, 1e-6);
  EXPECT_FALSE(estimator.haveStamp());

  ASSERT_TRUE(estimator.process(timeFromSec(0.02), Eigen::Vector3d::Zero()));
  EXPECT_NEAR(estimator.yaw(), M_PI_2, 1e-6);

  ASSERT_TRUE(estimator.process(timeFromSec(0.03), Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_NEAR(estimator.yaw(), M_PI_2 + 0.01, 1e-4);
}

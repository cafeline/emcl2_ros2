// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "emcl2/OdomYawGate.h"

#include <gtest/gtest.h>

using emcl2::ImuYawGateInput;
using emcl2::selectYaw;

TEST(OdomYawGateTest, UsesImuWhenAvailable)
{
  ImuYawGateInput input;
  input.use_imu_yaw = true;
  input.have_imu_yaw = true;
  input.time_since_last_imu = 0.1;
  input.imu_timeout = 1.0;
  input.imu_yaw = 1.23;
  input.odom_yaw = -0.5;

  double yaw_out = 0.0;
  bool used_imu = false;
  const bool ok = selectYaw(input, yaw_out, used_imu);

  EXPECT_TRUE(ok);
  EXPECT_TRUE(used_imu);
  EXPECT_DOUBLE_EQ(yaw_out, input.imu_yaw);
}

TEST(OdomYawGateTest, RejectsWhenImuMissing)
{
  ImuYawGateInput input;
  input.use_imu_yaw = true;
  input.have_imu_yaw = false;
  input.time_since_last_imu = 0.1;
  input.imu_timeout = 1.0;
  input.imu_yaw = 0.5;
  input.odom_yaw = -0.5;

  double yaw_out = 0.0;
  bool used_imu = false;
  const bool ok = selectYaw(input, yaw_out, used_imu);

  EXPECT_FALSE(ok);
  EXPECT_FALSE(used_imu);
}

TEST(OdomYawGateTest, RejectsWhenImuTimeout)
{
  ImuYawGateInput input;
  input.use_imu_yaw = true;
  input.have_imu_yaw = true;
  input.time_since_last_imu = 2.0;
  input.imu_timeout = 1.0;
  input.imu_yaw = 0.5;
  input.odom_yaw = -0.5;

  double yaw_out = 0.0;
  bool used_imu = false;
  const bool ok = selectYaw(input, yaw_out, used_imu);

  EXPECT_FALSE(ok);
  EXPECT_FALSE(used_imu);
}

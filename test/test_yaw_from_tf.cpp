// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#include <gtest/gtest.h>

#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "emcl2/yaw_from_tf.h"

TEST(YawFromTf, ExtractsYawOnly)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "imu_heading";
  tf2::Quaternion q;
  q.setRPY(0.1, -0.2, 1.0);  // roll/pitch should be ignored
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  const double yaw = emcl2::extractYawFromTf(tf);
  EXPECT_NEAR(yaw, 1.0, 1e-6);
}

TEST(YawFromTf, NormalizesYawToMinusPiPi)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "imu_heading";
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 3.5);  // > pi
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  const double yaw = emcl2::extractYawFromTf(tf);
  EXPECT_GT(yaw, -3.141592653589793);
  EXPECT_LT(yaw, 3.141592653589793);
  EXPECT_NEAR(yaw, 3.5 - 2 * M_PI, 1e-6);
}

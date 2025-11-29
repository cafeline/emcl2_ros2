// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__YAW_FROM_TF_H_
#define EMCL2__YAW_FROM_TF_H_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

namespace emcl2
{

  inline double extractYawFromTf(const geometry_msgs::msg::TransformStamped & tf_msg)
  {
    tf2::Quaternion q(
      tf_msg.transform.rotation.x,
      tf_msg.transform.rotation.y,
      tf_msg.transform.rotation.z,
      tf_msg.transform.rotation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return std::atan2(std::sin(yaw), std::cos(yaw));
  }

}  // namespace emcl2

#endif  // EMCL2__YAW_FROM_TF_H_

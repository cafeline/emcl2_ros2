// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__ODOM_YAW_GATE_H_
#define EMCL2__ODOM_YAW_GATE_H_

namespace emcl2
{

struct ImuYawGateInput
{
  bool use_imu_yaw {true};
  bool have_imu_yaw {false};
  double time_since_last_imu {0.0};
  double imu_timeout {0.0};
  double imu_yaw {0.0};
  double odom_yaw {0.0};
};

inline bool selectYaw(const ImuYawGateInput & input, double & yaw_out, bool & used_imu)
{
  if (!input.use_imu_yaw) {
    yaw_out = input.odom_yaw;
    used_imu = false;
    return true;
  }

  const bool imu_fresh = input.time_since_last_imu <= input.imu_timeout;
  if (!input.have_imu_yaw || !imu_fresh) {
    yaw_out = input.odom_yaw;
    used_imu = false;
    return false;
  }

  yaw_out = input.imu_yaw;
  used_imu = true;
  return true;
}

}  // namespace emcl2

#endif  // EMCL2__ODOM_YAW_GATE_H_

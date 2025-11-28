// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__IMU_YAW_ESTIMATOR_H_
#define EMCL2__IMU_YAW_ESTIMATOR_H_

#include <Eigen/Geometry>
#include <rclcpp/time.hpp>
#include <vector>

namespace emcl2
{

  struct ImuYawParams
  {
    int bias_sample_count {200};

    bool clip_enable {true};
    double clip_max_rad_per_s {5.0};

    bool lpf_enable {true};
    double lpf_alpha {0.2};

    bool dt_check_enable {true};
    double dt_min {0.0005};
    double dt_max {0.05};
  };

  class ImuYawEstimator
  {
public:
    explicit ImuYawEstimator(const ImuYawParams & params = ImuYawParams());

    void setParams(const ImuYawParams & params);
    void resetAll();
    void resetOrientation(double yaw_rad);

    bool process(
      const rclcpp::Time & stamp,
      const Eigen::Vector3d & angular_velocity,
      const Eigen::Vector3d & linear_acceleration = Eigen::Vector3d::Zero());

    double yaw() const;
    const Eigen::Quaterniond & orientation() const {return orientation_;}
    const Eigen::Vector3d & bias() const {return bias_;}
    bool biasReady() const {return bias_ready_;}
    const rclcpp::Time & lastStamp() const {return last_stamp_;}
    bool haveStamp() const {return have_stamp_;}

private:
    ImuYawParams params_;

    bool have_stamp_ {false};
    rclcpp::Time last_stamp_;

    bool bias_ready_ {false};
    Eigen::Vector3d bias_ {Eigen::Vector3d::Zero()};
    std::vector < Eigen::Vector3d > bias_samples_;

    bool lpf_initialized_ {false};
    Eigen::Vector3d lpf_prev_ {Eigen::Vector3d::Zero()};

    Eigen::Quaterniond orientation_ {Eigen::Quaterniond::Identity()};
  };

}  // namespace emcl2

#endif  // EMCL2__IMU_YAW_ESTIMATOR_H_

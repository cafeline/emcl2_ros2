// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "emcl2/ImuYawEstimator.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace emcl2
{

namespace
{

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const auto mid = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), mid, values.end());
  double med = *mid;
  if (values.size() % 2 == 0) {
    const auto mid_prev = values.begin() + (values.size() / 2 - 1);
    std::nth_element(values.begin(), mid_prev, values.end());
    med = 0.5 * (med + *mid_prev);
  }
  return med;
}

Eigen::Vector3d computeMedian(const std::vector<Eigen::Vector3d> & samples)
{
  if (samples.empty()) {
    return Eigen::Vector3d::Zero();
  }
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  xs.reserve(samples.size());
  ys.reserve(samples.size());
  zs.reserve(samples.size());
  for (const auto & s : samples) {
    xs.push_back(s.x());
    ys.push_back(s.y());
    zs.push_back(s.z());
  }
  return Eigen::Vector3d(median(xs), median(ys), median(zs));
}

}  // namespace

ImuYawEstimator::ImuYawEstimator(const ImuYawParams & params)
: params_(params)
{
  resetAll();
}

void ImuYawEstimator::setParams(const ImuYawParams & params)
{
  params_ = params;
}

void ImuYawEstimator::resetAll()
{
  have_stamp_ = false;
  bias_ready_ = false;
  bias_samples_.clear();
  bias_.setZero();
  lpf_initialized_ = false;
  lpf_prev_.setZero();
  orientation_.setIdentity();
}

void ImuYawEstimator::resetOrientation(double yaw_rad)
{
  orientation_ = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ());
  have_stamp_ = false;
  lpf_initialized_ = false;
  lpf_prev_.setZero();
}

bool ImuYawEstimator::process(
  const rclcpp::Time & stamp,
  const Eigen::Vector3d & angular_velocity,
  const Eigen::Vector3d &)
{
  if (!have_stamp_) {
    last_stamp_ = stamp;
    have_stamp_ = true;

    if (!bias_ready_) {
      bias_samples_.push_back(angular_velocity);
      if (bias_samples_.size() >= static_cast<std::size_t>(params_.bias_sample_count)) {
        bias_ = computeMedian(bias_samples_);
        bias_ready_ = true;
      }
    }

    if (params_.lpf_enable) {
      lpf_prev_ = angular_velocity;
      lpf_initialized_ = true;
    }
    return true;
  }

  const double dt = static_cast<double>((stamp - last_stamp_).nanoseconds()) / 1e9;
  if (params_.dt_check_enable) {
    if (dt < params_.dt_min || dt > params_.dt_max) {
      last_stamp_ = stamp;
      return false;
    }
  }

  last_stamp_ = stamp;

  if (!bias_ready_) {
    bias_samples_.push_back(angular_velocity);
    if (bias_samples_.size() >= static_cast<std::size_t>(params_.bias_sample_count)) {
      bias_ = computeMedian(bias_samples_);
      bias_ready_ = true;
    }
  }

  Eigen::Vector3d gyro = angular_velocity;
  if (bias_ready_) {
    gyro -= bias_;
  } else if (!bias_samples_.empty()) {
    const Eigen::Vector3d provisional_bias = computeMedian(bias_samples_);
    gyro -= provisional_bias;
  }

  if (params_.clip_enable) {
    gyro[0] = std::clamp(gyro[0], -params_.clip_max_rad_per_s, params_.clip_max_rad_per_s);
    gyro[1] = std::clamp(gyro[1], -params_.clip_max_rad_per_s, params_.clip_max_rad_per_s);
    gyro[2] = std::clamp(gyro[2], -params_.clip_max_rad_per_s, params_.clip_max_rad_per_s);
  }

  if (params_.lpf_enable) {
    if (!lpf_initialized_) {
      lpf_prev_ = gyro;
      lpf_initialized_ = true;
    } else {
      gyro = params_.lpf_alpha * gyro + (1.0 - params_.lpf_alpha) * lpf_prev_;
      lpf_prev_ = gyro;
    }
  }

  const Eigen::Vector3d angle_vec = gyro * dt;
  const double angle = angle_vec.norm();
  if (angle > 1e-9) {
    const Eigen::AngleAxisd delta(angle, angle_vec.normalized());
    orientation_ = orientation_ * Eigen::Quaterniond(delta);
    orientation_.normalize();
  }
  return true;
}

double ImuYawEstimator::yaw() const
{
  Eigen::Vector3d euler = orientation_.toRotationMatrix().eulerAngles(0, 1, 2);
  return euler.z();
}

}  // namespace emcl2

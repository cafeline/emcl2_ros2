// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__EXTERNAL_YAW_MANAGER_H_
#define EMCL2__EXTERNAL_YAW_MANAGER_H_

#include <cmath>

namespace emcl2
{

  class ExternalYawManager
  {
public:
    explicit ExternalYawManager(double initial_target_yaw = 0.0)
    : target_yaw_(initial_target_yaw)
    {
    }

    void setTargetYaw(double target_yaw)
    {
      target_yaw_ = target_yaw;
      offset_initialized_ = false;
    }

    void updateMeasurement(double yaw_rad)
    {
      last_yaw_measurement_ = yaw_rad;
      have_measurement_ = true;
      if (!offset_initialized_) {
        computeOffset(target_yaw_);
      }
    }

    bool applyInitialPose(double target_yaw)
    {
      target_yaw_ = target_yaw;
      if (!have_measurement_) {
        offset_initialized_ = false;
        return false;
      }
      computeOffset(target_yaw_);
      return true;
    }

    bool ready() const
    {
      return have_measurement_ && offset_initialized_;
    }

    double yaw() const
    {
      return normalize(last_yaw_measurement_ + yaw_offset_);
    }

    bool haveMeasurement() const
    {
      return have_measurement_;
    }

private:
    static double normalize(double yaw)
    {
      while (yaw > M_PI) {
        yaw -= 2.0 * M_PI;
      }
      while (yaw < -M_PI) {
        yaw += 2.0 * M_PI;
      }
      return yaw;
    }

    void computeOffset(double target_yaw)
    {
      yaw_offset_ = normalize(target_yaw - last_yaw_measurement_);
      offset_initialized_ = true;
    }

    double target_yaw_ {0.0};
    double last_yaw_measurement_ {0.0};
    double yaw_offset_ {0.0};
    bool have_measurement_ {false};
    bool offset_initialized_ {false};
  };

}  // namespace emcl2

#endif  // EMCL2__EXTERNAL_YAW_MANAGER_H_

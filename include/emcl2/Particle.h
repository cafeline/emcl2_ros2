// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__PARTICLE_H_
#define EMCL2__PARTICLE_H_

#include "emcl2/OctomapMap.h"
#include "emcl2/PointCloudObservation.h"
#include "emcl2/Pose.h"

namespace emcl2 {

  class Particle
  {
public:
    Particle(double x, double y, double yaw, double weight);

    double updateWeight(const OctomapMap & map, const PointCloudObservation & observation);

    const Pose & pose() const {return p_;}
    Pose & pose() {return p_;}
    double weight() const {return w_;}
    void setWeight(double weight) {w_ = weight;}

private:
    Pose p_;
    double w_;
  };

}  // namespace emcl2

#endif  // EMCL2__PARTICLE_H_

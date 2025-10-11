#ifndef EMCL2__PARTICLE_H_
#define EMCL2__PARTICLE_H_

#include "emcl2/HashedVoxelMap.h"
#include "emcl2/PointCloudObservation.h"
#include "emcl2/Pose.h"

namespace emcl2 {

  class Particle
  {
public:
    Particle(double x, double y, double yaw, double weight);

    double updateWeight(const HashedVoxelMap & map, const PointCloudObservation & observation);

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

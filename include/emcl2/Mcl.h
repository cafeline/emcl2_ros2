#ifndef EMCL2__MCL_H_
#define EMCL2__MCL_H_

#include "emcl2/CompressedVoxelMap.h"
#include "emcl2/Particle.h"
#include "emcl2/PointCloudObservation.h"

#include <random>
#include <vector>

namespace emcl2 {

  class Mcl
  {
public:
    explicit Mcl(std::vector < Particle > particles);

    void sensorUpdate(const CompressedVoxelMap & map, const PointCloudObservation & observation);
    void normalizeWeights();
    void resample(std::mt19937 & rng);
    void initialize(double x, double y, double yaw);

    const std::vector < Particle > & particles() const {return particles_;}
    std::vector < Particle > &particles() {
      return particles_;
    }

private:
    std::vector < Particle > particles_;
  };

}  // namespace emcl2

#endif  // EMCL2__MCL_H_

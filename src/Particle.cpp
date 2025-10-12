#include "emcl2/Particle.h"

#include "emcl2/SensorModel.h"

namespace emcl2
{

Particle::Particle(double x, double y, double yaw, double weight)
: p_(x, y, yaw), w_(weight)
{
}

double Particle::updateWeight(
  const RawVoxelGridMap & map, const PointCloudObservation & observation)
{
  const double score = evaluatePointCloudLikelihood(p_, observation, map);
  w_ = score;
  return w_;
}

}  // namespace emcl2

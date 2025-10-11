#ifndef EMCL2__SENSOR_MODEL_H_
#define EMCL2__SENSOR_MODEL_H_

#include "emcl2/OctomapMap.h"
#include "emcl2/PointCloudObservation.h"
#include "emcl2/Pose.h"

namespace emcl2
{

  double evaluatePointCloudLikelihood(
    const Pose & pose,
    const PointCloudObservation & observation,
    const OctomapMap & map);

}  // namespace emcl2

#endif  // EMCL2__SENSOR_MODEL_H_

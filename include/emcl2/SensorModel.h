// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EMCL2__SENSOR_MODEL_H_
#define EMCL2__SENSOR_MODEL_H_

#include "emcl2/RawVoxelGridMap.h"
#include "emcl2/PointCloudObservation.h"
#include "emcl2/Pose.h"

namespace emcl2
{

  double evaluatePointCloudLikelihood(
    const Pose & pose,
    const PointCloudObservation & observation,
    const RawVoxelGridMap & map);

}  // namespace emcl2

#endif  // EMCL2__SENSOR_MODEL_H_

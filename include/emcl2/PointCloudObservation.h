#ifndef EMCL2__POINT_CLOUD_OBSERVATION_H_
#define EMCL2__POINT_CLOUD_OBSERVATION_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

namespace emcl2 {

  struct PointCloudObservation
  {
    std::vector < Eigen::Vector3d > points;
    Eigen::Vector3d sensor_offset {Eigen::Vector3d::Zero()};
    Eigen::Quaterniond sensor_rotation {Eigen::Quaterniond::Identity()};
  };

}  // namespace emcl2

#endif  // EMCL2__POINT_CLOUD_OBSERVATION_H_

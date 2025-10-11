#include "emcl2/SensorModel.h"

#include <cmath>

namespace emcl2
{

double evaluatePointCloudLikelihood(
  const Pose & pose, const PointCloudObservation & observation,
  const OctomapMap & map)
{
  if (observation.points.empty()) {
    return 0.0;
  }

  const double cos_yaw = std::cos(pose.t_);
  const double sin_yaw = std::sin(pose.t_);

  double hits = 0.0;

  for (const auto & pt_sensor : observation.points) {
    Eigen::Vector3d point_in_base = observation.sensor_rotation * pt_sensor +
      observation.sensor_offset;
    Eigen::Vector3d point_world;
    point_world.x() = pose.x_ + cos_yaw * point_in_base.x() - sin_yaw * point_in_base.y();
    point_world.y() = pose.y_ + sin_yaw * point_in_base.x() + cos_yaw * point_in_base.y();
    point_world.z() = point_in_base.z();

    if (map.isOccupied(point_world)) {
      hits += 1.0;
    }
  }

  return hits;
}

}  // namespace emcl2

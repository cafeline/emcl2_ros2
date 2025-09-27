#include "emcl2/PointCloudHeightFilter.h"

#include <algorithm>

namespace emcl2
{

PointCloudHeightFilter::PointCloudHeightFilter(
  double default_z_min, double default_z_max, std::vector<HeightRegion> regions)
: regions_(std::move(regions))
{
  const auto default_bounds = std::minmax(default_z_min, default_z_max);
  default_z_min_ = default_bounds.first;
  default_z_max_ = default_bounds.second;

  for (auto & region : regions_) {
    if (region.x_min > region.x_max) {
      std::swap(region.x_min, region.x_max);
    }
    if (region.y_min > region.y_max) {
      std::swap(region.y_min, region.y_max);
    }
    const auto bounds = std::minmax(region.z_min, region.z_max);
    region.z_min = bounds.first;
    region.z_max = bounds.second;
  }
}

std::pair<double, double> PointCloudHeightFilter::rangeForPose(double x, double y) const
{
  for (const auto & region : regions_) {
    const bool inside_x = (x >= region.x_min) && (x <= region.x_max);
    const bool inside_y = (y >= region.y_min) && (y <= region.y_max);
    if (inside_x && inside_y) {
      return {region.z_min, region.z_max};
    }
  }
  return {default_z_min_, default_z_max_};
}

std::vector<Eigen::Vector3d> PointCloudHeightFilter::filter(
  const std::vector<Eigen::Vector3d> & points,
  double pose_x, double pose_y) const
{
  const auto range = rangeForPose(pose_x, pose_y);
  std::vector<Eigen::Vector3d> result;
  result.reserve(points.size());
  for (const auto & point : points) {
    const double z = point.z();
    if (z >= range.first && z <= range.second) {
      result.push_back(point);
    }
  }
  return result;
}

}  // namespace emcl2

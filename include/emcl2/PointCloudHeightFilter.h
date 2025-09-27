#ifndef EMCL2__POINT_CLOUD_HEIGHT_FILTER_H_
#define EMCL2__POINT_CLOUD_HEIGHT_FILTER_H_

#include <Eigen/Core>

#include <utility>
#include <vector>

namespace emcl2 {

  struct HeightRegion
  {
    double x_min {0.0};
    double x_max {0.0};
    double y_min {0.0};
    double y_max {0.0};
    double z_min {0.0};
    double z_max {0.0};
  };

  class PointCloudHeightFilter
  {
public:
    PointCloudHeightFilter(
      double default_z_min, double default_z_max,
      std::vector < HeightRegion > regions);

    std::pair < double, double > rangeForPose(double x, double y) const;

    std::vector < Eigen::Vector3d > filter(
      const std::vector < Eigen::Vector3d > &points,
      double pose_x, double pose_y) const;

private:
    double default_z_min_;
    double default_z_max_;
    std::vector < HeightRegion > regions_;
  };

}  // namespace emcl2

#endif  // EMCL2__POINT_CLOUD_HEIGHT_FILTER_H_

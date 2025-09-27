#include <gtest/gtest.h>

#include "emcl2/PointCloudHeightFilter.h"

namespace
{

std::vector<Eigen::Vector3d> makePoints(const std::vector<double> & zs)
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(zs.size());
  for (double z : zs) {
    points.emplace_back(0.0, 0.0, z);
  }
  return points;
}

}  // namespace

TEST(PointCloudHeightFilterTest, DefaultRangeFiltersPoints)
{
  emcl2::PointCloudHeightFilter filter(-1.0, 1.0, {});

  auto points = makePoints({-2.0, -0.5, 0.5, 2.0});
  auto filtered = filter.filter(points, 0.0, 0.0);

  ASSERT_EQ(2u, filtered.size());
  EXPECT_DOUBLE_EQ(-0.5, filtered[0].z());
  EXPECT_DOUBLE_EQ(0.5, filtered[1].z());
}

TEST(PointCloudHeightFilterTest, RegionOverridesDefaultRange)
{
  std::vector<emcl2::HeightRegion> regions;
  regions.push_back({1.0, 3.0, -1.0, 1.0, 2.0, 3.0});
  emcl2::PointCloudHeightFilter filter(-1.0, 1.0, regions);

  auto points = makePoints({0.5, 2.5});

  auto filtered_default = filter.filter(points, 0.0, 0.0);
  ASSERT_EQ(1u, filtered_default.size());
  EXPECT_DOUBLE_EQ(0.5, filtered_default[0].z());

  auto filtered_region = filter.filter(points, 2.0, 0.0);
  ASSERT_EQ(1u, filtered_region.size());
  EXPECT_DOUBLE_EQ(2.5, filtered_region[0].z());
}

TEST(PointCloudHeightFilterTest, BoundaryIsInclusive)
{
  std::vector<emcl2::HeightRegion> regions;
  regions.push_back({-1.0, 1.0, -1.0, 1.0, -0.2, 0.2});
  emcl2::PointCloudHeightFilter filter(-1.0, 1.0, regions);

  auto points = makePoints({-0.2, 0.0, 0.2, 0.3});

  auto filtered_center = filter.filter(points, 1.0, -1.0);
  ASSERT_EQ(3u, filtered_center.size());
  EXPECT_DOUBLE_EQ(-0.2, filtered_center[0].z());
  EXPECT_DOUBLE_EQ(0.0, filtered_center[1].z());
  EXPECT_DOUBLE_EQ(0.2, filtered_center[2].z());
}

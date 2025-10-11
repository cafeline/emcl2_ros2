#ifndef EMCL2__OCTOMAP_MAP_H_
#define EMCL2__OCTOMAP_MAP_H_

#include <Eigen/Core>

#include <string>
#include <memory>

namespace octomap
{
  class AbstractOcTree;
  class OcTree;
}  // namespace octomap

namespace emcl2
{
  class OctomapMap
  {
public:
    OctomapMap();
    ~OctomapMap();

    OctomapMap(const OctomapMap &) = delete;
    OctomapMap & operator = (const OctomapMap &) = delete;
    OctomapMap(OctomapMap &&) noexcept = default;
    OctomapMap & operator = (OctomapMap &&)noexcept = default;

    bool loadFromFile(const std::string & path);

    bool isOccupied(const Eigen::Vector3d & position) const;
    bool isOccupied(double x, double y, double z) const;

    double resolution() const;

private:
    std::unique_ptr < octomap::OcTree > tree_;
  };
}  // namespace emcl2

#endif  // EMCL2__OCTOMAP_MAP_H_

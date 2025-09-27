#ifndef EMCL2__EMCL2_NODE_H_
#define EMCL2__EMCL2_NODE_H_

#include "emcl2/CompressedVoxelMap.h"
#include "emcl2/Mcl.h"
#include "emcl2/OdomModel.h"
#include "emcl2/PointCloudHeightFilter.h"
#include "emcl2/PointCloudObservation.h"

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <memory>
#include <random>
#include <string>

namespace emcl2
{

  class EMcl2Node: public rclcpp::Node
  {
public:
    EMcl2Node();
    ~EMcl2Node() override;

    void loop();
    int getOdomFreq() const;

private:
    void declareParameter();
    void initCommunication();
    void initTF();
    void loadMap();
    void initializeParticles();

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void timerCallback();

    bool updateWithOdometry();
    void publishOutputs(const rclcpp::Time & stamp);
    geometry_msgs::msg::PoseWithCovarianceStamped buildPoseMessage(
      const Pose & mean_pose, double var_x, double var_y, double var_yaw,
      const rclcpp::Time & stamp) const;
    geometry_msgs::msg::PoseArray buildParticleArray(const rclcpp::Time & stamp) const;
    Pose computeWeightedMean(double & var_x, double & var_y, double & var_yaw) const;

    CompressedVoxelMap map_;
    std::unique_ptr < Mcl > filter_;
    std::unique_ptr < OdomModel > odom_model_;
    PointCloudObservation observation_template_;
    std::unique_ptr < PointCloudHeightFilter > height_filter_;

    std::string map_frame_id_ {"map"};
    std::string odom_frame_id_ {"odom"};
    std::string base_frame_id_ {"base_link"};
    std::string pointcloud_topic_ {"/pointcloud"};
    std::string filtered_pointcloud_topic_ {"filtered_pointcloud"};
    double transform_tolerance_ {0.2};
    int odom_freq_ {20};
    bool map_loaded_ {false};
    bool publish_filtered_pointcloud_ {false};

    std::mt19937 rng_;

    rclcpp::Subscription < sensor_msgs::msg::PointCloud2 > ::SharedPtr pointcloud_sub_;
    rclcpp::Publisher < geometry_msgs::msg::PoseArray > ::SharedPtr particle_pub_;
    rclcpp::Publisher < geometry_msgs::msg::PoseWithCovarianceStamped > ::SharedPtr pose_pub_;
    rclcpp::Publisher < sensor_msgs::msg::PointCloud2 > ::SharedPtr filtered_cloud_pub_;

    std::shared_ptr < tf2_ros::Buffer > tf_buffer_;
    std::shared_ptr < tf2_ros::TransformListener > tf_listener_;
    std::shared_ptr < tf2_ros::TransformBroadcaster > tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr loop_timer_;

    Pose last_odom_pose_;
    bool have_last_odom_ {false};

    double initial_pose_x_ {0.0};
    double initial_pose_y_ {0.0};
    double initial_pose_yaw_ {0.0};
    double initial_std_xy_ {0.5};
    double initial_std_yaw_ {0.5};

    int num_particles_ {200};

    double odom_noise_ff_ {0.05};
    double odom_noise_fr_ {0.05};
    double odom_noise_rf_ {0.05};
    double odom_noise_rr_ {0.05};
  };

}  // namespace emcl2

#endif  // EMCL2__EMCL2_NODE_H_

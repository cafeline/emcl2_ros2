#include "emcl2/emcl2_node.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>

namespace emcl2
{

EMcl2Node::EMcl2Node()
: Node("emcl2_node"), rng_(std::random_device{}())
{
  declareParameter();
  loadMap();
  initializeParticles();
  initTF();
  initCommunication();

  if (odom_freq_ > 0) {
    const auto period = std::chrono::duration<double>(1.0 / static_cast<double>(odom_freq_));
    loop_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&EMcl2Node::timerCallback, this));
  }
}

EMcl2Node::~EMcl2Node() = default;

void EMcl2Node::declareParameter()
{
  this->declare_parameter("map_frame_id", map_frame_id_);
  this->declare_parameter("odom_frame_id", odom_frame_id_);
  this->declare_parameter("base_frame_id", base_frame_id_);
  this->declare_parameter("pointcloud_topic", pointcloud_topic_);
  this->declare_parameter("map_hdf5_path", std::string(""));
  this->declare_parameter("odom_freq", odom_freq_);
  this->declare_parameter("transform_tolerance", transform_tolerance_);

  this->declare_parameter("num_particles", num_particles_);
  this->declare_parameter("initial_pose_x", initial_pose_x_);
  this->declare_parameter("initial_pose_y", initial_pose_y_);
  this->declare_parameter("initial_pose_yaw", initial_pose_yaw_);
  this->declare_parameter("initial_std_xy", initial_std_xy_);
  this->declare_parameter("initial_std_yaw", initial_std_yaw_);

  this->declare_parameter("sensor_offset_x", 0.0);
  this->declare_parameter("sensor_offset_y", 0.0);
  this->declare_parameter("sensor_offset_z", 0.0);
  this->declare_parameter("sensor_roll", 0.0);
  this->declare_parameter("sensor_pitch", 0.0);
  this->declare_parameter("sensor_yaw", 0.0);

  this->declare_parameter("odom_fw_dev_per_fw", odom_noise_ff_);
  this->declare_parameter("odom_fw_dev_per_rot", odom_noise_fr_);
  this->declare_parameter("odom_rot_dev_per_fw", odom_noise_rf_);
  this->declare_parameter("odom_rot_dev_per_rot", odom_noise_rr_);

  map_frame_id_ = this->get_parameter("map_frame_id").as_string();
  odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();
  pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
  odom_freq_ = this->get_parameter("odom_freq").as_int();
  transform_tolerance_ = this->get_parameter("transform_tolerance").as_double();

  num_particles_ = this->get_parameter("num_particles").as_int();
  initial_pose_x_ = this->get_parameter("initial_pose_x").as_double();
  initial_pose_y_ = this->get_parameter("initial_pose_y").as_double();
  initial_pose_yaw_ = this->get_parameter("initial_pose_yaw").as_double();
  initial_std_xy_ = this->get_parameter("initial_std_xy").as_double();
  initial_std_yaw_ = this->get_parameter("initial_std_yaw").as_double();

  odom_noise_ff_ = this->get_parameter("odom_fw_dev_per_fw").as_double();
  odom_noise_fr_ = this->get_parameter("odom_fw_dev_per_rot").as_double();
  odom_noise_rf_ = this->get_parameter("odom_rot_dev_per_fw").as_double();
  odom_noise_rr_ = this->get_parameter("odom_rot_dev_per_rot").as_double();

  observation_template_.sensor_offset.x() = this->get_parameter("sensor_offset_x").as_double();
  observation_template_.sensor_offset.y() = this->get_parameter("sensor_offset_y").as_double();
  observation_template_.sensor_offset.z() = this->get_parameter("sensor_offset_z").as_double();

  const double roll = this->get_parameter("sensor_roll").as_double();
  const double pitch = this->get_parameter("sensor_pitch").as_double();
  const double yaw = this->get_parameter("sensor_yaw").as_double();
  Eigen::AngleAxisd r_roll(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd r_pitch(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd r_yaw(yaw, Eigen::Vector3d::UnitZ());
  observation_template_.sensor_rotation = r_yaw * r_pitch * r_roll;
}

void EMcl2Node::initCommunication()
{
  particle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", 1);
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("mcl_pose", 1);

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&EMcl2Node::pointCloudCallback, this, std::placeholders::_1));
}

void EMcl2Node::initTF()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void EMcl2Node::loadMap()
{
  const auto path = this->get_parameter("map_hdf5_path").as_string();
  if (path.empty()) {
    throw rclcpp::exceptions::InvalidParametersException("map_hdf5_path parameter is empty");
  }
  if (!map_.loadFromFile(path)) {
    throw rclcpp::exceptions::InvalidParametersException("failed to load HDF5 map: " + path);
  }
  map_loaded_ = true;
  RCLCPP_INFO(get_logger(), "Loaded compressed voxel map: %s", path.c_str());
}

void EMcl2Node::initializeParticles()
{
  if (!map_loaded_) {
    return;
  }
  if (num_particles_ <= 0) {
    num_particles_ = 1;
  }

  std::normal_distribution<double> dist_xy(0.0, initial_std_xy_);
  std::normal_distribution<double> dist_yaw(0.0, initial_std_yaw_);

  std::vector<Particle> particles;
  particles.reserve(static_cast<std::size_t>(num_particles_));
  for (int i = 0; i < num_particles_; ++i) {
    double x = initial_pose_x_ + dist_xy(rng_);
    double y = initial_pose_y_ + dist_xy(rng_);
    double yaw = initial_pose_yaw_ + dist_yaw(rng_);
    particles.emplace_back(x, y, yaw, 1.0);  // weight normalized later
  }

  filter_ = std::make_unique<Mcl>(std::move(particles));
  filter_->normalizeWeights();
  odom_model_ = std::make_unique<OdomModel>(
    odom_noise_ff_, odom_noise_fr_, odom_noise_rf_,
    odom_noise_rr_);
  have_last_odom_ = false;
}

int EMcl2Node::getOdomFreq() const
{
  return odom_freq_;
}

void EMcl2Node::loop()
{
  const auto now = this->now();
  updateWithOdometry();
  publishOutputs(now);
}

void EMcl2Node::timerCallback()
{
  loop();
}

void EMcl2Node::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!filter_ || !map_loaded_) {
    return;
  }

  updateWithOdometry();

  PointCloudObservation observation;
  observation.sensor_offset = observation_template_.sensor_offset;
  observation.sensor_rotation = observation_template_.sensor_rotation;

  observation.points.reserve(static_cast<std::size_t>(msg->width) * msg->height);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const float x = *iter_x;
    const float y = *iter_y;
    const float z = *iter_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    observation.points.emplace_back(
      static_cast<double>(x), static_cast<double>(y),
      static_cast<double>(z));
  }

  if (observation.points.empty()) {
    return;
  }

  filter_->sensorUpdate(map_, observation);
  filter_->normalizeWeights();
  filter_->resample(rng_);

  publishOutputs(msg->header.stamp);
}

bool EMcl2Node::updateWithOdometry()
{
  if (!filter_ || !odom_model_) {
    return false;
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(odom_frame_id_, base_frame_id_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
    return false;
  }

  const double yaw = tf2::getYaw(tf.transform.rotation);
  Pose current(tf.transform.translation.x, tf.transform.translation.y, yaw);

  if (!have_last_odom_) {
    last_odom_pose_ = current;
    have_last_odom_ = true;
    return false;
  }

  Pose delta = current - last_odom_pose_;
  if (delta.nearlyZero()) {
    return false;
  }

  const double length = std::sqrt(delta.x_ * delta.x_ + delta.y_ * delta.y_);
  const double direction = std::atan2(delta.y_, delta.x_) - last_odom_pose_.t_;
  odom_model_->setDev(length, delta.t_);

  for (auto & particle : filter_->particles()) {
    particle.pose().move(
      length, direction, delta.t_, odom_model_->drawFwNoise(), odom_model_->drawRotNoise());
  }

  last_odom_pose_ = current;
  return true;
}

Pose EMcl2Node::computeWeightedMean(double & var_x, double & var_y, double & var_yaw) const
{
  Pose mean(initial_pose_x_, initial_pose_y_, initial_pose_yaw_);
  if (!filter_) {
    var_x = var_y = var_yaw = 0.0;
    return mean;
  }

  double sum_w = 0.0;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_sin = 0.0;
  double sum_cos = 0.0;

  for (const auto & particle : filter_->particles()) {
    const double w = particle.weight();
    sum_w += w;
    sum_x += particle.pose().x_ * w;
    sum_y += particle.pose().y_ * w;
    sum_sin += std::sin(particle.pose().t_) * w;
    sum_cos += std::cos(particle.pose().t_) * w;
  }

  if (sum_w <= 0.0) {
    var_x = var_y = var_yaw = 0.0;
    return mean;
  }

  mean.x_ = sum_x / sum_w;
  mean.y_ = sum_y / sum_w;
  mean.t_ = std::atan2(sum_sin, sum_cos);

  double accum_x = 0.0;
  double accum_y = 0.0;
  double accum_yaw = 0.0;
  for (const auto & particle : filter_->particles()) {
    const double w = particle.weight();
    accum_x += w * std::pow(particle.pose().x_ - mean.x_, 2.0);
    accum_y += w * std::pow(particle.pose().y_ - mean.y_, 2.0);
    double yaw_error = particle.pose().t_ - mean.t_;
    while (yaw_error > M_PI) {
      yaw_error -= 2 * M_PI;
    }
    while (yaw_error < -M_PI) {
      yaw_error += 2 * M_PI;
    }
    accum_yaw += w * yaw_error * yaw_error;
  }

  var_x = accum_x / sum_w;
  var_y = accum_y / sum_w;
  var_yaw = accum_yaw / sum_w;

  return mean;
}

geometry_msgs::msg::PoseWithCovarianceStamped EMcl2Node::buildPoseMessage(
  const Pose & mean_pose, double var_x, double var_y, double var_yaw,
  const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = map_frame_id_;

  msg.pose.pose.position.x = mean_pose.x_;
  msg.pose.pose.position.y = mean_pose.y_;
  msg.pose.pose.position.z = observation_template_.sensor_offset.z();

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, mean_pose.t_);
  msg.pose.pose.orientation = tf2::toMsg(q);

  msg.pose.covariance.fill(0.0);
  msg.pose.covariance[0] = var_x;
  msg.pose.covariance[7] = var_y;
  msg.pose.covariance[35] = var_yaw;

  return msg;
}

geometry_msgs::msg::PoseArray EMcl2Node::buildParticleArray(const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = map_frame_id_;
  msg.poses.reserve(filter_ ? filter_->particles().size() : 0);

  if (filter_) {
    for (const auto & particle : filter_->particles()) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = particle.pose().x_;
      pose.position.y = particle.pose().y_;
      pose.position.z = observation_template_.sensor_offset.z();
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, particle.pose().t_);
      pose.orientation = tf2::toMsg(q);
      msg.poses.push_back(pose);
    }
  }

  return msg;
}

void EMcl2Node::publishOutputs(const rclcpp::Time & stamp)
{
  if (!filter_) {
    return;
  }

  double var_x = 0.0;
  double var_y = 0.0;
  double var_yaw = 0.0;
  Pose mean_pose = computeWeightedMean(var_x, var_y, var_yaw);

  auto pose_msg = buildPoseMessage(mean_pose, var_x, var_y, var_yaw, stamp);
  pose_pub_->publish(pose_msg);

  auto cloud_msg = buildParticleArray(stamp);
  particle_pub_->publish(cloud_msg);

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = map_frame_id_;
  tf_msg.child_frame_id = odom_frame_id_;

  geometry_msgs::msg::TransformStamped odom_to_base;
  try {
    odom_to_base = tf_buffer_->lookupTransform(odom_frame_id_, base_frame_id_, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    odom_to_base = geometry_msgs::msg::TransformStamped();
    odom_to_base.transform.rotation.w = 1.0;
  }

  tf2::Transform t_map_base;
  t_map_base.setOrigin(
    tf2::Vector3(
      mean_pose.x_, mean_pose.y_,
      observation_template_.sensor_offset.z()));
  tf2::Quaternion q_map_base;
  q_map_base.setRPY(0.0, 0.0, mean_pose.t_);
  t_map_base.setRotation(q_map_base);

  tf2::Transform t_odom_base;
  tf2::fromMsg(odom_to_base.transform, t_odom_base);
  tf2::Transform t_map_odom = t_map_base * t_odom_base.inverse();

  tf_msg.transform = tf2::toMsg(t_map_odom);
  tf_broadcaster_->sendTransform(tf_msg);
}

}  // namespace emcl2

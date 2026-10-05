#include "ros_interface.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>

#include "config_loader.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

// Constants
const int kDofFloatingBase = 6;        // Number of DoF for floating base
const int kNumFloatingBaseJoints = 7;  // Number of joints for floating base (quaternion + xyz)
const int kDimQuaternion = 4;          // Dimension of a quaternion
constexpr double kPi = 3.14159265358979323846;
constexpr double kGoldenAngle = 2.39996322972865332;

namespace {

builtin_interfaces::msg::Time MakeSimTime(double simulation_time) {
  const auto nanoseconds = static_cast<std::int64_t>(std::llround(std::max(0.0, simulation_time) * 1.0e9));
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

}  // namespace

namespace mujoco {

RosInterface::RosInterface(const rclcpp::Node::SharedPtr& node, std::shared_ptr<ConfigLoader> config_loader)
    : node_(node), config_loader_(config_loader), model_(nullptr), data_(nullptr), is_floating_base_(false) {}

RosInterface::~RosInterface() {}

bool RosInterface::Initialize() {
  // Create publishers
  joint_state_pub_ =
      node_->create_publisher<interface_protocol::msg::JointState>(config_loader_->GetJointStateTopic(), 10);

  imu_pub_ = node_->create_publisher<interface_protocol::msg::ImuInfo>(config_loader_->GetImuTopic(), 10);
  lidar_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      config_loader_->GetLidarTopic(), rclcpp::SensorDataQoS());
  clock_pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  
  // Create publisher for motion state
  motion_state_pub_ = node_->create_publisher<interface_protocol::msg::MotionState>("/motion/motion_state", 10);

  // Create subscriber with more compatible QoS settings
  using std::placeholders::_1;

  // 创建更兼容的QoS设置
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

  joint_cmd_sub_ = node_->create_subscription<interface_protocol::msg::JointCommand>(
      config_loader_->GetJointCommandTopic(), qos, std::bind(&RosInterface::JointCommandCallback, this, _1));

  // Get number of joints from config loader
  num_total_joints_ = config_loader_->GetNumTotalJoints();

  // Initialize commanded values with zeros
  joint_command_.position.resize(num_total_joints_, 0.0);
  joint_command_.velocity.resize(num_total_joints_, 0.0);
  joint_command_.torque.resize(num_total_joints_, 0.0);
  joint_command_.feed_forward_torque.resize(num_total_joints_, 0.0);
  joint_command_.stiffness.resize(num_total_joints_, 0.0);
  joint_command_.damping.resize(num_total_joints_, 0.0);

  // Create timer for publishing motion state every 1 second
  motion_state_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RosInterface::MotionStateTimerCallback, this));

  RCLCPP_INFO(node_->get_logger(), "MuJoCo ROS interface initialized successfully");
  return true;
}

interface_protocol::msg::JointCommand RosInterface::GetCommandedSafe() {
  std::lock_guard<std::mutex> lock(mtx_);
  return joint_command_;
}

void RosInterface::JointCommandCallback(const interface_protocol::msg::JointCommand::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_);

  // Update commanded values
  joint_command_ = *msg;

  // Ensure all vectors are properly sized
  if (joint_command_.position.size() > num_total_joints_) {
    joint_command_.position.resize(num_total_joints_);
  }
  if (joint_command_.velocity.size() > num_total_joints_) {
    joint_command_.velocity.resize(num_total_joints_);
  }
  if (joint_command_.torque.size() > num_total_joints_) {
    joint_command_.torque.resize(num_total_joints_);
  }
  if (joint_command_.feed_forward_torque.size() > num_total_joints_) {
    joint_command_.feed_forward_torque.resize(num_total_joints_);
  }
  if (joint_command_.stiffness.size() > num_total_joints_) {
    joint_command_.stiffness.resize(num_total_joints_);
  }
  if (joint_command_.damping.size() > num_total_joints_) {
    joint_command_.damping.resize(num_total_joints_);
  }
}

void RosInterface::UpdateSimState(const mjModel* m, mjData* d) {
  is_floating_base_ = (m->nv != m->nu);
  const builtin_interfaces::msg::Time stamp = MakeSimTime(d->time);

  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = stamp;
  clock_pub_->publish(clock_msg);

  // Create messages
  auto joint_state_msg = std::make_unique<interface_protocol::msg::JointState>();
  auto imu_msg = std::make_unique<interface_protocol::msg::ImuInfo>();

  // Set timestamp
  joint_state_msg->header.stamp = stamp;
  imu_msg->header.stamp = stamp;

  // Set joint states
  joint_state_msg->position.resize(num_total_joints_);
  joint_state_msg->velocity.resize(num_total_joints_);
  joint_state_msg->torque.resize(num_total_joints_);

  if (is_floating_base_) {
    // Skip the floating base joints
    for (int i = 0; i < num_total_joints_; ++i) {
      joint_state_msg->position[i] = d->qpos[i + kNumFloatingBaseJoints];
      joint_state_msg->velocity[i] = d->qvel[i + kDofFloatingBase];
      joint_state_msg->torque[i] = d->actuator_force[i];
    }
  } else {
    for (int i = 0; i < num_total_joints_; ++i) {
      joint_state_msg->position[i] = d->qpos[i];
      joint_state_msg->velocity[i] = d->qvel[i];
      joint_state_msg->torque[i] = d->actuator_force[i];
    }
  }

  // IMU data typically comes from sensors in MuJoCo
  int index = 0;

  // Set IMU quaternion
  imu_msg->quaternion.w = d->sensordata[index + 0];
  imu_msg->quaternion.x = d->sensordata[index + 1];
  imu_msg->quaternion.y = d->sensordata[index + 2];
  imu_msg->quaternion.z = d->sensordata[index + 3];
  index += kDimQuaternion;

  // Set RPY values from the sensor data
  // Assuming the RPY values are the next three values after the quaternion
  imu_msg->rpy.x = d->sensordata[index + 0];  // Roll
  imu_msg->rpy.y = d->sensordata[index + 1];  // Pitch
  imu_msg->rpy.z = d->sensordata[index + 2];  // Yaw
  index += 3;

  // Linear acceleration
  imu_msg->linear_acceleration.x = d->sensordata[index + 0];
  imu_msg->linear_acceleration.y = d->sensordata[index + 1];
  imu_msg->linear_acceleration.z = d->sensordata[index + 2];
  index += 3;

  // Angular velocity
  imu_msg->angular_velocity.x = d->sensordata[index + 0];
  imu_msg->angular_velocity.y = d->sensordata[index + 1];
  imu_msg->angular_velocity.z = d->sensordata[index + 2];

  // Publish messages
  joint_state_pub_->publish(std::move(joint_state_msg));
  imu_pub_->publish(std::move(imu_msg));

  const double lidar_period = 1.0 / config_loader_->GetLidarRateHz();
  if (last_lidar_publish_time_ < 0.0 || d->time < last_lidar_publish_time_ ||
      d->time - last_lidar_publish_time_ + 1.0e-12 >= lidar_period) {
    PublishLidar(m, d, stamp);
    last_lidar_publish_time_ = d->time;
  }
}

void RosInterface::SetModelAndData(mjModel* model, mjData* data) {
  model_ = model;
  data_ = data;
  livox_site_id_ = model_ ? mj_name2id(model_, mjOBJ_SITE, "livox_lidar_site") : -1;
  last_lidar_publish_time_ = -1.0;
  lidar_scan_sequence_ = 0;

  if (livox_site_id_ < 0) {
    RCLCPP_ERROR(node_->get_logger(), "MuJoCo site 'livox_lidar_site' was not found; LiDAR is disabled");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Livox LiDAR publishing %d points at %.1f Hz on %s",
                config_loader_->GetLidarPointsPerScan(), config_loader_->GetLidarRateHz(),
                config_loader_->GetLidarTopic().c_str());
  }
}

void RosInterface::PublishLidar(const mjModel* model, mjData* data,
                                const builtin_interfaces::msg::Time& stamp) {
  if (livox_site_id_ < 0 || !lidar_pub_) {
    return;
  }

  const int ray_count = config_loader_->GetLidarPointsPerScan();
  const double horizontal_fov =
      std::min(2.0 * kPi, config_loader_->GetLidarHorizontalFovDeg() * kPi / 180.0);
  const double vertical_min = config_loader_->GetLidarVerticalFovMinDeg() * kPi / 180.0;
  const double vertical_max = config_loader_->GetLidarVerticalFovMaxDeg() * kPi / 180.0;
  const double scan_phase = std::fmod(static_cast<double>(lidar_scan_sequence_) * 0.61803398875,
                                      1.0) * horizontal_fov;

  std::vector<mjtNum> local_directions(static_cast<std::size_t>(ray_count) * 3);
  std::vector<mjtNum> world_directions(static_cast<std::size_t>(ray_count) * 3);
  const mjtNum* site_rotation = data->site_xmat + 9 * livox_site_id_;

  for (int i = 0; i < ray_count; ++i) {
    const double azimuth = -0.5 * horizontal_fov +
        std::fmod(static_cast<double>(i) * kGoldenAngle + scan_phase, horizontal_fov);
    const int shuffled_index = static_cast<int>(
        (static_cast<std::uint64_t>(i) * 7919ULL + lidar_scan_sequence_ * 104729ULL) %
        static_cast<std::uint64_t>(ray_count));
    const double vertical_fraction =
        (static_cast<double>(shuffled_index) + 0.5) / static_cast<double>(ray_count);
    const double elevation = vertical_min + (vertical_max - vertical_min) * vertical_fraction;

    mjtNum* local = local_directions.data() + 3 * i;
    local[0] = std::cos(elevation) * std::cos(azimuth);
    local[1] = std::cos(elevation) * std::sin(azimuth);
    local[2] = std::sin(elevation);
    mju_mulMatVec(world_directions.data() + 3 * i, site_rotation, local, 3, 3);
  }

  std::vector<int> geom_ids(ray_count, -1);
  std::vector<mjtNum> distances(ray_count, -1.0);
  const mjtNum* origin = data->site_xpos + 3 * livox_site_id_;
  mj_multiRay(model, data, origin, world_directions.data(), nullptr, 1,
              model->site_bodyid[livox_site_id_], geom_ids.data(), distances.data(), ray_count,
              config_loader_->GetLidarMaxRange());

  std::vector<std::array<float, 4>> points;
  points.reserve(ray_count);
  for (int i = 0; i < ray_count; ++i) {
    const double distance = distances[i];
    if (!std::isfinite(distance) || distance < config_loader_->GetLidarMinRange() ||
        distance > config_loader_->GetLidarMaxRange()) {
      continue;
    }
    const mjtNum* direction = local_directions.data() + 3 * i;
    points.push_back({static_cast<float>(direction[0] * distance),
                      static_cast<float>(direction[1] * distance),
                      static_cast<float>(direction[2] * distance), 1.0F});
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = config_loader_->GetLidarFrameId();
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> x_iter(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y_iter(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z_iter(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity_iter(cloud, "intensity");
  for (const auto& point : points) {
    *x_iter = point[0];
    *y_iter = point[1];
    *z_iter = point[2];
    *intensity_iter = point[3];
    ++x_iter;
    ++y_iter;
    ++z_iter;
    ++intensity_iter;
  }

  lidar_pub_->publish(cloud);
  PublishLidarTransform(data, stamp);
  ++lidar_scan_sequence_;
}

void RosInterface::PublishLidarTransform(const mjData* data,
                                         const builtin_interfaces::msg::Time& stamp) {
  if (livox_site_id_ < 0 || !tf_broadcaster_) {
    return;
  }

  const mjtNum* position = data->site_xpos + 3 * livox_site_id_;
  const mjtNum* rotation = data->site_xmat + 9 * livox_site_id_;
  mjtNum quaternion[4];
  mju_mat2Quat(quaternion, rotation);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = config_loader_->GetLidarParentFrameId();
  transform.child_frame_id = config_loader_->GetLidarFrameId();
  transform.transform.translation.x = position[0];
  transform.transform.translation.y = position[1];
  transform.transform.translation.z = position[2];
  transform.transform.rotation.w = quaternion[0];
  transform.transform.rotation.x = quaternion[1];
  transform.transform.rotation.y = quaternion[2];
  transform.transform.rotation.z = quaternion[3];
  tf_broadcaster_->sendTransform(transform);
}

void RosInterface::MotionStateTimerCallback() {
  // Create a motion state message
  auto motion_state_msg = std::make_unique<interface_protocol::msg::MotionState>();
  
  // Set the current_motion_task field to "joint_bridge"
  motion_state_msg->current_motion_task = "joint_bridge";
  
  // Publish the message
  motion_state_pub_->publish(std::move(motion_state_msg));
}

}  // namespace mujoco

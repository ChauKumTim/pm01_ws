#ifndef MUJOCO_ROS_INTERFACE_H_
#define MUJOCO_ROS_INTERFACE_H_

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "interface_protocol/msg/imu_info.hpp"
#include "interface_protocol/msg/joint_command.hpp"
#include "interface_protocol/msg/joint_state.hpp"
#include "interface_protocol/msg/motion_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

// MuJoCo includes
#include <mujoco/mujoco.h>

// Forward declarations
namespace mujoco {
class Simulate;
}
class ConfigLoader;

namespace mujoco {

class RosInterface {
 public:
  RosInterface(const rclcpp::Node::SharedPtr& node, std::shared_ptr<ConfigLoader> config_loader);
  ~RosInterface();

  // Initialize the MuJoCo interface
  bool Initialize();

  // Callback for joint command messages
  void JointCommandCallback(const interface_protocol::msg::JointCommand::SharedPtr msg);

  // Update the simulation state to publish to ROS
  void UpdateSimState(const mjModel* m, mjData* d);

  // Get joint command values (thread-safe)
  interface_protocol::msg::JointCommand GetCommandedSafe();

  // Set the current mjModel and mjData
  void SetModelAndData(mjModel* model, mjData* data);

  // Get the ROS node
  rclcpp::Node::SharedPtr GetNode() const { return node_; }

 private:
  // ROS2 node
  rclcpp::Node::SharedPtr node_;

  // Publishers
  rclcpp::Publisher<interface_protocol::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<interface_protocol::msg::ImuInfo>::SharedPtr imu_pub_;
  rclcpp::Publisher<interface_protocol::msg::MotionState>::SharedPtr motion_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Subscribers
  rclcpp::Subscription<interface_protocol::msg::JointCommand>::SharedPtr joint_cmd_sub_;

  // Config loader
  std::shared_ptr<ConfigLoader> config_loader_;

  // Number of joints
  int num_total_joints_ = 0;

  // Current joint command
  interface_protocol::msg::JointCommand joint_command_;

  // MuJoCo model and data
  mjModel* model_;
  mjData* data_;
  int livox_site_id_ = -1;
  double last_lidar_publish_time_ = -1.0;
  std::uint64_t lidar_scan_sequence_ = 0;

  // Timer for publishing motion state
  rclcpp::TimerBase::SharedPtr motion_state_timer_;
  
  // Motion state timer callback
  void MotionStateTimerCallback();
  void PublishLidar(const mjModel* model, mjData* data,
                    const builtin_interfaces::msg::Time& stamp);
  void PublishLidarTransform(const mjData* data,
                             const builtin_interfaces::msg::Time& stamp);

  // Mutex for thread safety
  std::mutex mtx_;

  // Flag indicating if we have a floating base robot
  bool is_floating_base_;
};

}  // namespace mujoco

#endif  // MUJOCO_ROS_INTERFACE_H_

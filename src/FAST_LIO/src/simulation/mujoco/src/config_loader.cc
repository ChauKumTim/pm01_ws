#include "config_loader.h"
#include <fstream>
#include <iostream>
#include <stdexcept>

ConfigLoader::ConfigLoader(const std::string& config_file) : config_file_(config_file) {}

bool ConfigLoader::LoadConfig() {
  try {
    // Load YAML file
    YAML::Node config = YAML::LoadFile(config_file_);

    // Load model parameters
    urdf_filename_ = config["urdf"].as<std::string>();
    xml_filename_ = config["xml"].as<std::string>();

    if (config["model_param"]) {
      num_total_joints_ = config["model_param"]["num_total_joints"].as<int>();
      num_contacts_ = config["model_param"]["num_contacts"].as<int>();
      num_single_contact_dimensions_ = config["model_param"]["num_single_contact_dimensions"].as<int>();
    }

    // Load topic names
    if (config["sensor"]) {
      const YAML::Node sensor = config["sensor"];
      if (sensor["imu_topic"]) imu_topic_ = sensor["imu_topic"].as<std::string>();
      if (sensor["lidar_topic"]) lidar_topic_ = sensor["lidar_topic"].as<std::string>();
      if (sensor["lidar_frame_id"]) lidar_frame_id_ = sensor["lidar_frame_id"].as<std::string>();
      if (sensor["lidar_parent_frame_id"]) {
        lidar_parent_frame_id_ = sensor["lidar_parent_frame_id"].as<std::string>();
      }
      if (sensor["lidar_rate_hz"]) lidar_rate_hz_ = sensor["lidar_rate_hz"].as<double>();
      if (sensor["lidar_min_range"]) lidar_min_range_ = sensor["lidar_min_range"].as<double>();
      if (sensor["lidar_max_range"]) lidar_max_range_ = sensor["lidar_max_range"].as<double>();
      if (sensor["lidar_points_per_scan"]) {
        lidar_points_per_scan_ = sensor["lidar_points_per_scan"].as<int>();
      }
      if (sensor["lidar_horizontal_fov_deg"]) {
        lidar_horizontal_fov_deg_ = sensor["lidar_horizontal_fov_deg"].as<double>();
      }
      if (sensor["lidar_vertical_fov_min_deg"]) {
        lidar_vertical_fov_min_deg_ = sensor["lidar_vertical_fov_min_deg"].as<double>();
      }
      if (sensor["lidar_vertical_fov_max_deg"]) {
        lidar_vertical_fov_max_deg_ = sensor["lidar_vertical_fov_max_deg"].as<double>();
      }
    }

    if (lidar_rate_hz_ <= 0.0 || lidar_min_range_ < 0.0 || lidar_max_range_ <= lidar_min_range_ ||
        lidar_points_per_scan_ <= 0 || lidar_horizontal_fov_deg_ <= 0.0 ||
        lidar_vertical_fov_max_deg_ < lidar_vertical_fov_min_deg_) {
      throw std::runtime_error("Invalid LiDAR configuration");
    }

    if (config["actuator"]) {
      if (config["actuator"]["joint_state_topic"]) {
        joint_state_topic_ = config["actuator"]["joint_state_topic"].as<std::string>();
      }

      if (config["actuator"]["joint_command_topic"]) {
        joint_command_topic_ = config["actuator"]["joint_command_topic"].as<std::string>();
      }
    }

    return true;
  } catch (const std::exception& e) {
    std::cerr << "Error loading config file: " << e.what() << std::endl;
    return false;
  }
}

std::string ConfigLoader::GetModelFilePath() const { return assets_path_ + "/resource/" + xml_filename_; }

std::string ConfigLoader::GetResourceDir() const { return assets_path_ + "/resource"; }

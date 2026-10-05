#ifndef CONFIG_LOADER_H_
#define CONFIG_LOADER_H_

#include <yaml-cpp/yaml.h>
#include <map>
#include <string>
#include <vector>

class ConfigLoader {
 public:
  ConfigLoader(const std::string& config_file);
  ~ConfigLoader() = default;

  bool LoadConfig();

  // Getters for various configuration parameters
  std::string GetUrdfFilename() const { return urdf_filename_; }
  std::string GetXmlFilename() const { return xml_filename_; }
  int GetNumTotalJoints() const { return num_total_joints_; }
  int GetNumContacts() const { return num_contacts_; }
  int GetNumSingleContactDimensions() const { return num_single_contact_dimensions_; }

  // Topic names
  std::string GetImuTopic() const { return imu_topic_; }
  std::string GetLidarTopic() const { return lidar_topic_; }
  std::string GetLidarFrameId() const { return lidar_frame_id_; }
  std::string GetLidarParentFrameId() const { return lidar_parent_frame_id_; }
  double GetLidarRateHz() const { return lidar_rate_hz_; }
  double GetLidarMinRange() const { return lidar_min_range_; }
  double GetLidarMaxRange() const { return lidar_max_range_; }
  int GetLidarPointsPerScan() const { return lidar_points_per_scan_; }
  double GetLidarHorizontalFovDeg() const { return lidar_horizontal_fov_deg_; }
  double GetLidarVerticalFovMinDeg() const { return lidar_vertical_fov_min_deg_; }
  double GetLidarVerticalFovMaxDeg() const { return lidar_vertical_fov_max_deg_; }
  std::string GetJointStateTopic() const { return joint_state_topic_; }
  std::string GetJointCommandTopic() const { return joint_command_topic_; }

  // Asset path related methods
  std::string GetModelFilePath() const;
  std::string GetResourceDir() const;
  void SetAssetsPath(const std::string& assets_path) { assets_path_ = assets_path; }

 private:
  std::string config_file_;
  std::string assets_path_;  // Path to assets directory

  // Model parameters
  std::string urdf_filename_;
  std::string xml_filename_;
  int num_total_joints_ = 0;
  int num_contacts_ = 0;
  int num_single_contact_dimensions_ = 0;

  // Topic names
  std::string imu_topic_;
  std::string lidar_topic_ = "/livox/lidar";
  std::string lidar_frame_id_ = "livox_frame";
  std::string lidar_parent_frame_id_ = "world";
  double lidar_rate_hz_ = 10.0;
  double lidar_min_range_ = 0.1;
  double lidar_max_range_ = 70.0;
  int lidar_points_per_scan_ = 2000;
  double lidar_horizontal_fov_deg_ = 360.0;
  double lidar_vertical_fov_min_deg_ = -7.0;
  double lidar_vertical_fov_max_deg_ = 52.0;
  std::string joint_state_topic_;
  std::string joint_command_topic_;
};

#endif  // CONFIG_LOADER_H_

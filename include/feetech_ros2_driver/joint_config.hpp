#pragma once

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <unordered_map>

namespace feetech_ros2_driver {

/// Joint parameters loaded from YAML config file
using JointParams = std::unordered_map<std::string, std::string>;
/// Map of joint name to its parameters
using JointConfigMap = std::unordered_map<std::string, JointParams>;

/// Load joint configuration from a YAML file
/// @param file_path Path to the YAML configuration file
/// @return JointConfigMap if successful, std::nullopt on error
///
/// Expected YAML format:
/// ```yaml
/// joints:
///   joint1:
///     id: 1
///     homing_offset: 582
///     range_min: 700
///     range_max: 3600
///     p_coefficient: 16
///     i_coefficient: 0
///     d_coefficient: 32
///   joint2:
///     id: 2
///     max_torque_limit: 500
///     protection_current: 250
///     overload_torque: 25
/// ```
inline std::optional<JointConfigMap> load_joint_config(const std::string& file_path) {
  // Check if file exists before attempting to parse
  if (!std::filesystem::exists(file_path)) {
    spdlog::error("joint_config_file '{}' does not exist", file_path);
    return std::nullopt;
  }

  std::ifstream file(file_path);
  if (!file.is_open()) {
    spdlog::error("Failed to open joint_config_file '{}'", file_path);
    return std::nullopt;
  }

  try {
    YAML::Node root = YAML::Load(file);
    auto joints = root["joints"];

    if (!joints || !joints.IsMap()) {
      spdlog::error("joint_config_file '{}' has no top-level 'joints:' map", file_path);
      return std::nullopt;
    }

    JointConfigMap config;
    config.reserve(joints.size());

    for (auto it = joints.begin(); it != joints.end(); ++it) {
      const std::string joint_name = it->first.as<std::string>();
      const YAML::Node joint_node = it->second;

      if (!joint_node.IsMap()) {
        spdlog::warn("Joint '{}' entry is not a map; ignoring", joint_name);
        continue;
      }

      JointParams params;
      params.reserve(joint_node.size());

      for (auto p = joint_node.begin(); p != joint_node.end(); ++p) {
        const std::string key = p->first.as<std::string>();
        const YAML::Node val = p->second;

        if (val.IsScalar()) {
          params[key] = val.Scalar();
        } else {
          spdlog::warn("Ignoring non-scalar param '{}' for joint '{}'", key, joint_name);
        }
      }

      config.emplace(joint_name, std::move(params));
    }

    spdlog::info("Loaded joint configuration for {} joints from '{}'", config.size(), file_path);
    return config;

  } catch (const YAML::Exception& e) {
    spdlog::error("YAML parsing error in '{}': {}", file_path, e.what());
    return std::nullopt;
  } catch (const std::exception& e) {
    spdlog::error("Failed to load joint_config_file '{}': {}", file_path, e.what());
    return std::nullopt;
  }
}

/// Merge YAML config parameters with URDF parameters (YAML takes precedence)
/// @param yaml_params Parameters from YAML config file
/// @param urdf_params Parameters from URDF joint definition
/// @return Merged parameters with YAML values overriding URDF values
inline JointParams merge_joint_params(const JointParams& yaml_params, const JointParams& urdf_params) {
  JointParams merged = urdf_params;
  for (const auto& [key, value] : yaml_params) {
    merged[key] = value;
  }
  return merged;
}

}  // namespace feetech_ros2_driver


#pragma once

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <unordered_map>

namespace feetech_ros2_driver {

using JointParams = std::unordered_map<std::string, std::string>;
using JointConfigMap = std::unordered_map<std::string, JointParams>;
using JointIdConfigMap = std::unordered_map<int, JointParams>;

/// Load joint configuration from a YAML file (expects top-level "joints:" map).
inline std::optional<JointConfigMap> load_joint_config(const std::string& file_path) {
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

/// Merge YAML over URDF parameters (YAML takes precedence).
inline JointParams merge_joint_params(const JointParams& yaml_params, const JointParams& urdf_params) {
  JointParams merged = urdf_params;
  for (const auto& [key, value] : yaml_params) {
    merged[key] = value;
  }
  return merged;
}

}  // namespace feetech_ros2_driver

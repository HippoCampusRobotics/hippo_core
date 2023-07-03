#pragma once

#include <yaml-cpp/yaml.h>

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

namespace hippo_common {
namespace yaml {
struct TagPose {
  std::string frame_id;
  int id;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d position;
  double size;
};

typedef std::vector<TagPose> TagPoses;
typedef Eigen::Vector3d Waypoint;
typedef std::vector<Waypoint> Waypoints;

}  // namespace yaml
}  // namespace hippo_common

namespace YAML {
template <>
struct convert<hippo_common::yaml::TagPose> {
  static Node encode(const hippo_common::yaml::TagPose &rhs) {
    Node node;
    node["frame_id"] = rhs.frame_id;
    node["id"] = rhs.id;
    node["qw"] = rhs.orientation.w();
    node["qx"] = rhs.orientation.y();
    node["qy"] = rhs.orientation.y();
    node["qz"] = rhs.orientation.z();
    node["x"] = rhs.position.x();
    node["y"] = rhs.position.y();
    node["z"] = rhs.position.z();
    node["size"] = rhs.size;
    return node;
  }
  static bool decode(const Node &node, hippo_common::yaml::TagPose &rhs) {
    if (!node.IsMap()) {
      return false;
    }
    rhs.frame_id = node["frame_id"].as<std::string>();
    rhs.id = node["id"].as<int>();
    rhs.orientation.w() = node["qw"].as<double>();
    rhs.orientation.x() = node["qx"].as<double>();
    rhs.orientation.y() = node["qy"].as<double>();
    rhs.orientation.z() = node["qz"].as<double>();
    rhs.position.x() = node["x"].as<double>();
    rhs.position.y() = node["y"].as<double>();
    rhs.position.z() = node["z"].as<double>();
    rhs.size = node["size"].as<double>();
    return true;
  }
};

template <>
struct convert<hippo_common::yaml::TagPoses> {
  static Node encode(const hippo_common::yaml::TagPoses &rhs) {
    Node node;
    for (const auto &pose : rhs) {
      node.push_back(pose);
    }
    return node;
  }

  static bool decode(const Node &node, hippo_common::yaml::TagPoses &rhs) {
    // if (node.IsMap()) {
    //   for (const auto &key_value : node) {
    //     rhs.push_back(key_value.second.as<hippo_common::yaml::TagPose>());
    //   }

    //   return true;
    // }
    for (const auto &value : node) {
      rhs.push_back(value.as<hippo_common::yaml::TagPose>());
    }
    return true;
  }
};

template <>
struct convert<hippo_common::yaml::Waypoint> {
  static Node encode(const hippo_common::yaml::Waypoint &rhs) {
    Node node;
    node["x"] = rhs.x();
    node["y"] = rhs.y();
    node["z"] = rhs.z();
    return node;
  }

  static bool decode(const Node &node, hippo_common::yaml::Waypoint &rhs) {
    rhs.x() = node["x"].as<double>();
    rhs.y() = node["y"].as<double>();
    rhs.z() = node["z"].as<double>();
    return true;
  }
};

template <>
struct convert<hippo_common::yaml::Waypoints> {
  static Node encode(const hippo_common::yaml::Waypoints &rhs) {
    Node node;
    for (const auto &waypoint : rhs) {
      node.push_back(waypoint);
    }
    return node;
  }
  static bool decode(const Node &node, hippo_common::yaml::Waypoints &rhs) {
    for (const auto &value : node) {
      rhs.push_back(value.as<hippo_common::yaml::Waypoint>());
    }
    return true;
  }
};

}  // namespace YAML

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>
namespace hippo_common {
namespace tf2_utils {
namespace frame_id {
static constexpr char kBarometerName[] = "barometer";
static constexpr char kBaseLinkName[] = "base_link";
static constexpr char kInertialName[] = "map";

inline std::string Prefix(rclcpp::Node *_node) {
  std::string name = _node->get_namespace();
  // remove leading slash as tf2 does not like leading slashes.
  name.erase(0, name.find_first_not_of('/'));
  return name;
}
inline std::string Barometer(rclcpp::Node *_node) {
  return Prefix(_node) + "/" + kBarometerName;
}
inline std::string BaseLink(rclcpp::Node *_node) {
  return Prefix(_node) + "/" + kBaseLinkName;
}

inline std::string InertialFrame() {
  return kInertialName;
}

}  // namespace frame_id
}  // namespace tf2_utils
}  // namespace hippo_common

#pragma once
#include <rclcpp/rclcpp.hpp>

namespace hippo_control {
namespace rate_control {
class RateController : public rclcpp::Node {
 public:
  explicit RateController(rclcpp::NodeOptions const &_options);
};
}  // namespace rate_control
}  // namespace hippo_control

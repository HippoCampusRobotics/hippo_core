#include <hippo_common/param_utils.hpp>

#include "path_follower_node.hpp"

namespace path_planning {
void PathFollowerNode::DeclareParams() {
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(look_ahead_distance);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(depth_gain);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(ignore_z_distance);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(path_file);

  params_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        return OnParameters(params);
      });
}

rcl_interfaces::msg::SetParametersResult PathFollowerNode::OnParameters(
    const std::vector<rclcpp::Parameter> &_parameters) {
  std::string text;
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "unhandled";
  result.successful = true;
  bool updated{false};

  for (const auto &parameter : _parameters) {
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(look_ahead_distance, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(depth_gain, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(ignore_z_distance, updated, text);
  }
  if (updated) {
    path_->SetLookAhead(params_.look_ahead_distance);
    path_->ignore_z() = params_.ignore_z_distance;
  }
  return result;
}
}  // namespace path_planning

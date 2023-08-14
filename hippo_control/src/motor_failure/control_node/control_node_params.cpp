#include <hippo_common/param_utils.hpp>

#include "control_node.hpp"

namespace hippo_control {
namespace motor_failure {

void ControlNode::DeclareParams() {
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(model.damping.linear.surge);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(model.damping.linear.pitch);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(model.damping.linear.yaw);

  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(model.inertia.surge);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(model.inertia.pitch);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(model.inertia.yaw);

  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.p.surge);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.p.pitch);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.p.yaw);

  SetControllerGains();
  SetControllerModel();

  params_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        return OnParameters(params);
      });
}

rcl_interfaces::msg::SetParametersResult ControlNode::OnParameters(
    const std::vector<rclcpp::Parameter> &_parameters) {
  std::string text;
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "unhandled";
  result.successful = true;
  bool updated{false};
  for (const auto &parameter : _parameters) {
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(model.damping.linear.surge, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(model.damping.linear.pitch, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(model.damping.linear.yaw, updated, text);

    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(model.inertia.surge, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(model.inertia.pitch, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(model.inertia.yaw, updated, text);

    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.p.surge, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.p.pitch, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.p.yaw, updated, text);
  }
  if (updated) {
    SetControllerGains();
    SetControllerModel();
  }
  return result;
}
}  // namespace motor_failure
}  // namespace hippo_control

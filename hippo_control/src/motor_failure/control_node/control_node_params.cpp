#include <hippo_common/param_utils.hpp>

#include "control_node.hpp"

namespace hippo_control {
namespace motor_failure {

void ControlNode::DeclareParams() {
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(model.damping.linear.surge);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(model.damping.linear.roll);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(model.damping.linear.pitch);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(model.damping.linear.yaw);

  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(model.inertia.surge);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(model.inertia.roll);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(model.inertia.pitch);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(model.inertia.yaw);

  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.p.surge);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.p.roll);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.p.pitch);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.p.yaw);

  SetControllerGains();
  SetControllerModel();

  HIPPO_COMMON_DECLARE_PARAM_READONLY(phase_duration_ms);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(phase_order);
  if (params_.phase_duration_ms.size() != params_.phase_order.size()) {
    throw std::runtime_error(
        "phase order and duration do not match. Order=" +
        std::to_string(params_.phase_order.size()) +
        " duration=" + std::to_string(params_.phase_duration_ms.size()));
  }

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
  bool controller_updated{false};
  for (const auto &parameter : _parameters) {
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(model.damping.linear.surge,
                                   controller_updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(model.damping.linear.roll,
                                   controller_updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(model.damping.linear.pitch,
                                   controller_updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(model.damping.linear.yaw, controller_updated,
                                   text);

    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(model.inertia.surge, controller_updated,
                                   text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(model.inertia.roll, controller_updated,
                                   text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(model.inertia.pitch, controller_updated,
                                   text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(model.inertia.yaw, controller_updated, text);

    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.p.surge, controller_updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.p.roll, controller_updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.p.pitch, controller_updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.p.yaw, controller_updated, text);
  }
  if (controller_updated) {
    SetControllerGains();
    SetControllerModel();
  }
  return result;
}
}  // namespace motor_failure
}  // namespace hippo_control

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "hippo_control/trajectory_tracking/tracking_controller.hpp"

namespace hippo_control {
namespace trajectory_tracking {

void TrackingControllerNode::DeclareParams() {
  std::string name;
  std::string description;
  rcl_interfaces::msg::ParameterDescriptor descriptor;

  name = "position_gain";
  description = "Gain of the position error.";
  descriptor = hippo_common::param_utils::Description(description);
  {
    auto &param = params_.position_gain;
    param = declare_parameter(name, param, descriptor);
  }

  name = "velocity_gain";
  description = "Gain of the velocity error.";
  descriptor = hippo_common::param_utils::Description(description);
  {
    auto &param = params_.velocity_gain;
    param = declare_parameter(name, param, descriptor);
  }

  params_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &_params) {
        return OnParams(_params);
      });
}

rcl_interfaces::msg::SetParametersResult TrackingControllerNode::OnParams(
    const std::vector<rclcpp::Parameter> &_parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "Unhandled";
  result.successful = true;
  std::string name;
  std::string result_text;
  for (const rclcpp::Parameter &parameter : _parameters) {
    name = "position_gain";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.position_gain, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }
    name = "velocity_gain";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.velocity_gain, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }
  }
  if (result.reason != "Unhandled") {
    params_.updated = true;
  }
  return result;
}

}  // namespace trajectory_tracking
}  // namespace hippo_control

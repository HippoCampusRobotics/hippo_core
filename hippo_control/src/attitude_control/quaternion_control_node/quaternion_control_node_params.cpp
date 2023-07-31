#include <hippo_common/param_utils.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "quaternion_control_node.hpp"

namespace hippo_control {
namespace attitude_control {

void QuaternionControlNode::DeclareParams() {
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gain);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(roll_weight);

  params_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        return OnParameters(params);
      });
}

rcl_interfaces::msg::SetParametersResult QuaternionControlNode::OnParameters(
    const std::vector<rclcpp::Parameter> &_parameters) {
  std::string text;
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "unhandled";
  result.successful = true;
  bool updated{false};
  for (const auto &parameter : _parameters) {
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gain, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(roll_weight, updated, text);
  }
  if (updated) {
    // put everything here that is required to handle updated parameters.
    SetControllerGains();
  }
  return result;
}

}  // namespace attitude_control
}  // namespace hippo_control

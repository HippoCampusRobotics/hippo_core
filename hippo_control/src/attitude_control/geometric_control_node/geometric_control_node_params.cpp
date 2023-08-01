#include <hippo_common/param_utils.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "geometric_control_node.hpp"

namespace hippo_control {
namespace attitude_control {

void GeometricControlNode::DeclareParams() {
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gain.roll.p);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gain.roll.d);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gain.pitch.p);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gain.pitch.d);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gain.yaw.p);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gain.yaw.d);
  SetControllerGains();

  gain_params_cb_handle_ = add_on_set_parameters_callback(std::bind(
      &GeometricControlNode::OnGainParameters, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult GeometricControlNode::OnGainParameters(
    const std::vector<rclcpp::Parameter> &_parameters) {
  std::string text;
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "unhandled";
  result.successful = true;
  bool updated{false};
  for (const auto &parameter : _parameters) {
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gain.roll.p, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gain.roll.d, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gain.pitch.p, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gain.pitch.d, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gain.yaw.p, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gain.yaw.d, updated, text);
  }
  if (updated) {
    SetControllerGains();
  }
  return result;
}

}  // namespace attitude_control
}  // namespace hippo_control

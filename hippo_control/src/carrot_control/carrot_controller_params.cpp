#include <hippo_common/param_utils.hpp>

#include "carrot_controller.hpp"

// assumes parameters are stored in this.params_ and names are identical.
#define DECLARE_PARAM_DESCR(x)                        \
  do {                                                \
    auto &param = params_.x;                          \
    param = declare_parameter(#x, param, descriptor); \
  } while (false)
#define DECLARE_PARAM(x)                  \
  do {                                    \
    auto &param = params_.x;              \
    param = declare_parameter(#x, param); \
  } while (false)

#define ASSIGN_SIMPLE_LOG(x, y)                                                \
  if (hippo_common::param_utils::AssignIfMatch(parameter, #x, params_.x, y)) { \
    RCLCPP_INFO_STREAM(get_logger(), y);                                       \
    result.reason = text;                                                      \
    params_.updated = true;                                                    \
    continue;                                                                  \
  }

namespace hippo_control {
namespace carrot_control {
void CarrotController::DeclareParams() {
  std::string name;
  std::string description;
  rcl_interfaces::msg::ParameterDescriptor descriptor;

  DECLARE_PARAM(look_ahead_distance);
  DECLARE_PARAM(path_file);
  DECLARE_PARAM(depth_gain);

  params_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&CarrotController::OnParams, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult CarrotController::OnParams(
    const std::vector<rclcpp::Parameter> &_parameters) {
  std::string text;
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "unhandled";
  result.successful = true;
  for (const auto &parameter : _parameters) {
    ASSIGN_SIMPLE_LOG(look_ahead_distance, text);
    ASSIGN_SIMPLE_LOG(depth_gain, text);
  }
  return result;
}
}  // namespace carrot_control
}  // namespace hippo_control

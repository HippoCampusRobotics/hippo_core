#include <hippo_control/rate_control/rate_controller.hpp>

namespace hippo_control {
namespace rate_control {
RateController::RateController(rclcpp::NodeOptions const &_options)
    : Node("rate_controller", _options) {
  // do stuff
}
}  // namespace rate_control
}  // namespace hippo_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hippo_control::rate_control::RateController)

#include "hippo_control/trajectory_tracking/tracking_controller.hpp"
namespace hippo_control {
namespace trajectory_tracking {

TrackingControllerNode::TrackingControllerNode(
    rclcpp::NodeOptions const &_options)
    : Node("tracking_controller", _options) {}
}  // namespace trajectory_tracking
}  // namespace hippo_control

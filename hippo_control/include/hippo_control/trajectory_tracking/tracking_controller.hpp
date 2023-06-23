#include <rclcpp/rclcpp.hpp>
namespace hippo_control {
namespace trajectory_tracking {
class TrackingControllerNode : public rclcpp::Node {
 public:
  explicit TrackingControllerNode(rclcpp::NodeOptions const &_options);
};
}  // namespace trajectory_tracking
}  // namespace hippo_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hippo_control::trajectory_tracking::TrackingControllerNode)

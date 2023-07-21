#pragma once
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "path_planning/path.hpp"

namespace path_planning {
class RvizHelper {
 public:
  RvizHelper(rclcpp::Node::SharedPtr node);
  void PublishPath(const std::shared_ptr<Path> _path);

 private:
  void InitPathMarker();
  void InitTargetMarker();
  rclcpp::Node::SharedPtr node_{nullptr};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_pub_{
      nullptr};

  visualization_msgs::msg::Marker path_marker_;
  visualization_msgs::msg::Marker target_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
};
}  // namespace path_planning

#pragma once

#include <hippo_gz_msgs/msg/range_measurement_array.pb.h>

#include <hippo_msgs/msg/range_measurement_array.hpp>
#include <ignition/transport/Node.hh>
#include <rclcpp/node_interfaces/node_topics.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hippo_gz_plugins {
namespace range_sensor_bridge {
class RangeSensor : public rclcpp::Node {
 public:
  explicit RangeSensor(rclcpp::NodeOptions const &_options);

 private:
  void DeclareParams();
  void OnRanges(const hippo_gz_msgs::msg::RangeMeasurementArray &_msg);
  rclcpp::Publisher<hippo_msgs::msg::RangeMeasurementArray>::SharedPtr
      ranges_pub_;
  rclcpp::node_interfaces::NodeTopics *node_topics;
  std::shared_ptr<ignition::transport::Node> gz_node_ =
      std::make_shared<ignition::transport::Node>();
};
}  // namespace range_sensor_bridge
}  // namespace hippo_gz_plugins

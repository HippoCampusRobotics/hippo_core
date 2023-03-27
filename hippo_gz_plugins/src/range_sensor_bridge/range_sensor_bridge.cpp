#include "range_sensor_bridge.hpp"

namespace hippo_gz_plugins {
namespace range_sensor_bridge {
RangeSensor::RangeSensor(rclcpp::NodeOptions const &_options)
    : Node("range_sensor", _options) {
  //      DeclareParams();
  ranges_pub_ = create_publisher<hippo_msgs::msg::RangeMeasurementArray>(
      "ranges", rclcpp::SensorDataQoS());
  node_topics =
      (rclcpp::node_interfaces::NodeTopics *)get_node_topics_interface().get();

  std::string topic_name;
  topic_name = node_topics->resolve_topic_name("ranges");
  gz_node_->Subscribe(topic_name, &RangeSensor::OnRanges, this);
}

void RangeSensor::OnRanges(
    const hippo_gz_msgs::msg::RangeMeasurementArray &_msg) {
  rclcpp::Time t_now = now();
  hippo_msgs::msg::RangeMeasurementArray ros_msg;
  ros_msg.header.stamp = t_now;
  for (int i = 0; i < _msg.measurements_size(); ++i) {
    hippo_msgs::msg::RangeMeasurement ros_meas;
    auto gz_meas = _msg.measurements(i);
    ros_meas.id = gz_meas.id();
    ros_meas.range = gz_meas.range();
    ros_meas.header.stamp = t_now;
    ros_msg.measurements.push_back(ros_meas);
  }
  ranges_pub_->publish(ros_msg);
}
}  // namespace range_sensor_bridge
}  // namespace hippo_gz_plugins

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hippo_gz_plugins::range_sensor_bridge::RangeSensor)

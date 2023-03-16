#pragma once

#include <hippo_gz_msgs/msg/range_measurement_array.pb.h>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Element.hh>

namespace range_sensor {
class PluginPrivate {
 public:
  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf);
  bool InitModel(ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::Entity _entity);
  void PublishRanges();
  void AdvertiseRanges() {
    ranges_publisher_ =
        node_.Advertise<hippo_gz_msgs::msg::RangeMeasurementArray>(
            RangesTopicName());
  }

 private:
  struct SdfParams {
    std::string link{"base_link"};
    std::string ranges_base_topic{"ranges"};
  } sdf_params_;

  void InitComponents(ignition::gazebo::EntityComponentManager &_ecm);

  std::string TopicPrefix() { return "/" + model_name_; }

  std::string RangesTopicName() {
    return TopicPrefix() + "/" + sdf_params_.ranges_base_topic;
  }

  ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Link link_{ignition::gazebo::kNullEntity};
  std::string model_name_;

  ignition::transport::Node node_;
  ignition::transport::Node::Publisher ranges_publisher_;
};
}  // namespace range_sensor

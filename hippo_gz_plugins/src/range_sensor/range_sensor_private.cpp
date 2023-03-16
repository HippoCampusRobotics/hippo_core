#include "range_sensor_private.hpp"
#include <ignition/gazebo/components/Pose.hh>

namespace range_sensor {
void PluginPrivate::ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf) {
  // TODO
}

bool PluginPrivate::InitModel(ignition::gazebo::EntityComponentManager &_ecm,
                              ignition::gazebo::Entity _entity) {
  model_ = ignition::gazebo::Model(_entity);
  if (!model_.Valid(_ecm)) {
    return false;
  }
  model_name_ = model_.Name(_ecm);
  InitComponents(_ecm);
  return true;
}

void PluginPrivate::InitComponents(
    ignition::gazebo::EntityComponentManager &_ecm) {
  link_ = ignition::gazebo::Link(model_.LinkByName(_ecm, sdf_params_.link));
  if (!link_.Valid(_ecm)) {
    ignerr << "Link not available: " << sdf_params_.link << std::endl;
    return;
  }
  if (!_ecm.Component<ignition::gazebo::components::WorldPose>(
          link_.Entity())) {
    _ecm.CreateComponent(link_.Entity(),
                         ignition::gazebo::components::WorldPose());
  }
}

void PluginPrivate::PublishRanges() {
  hippo_gz_msgs::msg::RangeMeasurementArray range_array;
  // TODO(lennartalff): actually get the range information
  ranges_publisher_.Publish(range_array);
}
}  // namespace range_sensor

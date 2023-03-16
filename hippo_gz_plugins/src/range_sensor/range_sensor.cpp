#include "range_sensor.hpp"

#include "range_sensor_private.hpp"
#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(range_sensor::Plugin, ignition::gazebo::System,
                    range_sensor::Plugin::ISystemConfigure,
                    range_sensor::Plugin::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(range_sensor::Plugin,
                          "hippo_gz_plugins::range_sensor")

namespace range_sensor {
Plugin::Plugin() : System(), private_(std::make_unique<PluginPrivate>()) {}

void Plugin::Configure(const ignition::gazebo::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       ignition::gazebo::EntityComponentManager &_ecm,
                       ignition::gazebo::EventManager &_eventMgr) {
  ignerr << "Hello";
  private_->ParseSdf(_sdf);
  if (!private_->InitModel(_ecm, _entity)) {
    ignerr << "Plugin needs to be attached to model entity." << std::endl;
    return;
  }
  private_->AdvertiseRanges();
}

void Plugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                        const ignition::gazebo::EntityComponentManager &_ecm) {
  // TODO
  private_->PublishRanges();
}
}  // namespace range_sensor

#include "buoyancy.hpp"

#include <ignition/gazebo/Conversions.hh>
#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(buoyancy::Plugin, ignition::gazebo::System,
                    buoyancy::Plugin::ISystemConfigure,
                    buoyancy::Plugin::ISystemUpdate)
IGNITION_ADD_PLUGIN_ALIAS(buoyancy::Plugin, "hippo_gz_plugins::buoyancy")

namespace buoyancy {
Plugin::Plugin() : System(), private_(std::make_unique<PluginPrivate>()) {}

void Plugin::Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    [[maybe_unused]] ignition::gazebo::EventManager &_eventMgr) {
  private_->ParseSdf(_sdf);
  if (!private_->InitModel(_ecm, _entity)) {
    ignerr << "Plugin needs to be attached to model entity." << std::endl;
    return;
  }
}
void Plugin::Update(const ignition::gazebo::UpdateInfo &_info,
                    ignition::gazebo::EntityComponentManager &_ecm) {
  if (_info.paused) {
    return;
  }

  private_->ApplyBuoyancy(_ecm);
}
}  // namespace buoyancy

#include "barometer.hpp"

#include <ignition/gazebo/Conversions.hh>
#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(barometer::Plugin, ignition::gazebo::System,
                    barometer::Plugin::ISystemConfigure,
                    barometer::Plugin::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(barometer::Plugin, "hippo_gz_plugins::barometer")

namespace barometer {
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
  private_->Advertise();
}
void Plugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                        const ignition::gazebo::EntityComponentManager &_ecm) {
  if (_info.paused) {
    return;
  }

  auto dt = _info.simTime - private_->last_pub_time_;
  if ((dt > std::chrono::steady_clock::duration::zero()) &&
      (dt < private_->update_period_)) {
    return;
  }

  private_->last_pub_time_ = _info.simTime;
  private_->Publish(
      _ecm, ignition::gazebo::convert<ignition::msgs::Time>(_info.simTime));
}
}  // namespace barometer

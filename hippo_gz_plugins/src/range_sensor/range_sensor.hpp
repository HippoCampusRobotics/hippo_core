#pragma once

#include <ignition/gazebo/System.hh>
#include <sdformat.hh>
#include "range_sensor_private.hpp"

namespace range_sensor {
class Plugin : public ignition::gazebo::System,
               public ignition::gazebo::ISystemConfigure,
               public ignition::gazebo::ISystemUpdate,
               public ignition::gazebo::ISystemPostUpdate {
 public:
  Plugin();
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;
  void Update(const ignition::gazebo::UpdateInfo &_info,
              ignition::gazebo::EntityComponentManager &_ecm) override;
  void PostUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm) override;

 private:
  std::unique_ptr<PluginPrivate> private_;
};
}  // namespace range_sensor

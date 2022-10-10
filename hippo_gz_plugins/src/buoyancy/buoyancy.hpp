#pragma once

#include <ignition/gazebo/System.hh>
#include "buoyancy_private.hpp"

namespace buoyancy {
class Plugin : public ignition::gazebo::System,
               public ignition::gazebo::ISystemConfigure,
               public ignition::gazebo::ISystemUpdate {
 public:
  Plugin();
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;
  void Update(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm) override;

 private:
  std::unique_ptr<PluginPrivate> private_;
};
}  // namespace buoyancy

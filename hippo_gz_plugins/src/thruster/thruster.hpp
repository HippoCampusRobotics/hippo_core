#pragma once

#include <ignition/gazebo/System.hh>

#include "thruster_private.hpp"

namespace thruster {
class Plugin : public ignition::gazebo::System,
               public ignition::gazebo::ISystemConfigure,
               public ignition::gazebo::ISystemPreUpdate {
 public:
  Plugin();
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;
  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override;

 private:
  std::unique_ptr<PluginPrivate> private_;
};
}  // namespace thruster

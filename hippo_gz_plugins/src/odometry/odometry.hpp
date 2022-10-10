#pragma once

#include <ignition/gazebo/System.hh>
#include "odometry_private.hpp"

namespace odometry {
class Plugin : public ignition::gazebo::System,
               public ignition::gazebo::ISystemConfigure,
               public ignition::gazebo::ISystemPostUpdate {
 public:
  Plugin();
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;
  void PostUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm) override;

 private:
  std::unique_ptr<PluginPrivate> private_;
};
}  // namespace odometry

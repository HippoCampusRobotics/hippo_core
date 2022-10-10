#pragma once

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/math.hh>

namespace hydrodynamics {
class HydrodynamicsPlugin : public ignition::gazebo::System,
                            public ignition::gazebo::ISystemConfigure,
                            public ignition::gazebo::ISystemUpdate {
 public:
  HydrodynamicsPlugin();
  ~HydrodynamicsPlugin() override;
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;
  void Update(const ignition::gazebo::UpdateInfo &_info,
              ignition::gazebo::EntityComponentManager &_ecm) override;

 private:
  ignition::math::Vector3d added_mass_linear_{0.0, 0.0, 0.0};
  ignition::math::Vector3d added_mass_angular_{0.0, 0.0, 0.0};
  ignition::math::Matrix3d damping_linear_;
  ignition::math::Matrix3d damping_angular_;

  ignition::gazebo::Entity link_entity_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Link link_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};

  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf,
                const ignition::gazebo::EntityComponentManager &_ecm);
  void ParseHydrodynamics(const sdf::ElementPtr _element,
                          const ignition::gazebo::EntityComponentManager &_ecm);
  void UpdateForcesAndMoments(ignition::gazebo::EntityComponentManager &_ecm);

  void CreateComponents(const ignition::gazebo::Entity &_entity,
                        ignition::gazebo::EntityComponentManager &_ecm);
};
}  // namespace hydrodynamics

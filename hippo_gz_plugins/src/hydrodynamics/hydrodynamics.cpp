#include "hydrodynamics.hpp"

#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>

#define SDF_MISSING_ELEMENT(x) \
  (ignerr << "Could not find [" << x << "] element in sdf." << std::endl)

IGNITION_ADD_PLUGIN(hydrodynamics::HydrodynamicsPlugin,
                    ignition::gazebo::System,
                    hydrodynamics::HydrodynamicsPlugin::ISystemConfigure,
                    hydrodynamics::HydrodynamicsPlugin::ISystemUpdate)
IGNITION_ADD_PLUGIN_ALIAS(hydrodynamics::HydrodynamicsPlugin,
                          "hippo_gz_plugins::hydrodynamics")

using namespace hydrodynamics;

HydrodynamicsPlugin::HydrodynamicsPlugin() : System() {}
HydrodynamicsPlugin::~HydrodynamicsPlugin() {}

void HydrodynamicsPlugin::Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    [[maybe_unused]] ignition::gazebo::EventManager &_eventMgr) {
  model_ = ignition::gazebo::Model(_entity);
  if (!model_.Valid(_ecm)) {
    ignerr << "Hydrodynamics Plugin can only be attached to a model entity."
           << std::endl;
    return;
  }
  ParseSdf(_sdf, _ecm);
  CreateComponents(link_entity_, _ecm);
}

void HydrodynamicsPlugin::Update(
    [[maybe_unused]] const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm) {
  if (_info.paused) {
    return;
  }
  UpdateForcesAndMoments(_ecm);
}

void HydrodynamicsPlugin::ParseSdf(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const ignition::gazebo::EntityComponentManager &_ecm) {
  for (sdf::ElementPtr element = _sdf->GetFirstElement(); element != nullptr;
       element = element->GetNextElement()) {
    if (element->GetName() != "hydrodynamics") {
      continue;
    }
    ignmsg << "Found hydrodynamics element." << std::endl;
    ParseHydrodynamics(element, _ecm);
  }
}

void HydrodynamicsPlugin::ParseHydrodynamics(
    const sdf::ElementPtr _element,
    const ignition::gazebo::EntityComponentManager &_ecm) {
  std::string name;

  name = "link";
  if (_element->HasElement(name)) {
    std::string link_name = _element->Get<std::string>(name);
    link_entity_ = model_.LinkByName(_ecm, link_name);
    link_ = ignition::gazebo::Link(link_entity_);
  } else {
    SDF_MISSING_ELEMENT(name);
  }

  name = "added_mass_linear";
  if (_element->HasElement(name)) {
    added_mass_linear_ = _element->Get<ignition::math::Vector3d>(name);
  } else {
    SDF_MISSING_ELEMENT(name);
  }

  name = "added_mass_angular";
  if (_element->HasElement(name)) {
    added_mass_angular_ = _element->Get<ignition::math::Vector3d>(name);
  } else {
    SDF_MISSING_ELEMENT(name);
  }

  name = "damping_linear";
  if (_element->HasElement(name)) {
    auto damping = _element->Get<ignition::math::Vector3d>(name);
    damping_linear_(0, 0) = -damping.X();
    damping_linear_(1, 1) = -damping.Y();
    damping_linear_(2, 2) = -damping.Z();
  } else {
    SDF_MISSING_ELEMENT(name);
  }

  name = "damping_angular";
  if (_element->HasElement(name)) {
    auto damping = _element->Get<ignition::math::Vector3d>(name);
    damping_angular_(0, 0) = -damping.X();
    damping_angular_(1, 1) = -damping.Y();
    damping_angular_(2, 2) = -damping.Z();
  } else {
    SDF_MISSING_ELEMENT(name);
  }
}

void HydrodynamicsPlugin::UpdateForcesAndMoments(
    ignition::gazebo::EntityComponentManager &_ecm) {
  if (!_ecm.HasEntity(link_.Entity())) {
    return;
  }
  auto velocity_linear =
      link_.WorldLinearVelocity(_ecm).value_or(ignition::math::Vector3d::Zero);
  auto velocity_angular =
      link_.WorldAngularVelocity(_ecm).value_or(ignition::math::Vector3d::Zero);

  auto pose = link_.WorldPose(_ecm);

  auto velocity_linear_local = pose->Rot().Inverse() * velocity_linear;
  auto velocity_angular_local = pose->Rot().Inverse() * velocity_angular;

  auto vx = velocity_linear_local.X();
  auto vy = velocity_linear_local.Y();
  auto vz = velocity_linear_local.Z();

  ignition::math::Matrix3d corriolis_force_matrix(
      0.0, added_mass_linear_.Z() * vz, -added_mass_linear_.Y() * vy,
      -added_mass_linear_.Z() * vz, 0.0, added_mass_linear_.X() * vx,
      added_mass_linear_.Y() * vy, -added_mass_linear_.X() * vx, 0.0);

  auto wx = velocity_angular_local.X();
  auto wy = velocity_angular_local.Y();
  auto wz = velocity_angular_local.Z();
  ignition::math::Matrix3d corriolis_torque_matrix(
      0.0, added_mass_angular_.Z() * wz, -added_mass_angular_.Y() * wy,
      -added_mass_angular_.Z() * wz, 0.0, added_mass_angular_.X() * wx,
      added_mass_angular_.Y() * wy, -added_mass_angular_.X() * wx, 0.0);

  auto damping_force = damping_linear_ * velocity_linear_local;
  auto damping_torque = damping_angular_ * velocity_angular_local;

  auto corriolis_force = corriolis_force_matrix * velocity_angular_local;
  auto corriolis_torque = (corriolis_force_matrix * velocity_linear_local) +
                          (corriolis_torque_matrix * velocity_angular_local);

  auto total_force = damping_force + corriolis_force;
  auto total_torque = damping_torque + corriolis_torque;

  // ignmsg << "Appling force: " << total_force
  //        << "to: " << link_.Name(_ecm).value_or("No name found") <<
  //        std::endl;
  link_.AddWorldWrench(_ecm, pose->Rot() * total_force,
                       pose->Rot() * total_torque);
}

void HydrodynamicsPlugin::CreateComponents(
    const ignition::gazebo::Entity &_entity,
    ignition::gazebo::EntityComponentManager &_ecm) {
  if (!_ecm.Component<ignition::gazebo::components::LinearVelocity>(_entity)) {
    _ecm.CreateComponent(_entity,
                         ignition::gazebo::components::LinearVelocity());
  }
  if (!_ecm.Component<ignition::gazebo::components::WorldLinearVelocity>(
          _entity)) {
    _ecm.CreateComponent(_entity,
                         ignition::gazebo::components::WorldLinearVelocity());
  }

  if (!_ecm.Component<ignition::gazebo::components::AngularVelocity>(_entity)) {
    _ecm.CreateComponent(_entity,
                         ignition::gazebo::components::AngularVelocity());
  }
  if (!_ecm.Component<ignition::gazebo::components::WorldAngularVelocity>(
          _entity)) {
    _ecm.CreateComponent(_entity,
                         ignition::gazebo::components::WorldAngularVelocity());
  }
}

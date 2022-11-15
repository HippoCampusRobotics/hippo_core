#include "thruster_private.hpp"

#include <ignition/gazebo/components/ExternalWorldWrenchCmd.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/ParentLinkName.hh>
#include <ignition/gazebo/components/Pose.hh>

#include "common.hpp"

namespace thruster {
namespace direction {
static constexpr int CCW = -1;
static constexpr int CW = 1;
}  // namespace direction

void PluginPrivate::ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf) {
  AssignSdfParam(_sdf, "link", sdf_params_.link);
  AssignSdfParam(_sdf, "joint", sdf_params_.joint);
  AssignSdfParam(_sdf, "publish_rate", sdf_params_.publish_rate);
  AssignSdfParam(_sdf, "rpm_base_topic", sdf_params_.rpm_base_topic);
  AssignSdfParam(_sdf, "thrust_base_topic", sdf_params_.thrust_base_topic);
  AssignSdfParam(_sdf, "linear_coeff", sdf_params_.linear_coeff);
  AssignSdfParam(_sdf, "quadratic_coeff", sdf_params_.quadratic_coeff);
  AssignSdfParam(_sdf, "torque_coeff", sdf_params_.torque_coeff);
  AssignSdfParam(_sdf, "rpm_scaler", sdf_params_.rpm_scaler);
  AssignSdfParam(_sdf, "maximum_rpm", sdf_params_.maximum_rpm);
  AssignSdfParam(_sdf, "thruster_number", sdf_params_.thruster_number);
  AssignSdfParam(_sdf, "turning_direction", sdf_params_.turning_direction);
  if (sdf_params_.turning_direction == "cw") {
    turning_direction_ = direction::CW;
  } else {
    turning_direction_ = direction::CCW;
  }
  AssignSdfParam(_sdf, "propeller_direction", sdf_params_.propeller_direction);
  if (sdf_params_.propeller_direction == "cw") {
    propeller_direction_ = direction::CW;
  } else {
    propeller_direction_ = direction::CCW;
  }
  AssignSdfParam(_sdf, "timeconstant_up", sdf_params_.timeconstant_up);
  AssignSdfParam(_sdf, "timeconstant_down", sdf_params_.timeconstant_down);
}

bool PluginPrivate::InitModel(ignition::gazebo::EntityComponentManager &_ecm,
                              ignition::gazebo::Entity _entity) {
  model_ = ignition::gazebo::Model(_entity);
  if (!model_.Valid(_ecm)) {
    return false;
  }
  model_name_ = model_.Name(_ecm);
  InitComponents(_ecm);
  rotor_velocity_filter_ = std::make_unique<FirstOrderFilter<double>>(
      sdf_params_.timeconstant_up, sdf_params_.timeconstant_down,
      rotor_velocity_setpoint_);
  return true;
}

void PluginPrivate::ApplyWrench(
    ignition::gazebo::EntityComponentManager &_ecm) {
  auto pose = link_.WorldPose(_ecm);
  auto parent_pose = parent_link_.WorldPose(_ecm);
  auto pose_diff = *pose - *parent_pose;
  link_.AddWorldForce(_ecm, pose->Rot().RotateVector(Thrust()));

  auto parent_wrench_component =
      _ecm.Component<ignition::gazebo::components::ExternalWorldWrenchCmd>(
          parent_link_.Entity());
  auto parent_torque = pose_diff.Rot().RotateVector(Torque());
  auto parent_torque_world = parent_pose->Rot().RotateVector(parent_torque);
  ignition::msgs::Set(
      parent_wrench_component->Data().mutable_torque(),
      ignition::msgs::Convert(parent_wrench_component->Data().torque()) +
          parent_torque_world);
}

void PluginPrivate::UpdateRotorVelocity(
    ignition::gazebo::EntityComponentManager &_ecm, double _dt) {
  {
    std::lock_guard<std::mutex> lock(thrust_cmd_mutex_);
    rotor_velocity_ =
        rotor_velocity_filter_->Update(rotor_velocity_setpoint_, _dt);
  }
  SetRotorVelocity(_ecm, rotor_velocity_);
}

ignition::math::Vector3d PluginPrivate::Thrust() {
  double thrust;
  // get rotations per second
  double tmp = std::abs(rotor_velocity_ / 6.28);
  thrust =
      tmp * tmp * sdf_params_.quadratic_coeff + tmp * sdf_params_.linear_coeff;
  if (rotor_velocity_ < 0) {
    thrust *= -1.0;
  }
  double force = turning_direction_ * propeller_direction_ * thrust;
  return ignition::math::Vector3d(force, 0, 0);
}

ignition::math::Vector3d PluginPrivate::Torque() {
  return -turning_direction_ * propeller_direction_ * Thrust() *
         sdf_params_.torque_coeff;
}

void PluginPrivate::SetRotorVelocity(ignition::gazebo::EntityComponentManager &_ecm, double _velocity) {
  auto velocity_component = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(joint_entity_);
  if (!velocity_component) {
    ignerr << "Missing JointVelocityCmd component!" << std::endl;
    return;
  }
  if (!velocity_component->Data().empty()) {
    velocity_component->Data()[0] = _velocity / sdf_params_.rpm_scaler;
  }
}

double PluginPrivate::RotorVelocity(const ignition::gazebo::EntityComponentManager &_ecm) {
  auto velocity_component = _ecm.Component<ignition::gazebo::components::JointVelocity>(joint_entity_);
  if (!velocity_component) {
    ignerr << "Joint has no JointVelocity component!" << std::endl;
    return 0.0;
  }
  if (!velocity_component->Data().empty()) {
    return velocity_component->Data()[0] * sdf_params_.rpm_scaler;
  }
  ignerr << "Joint velocity data component is empty!" << std::endl;
  return 0.0;
}

void PluginPrivate::PublishRpm(
    const ignition::gazebo::EntityComponentManager &_ecm) {
  ignition::msgs::Double msg;
  msg.set_data(RotorVelocity(_ecm));
  rpm_publisher_.Publish(msg);
}

void PluginPrivate::AdvertiseRpm() {
  rpm_publisher_ = node_.Advertise<ignition::msgs::Double>(RpmTopicName());
}

void PluginPrivate::SubscribeThrust() {
  node_.Subscribe(ThrustTopicName(), &PluginPrivate::OnThrustCmd, this);
}

void PluginPrivate::OnThrustCmd(const ignition::msgs::Double &_msg) {
  std::lock_guard<std::mutex> lock(thrust_cmd_mutex_);
  rotor_velocity_setpoint_ = ThrustToVelocity(_msg.data());
}

std::string PluginPrivate::RpmTopicName() {
  return  TopicPrefix() + "/" + sdf_params_.rpm_base_topic;
}

std::string PluginPrivate::ThrustTopicName() {
  return TopicPrefix() + "/" + sdf_params_.thrust_base_topic;
}

std::string PluginPrivate::TopicPrefix() {
  return "/" + model_name_ + "/" + "thruster_" + std::to_string(sdf_params_.thruster_number);
}

void PluginPrivate::InitComponents(
    ignition::gazebo::EntityComponentManager &_ecm) {
  //////////////////////////////////////////////////////////////////////////////
  // joint components
  //////////////////////////////////////////////////////////////////////////////
  joint_entity_ = model_.JointByName(_ecm, sdf_params_.joint);
  if (joint_entity_ == ignition::gazebo::kNullEntity) {
    ignerr << "Joint with name [" << sdf_params_.joint << "] not found!"
           << std::endl;
    return;
  }

  if (!_ecm.Component<ignition::gazebo::components::JointVelocity>(
          joint_entity_)) {
    _ecm.CreateComponent(joint_entity_,
                         ignition::gazebo::components::JointVelocity({0.0}));
  }

  if (!_ecm.Component<ignition::gazebo::components::JointVelocityCmd>(
          joint_entity_)) {
    _ecm.CreateComponent(joint_entity_,
                         ignition::gazebo::components::JointVelocityCmd({0.0}));
  }

  //////////////////////////////////////////////////////////////////////////////
  // thruster link
  //////////////////////////////////////////////////////////////////////////////
  link_ = ignition::gazebo::Link(model_.LinkByName(_ecm, sdf_params_.link));
  if (!_ecm.Component<ignition::gazebo::components::WorldPose>(
          link_.Entity())) {
    _ecm.CreateComponent(link_.Entity(),
                         ignition::gazebo::components::WorldPose());
  }

  //////////////////////////////////////////////////////////////////////////////
  // parent link components
  //////////////////////////////////////////////////////////////////////////////
  std::string parent_name =
      _ecm.Component<ignition::gazebo::components::ParentLinkName>(
              joint_entity_)
          ->Data();
  auto parent_entity = model_.LinkByName(_ecm, parent_name);
  parent_link_ = ignition::gazebo::Link(parent_entity);

  if (!_ecm.Component<ignition::gazebo::components::WorldPose>(parent_entity)) {
    _ecm.CreateComponent(parent_entity,
                         ignition::gazebo::components::WorldPose());
  }

  if (!_ecm.Component<ignition::gazebo::components::ExternalWorldWrenchCmd>(
          parent_entity)) {
    _ecm.CreateComponent(
        parent_entity, ignition::gazebo::components::ExternalWorldWrenchCmd());
  }
}
}  // namespace thruster

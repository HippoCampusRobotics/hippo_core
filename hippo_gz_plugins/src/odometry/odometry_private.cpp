#include "odometry_private.hpp"

namespace odometry {

void PluginPrivate::ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf,
                             ignition::gazebo::EntityComponentManager &_ecm) {
  sdf_params_.link = _sdf->Get<std::string>("link", sdf_params_.link).first;
  sdf_params_.base_topic =
      _sdf->Get<std::string>("base_topic", sdf_params_.base_topic).first;
  sdf_params_.update_rate =
      _sdf->Get<double>("update_rate", sdf_params_.update_rate).first;
  if (sdf_params_.update_rate > 0.0) {
    std::chrono::duration<double> period{1.0 / sdf_params_.update_rate};
    update_period_ =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  }
}

bool PluginPrivate::InitModel(ignition::gazebo::EntityComponentManager &_ecm,
                              ignition::gazebo::Entity _entity) {
  model_ = ignition::gazebo::Model(_entity);
  if (!model_.Valid(_ecm)) {
    return false;
  }
  model_name_ = model_.Name(_ecm);
  InitComponents(_ecm);
  InitHeader();
  return true;
}

void PluginPrivate::Publish(
    const ignition::gazebo::EntityComponentManager &_ecm,
    const ignition::msgs::Time &stamp) {

    msg_.mutable_header()->mutable_stamp()->CopyFrom(stamp);

    auto pose = link_.WorldPose(_ecm);
    auto v_linear = link_.WorldLinearVelocity(_ecm);
    auto v_angular = link_.WorldAngularVelocity(_ecm);
    auto v_angular_local = pose->Rot().Inverse().RotateVector(*v_angular);
    auto header = msg_.mutable_header();
    auto twist = msg_.mutable_twist();
    header->mutable_stamp()->CopyFrom(stamp);
    ignition::msgs::Set(msg_.mutable_pose(), *pose);
    ignition::msgs::Set(twist->mutable_angular(), v_angular_local);
    ignition::msgs::Set(twist->mutable_linear(), *v_linear);
    publisher_.Publish(msg_);
}

void PluginPrivate::Advertise() {
  publisher_ = node_.Advertise<ignition::msgs::Odometry>(TopicName());
}

std::string PluginPrivate::TopicName() {
  return "/" + model_name_ + "/" + sdf_params_.base_topic;
}

void PluginPrivate::InitHeader() {
  auto header = msg_.mutable_header();
  auto frame = header->add_data();
  frame->set_key("frame_id");
  frame->add_value("map");
}

void PluginPrivate::InitComponents(
    ignition::gazebo::EntityComponentManager &_ecm) {
  link_ = ignition::gazebo::Link(model_.LinkByName(_ecm, sdf_params_.link));
  if (!_ecm.Component<ignition::gazebo::components::WorldPose>(
          link_.Entity())) {
    _ecm.CreateComponent(link_.Entity(),
                         ignition::gazebo::components::WorldPose());
  }
  if (!_ecm.Component<ignition::gazebo::components::AngularVelocity>(
          link_.Entity())) {
    _ecm.CreateComponent(link_.Entity(),
                         ignition::gazebo::components::AngularVelocity());
  }
  if (!_ecm.Component<ignition::gazebo::components::LinearVelocity>(
          link_.Entity())) {
    _ecm.CreateComponent(link_.Entity(),
                         ignition::gazebo::components::LinearVelocity());
  }
}
}  // namespace barometer

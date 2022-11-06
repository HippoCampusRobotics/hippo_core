#include "barometer_private.hpp"

namespace barometer {

void PluginPrivate::ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf) {
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
  sdf_params_.atmospheric_pressure =
      _sdf->Get<double>("atmospheric_pressure",
                        sdf_params_.atmospheric_pressure)
          .first;
  sdf_params_.water_surface_offset =
      _sdf->Get<double>("water_surface_offset",
                        sdf_params_.water_surface_offset)
          .first;
  sdf_params_.position =
      _sdf->Get<ignition::math::Vector3d>("position", sdf_params_.position)
          .first;
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
  auto pose = link_.WorldPose(_ecm).value_or(ignition::math::Pose3d::Zero);
  double offset = pose.Rot().RotateVector(sdf_params_.position).Z();
  double z = pose.Pos().Z() - sdf_params_.water_surface_offset + offset;
  if (z > 0) {
    msg_.set_pressure(sdf_params_.atmospheric_pressure);
  } else {
    msg_.set_pressure(-1e4 * z + sdf_params_.atmospheric_pressure);
  }
  msg_.mutable_header()->mutable_stamp()->CopyFrom(stamp);
  publisher_.Publish(msg_);
}

void PluginPrivate::Advertise() {
  publisher_ = node_.Advertise<ignition::msgs::FluidPressure>(TopicName());
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

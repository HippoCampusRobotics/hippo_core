#include "range_sensor_private.hpp"

#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>

#include "common.hpp"

namespace range_sensor {
void PluginPrivate::ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf) {
  AssignSdfParam(_sdf, "link", sdf_params_.link);
  auto ptr = std::const_pointer_cast<sdf::Element>(_sdf);
  if (!ptr->HasElement("model") || !ptr->GetElement("model")) {
    ignerr << "No model element specified." << std::endl;
    return;
  }
  for (auto model_element = ptr->GetElement("model"); model_element;
       model_element = model_element->GetNextElement("model")) {
    auto param = model_element->GetAttribute("name");
    if (!param) {
      ignerr << "Model is missing required attribute 'name'" << std::endl;
      continue;
    }
    std::string name{"no-name"};
    std::string link{"no-name"};
    int id = -1;
    param->Get(name);
    if (!AssignSdfParam(model_element, "link", link)) {
      ignerr << "No link specified. Ignoring model [" << name << "]"
             << std::endl;
      continue;
    }
    if (!AssignSdfParam(model_element, "id", id)) {
      ignerr << "No id specified. Ignoring model [" << name << "]" << std::endl;
      continue;
    }
    TargetModel target_model;
    target_model.name = name;
    target_model.link = link;
    target_model.id = id;
    sdf_params_.target_models.push_back(target_model);
  }

  AssignSdfParam(_sdf, "update_rate", sdf_params_.update_rate);
  if (sdf_params_.update_rate > 0.0) {
    std::chrono::duration<double> period{1.0 / sdf_params_.update_rate};
    update_period_ =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  }
  AssignSdfParam(_sdf, "range_noise_stddev", sdf_params_.range_noise_stddev);
  AssignSdfParam(_sdf, "fov_angle", sdf_params_.fov_angle);
  AssignSdfParam(_sdf, "max_viewing_angle", sdf_params_.max_viewing_angle);
  AssignSdfParam(_sdf, "drop_probability", sdf_params_.drop_probability);
  AssignSdfParam(_sdf, "max_detection_distance",
                 sdf_params_.max_detection_distance);
  AssignSdfParam(_sdf, "drop_probability_exp",
                 sdf_params_.drop_probability_exp);
}

bool PluginPrivate::InitModel(ignition::gazebo::EntityComponentManager &_ecm,
                              ignition::gazebo::Entity _entity) {
  model_ = ignition::gazebo::Model(_entity);
  if (!model_.Valid(_ecm)) {
    return false;
  }
  model_name_ = model_.Name(_ecm);
  InitComponents(_ecm);
  return true;
}

std::optional<double> PluginPrivate::GetRange(
    const TargetModel &_target,
    const ignition::gazebo::EntityComponentManager &_ecm) {
  auto pose = GetPose(_ecm);
  auto target_pose = GetTargetPose(_ecm, _target);
  if (!pose) {
    ignerr << "Cannot get own model position" << std::endl;
    return std::nullopt;
  }
  if (!target_pose) {
    ignwarn << "Cannot get target model postion" << std::endl;
    return std::nullopt;
  }
  ignition::math::Vector3<double> diff = (*pose).Pos() - (*target_pose).Pos();
  return diff.Length();
}

std::optional<ignition::math::Pose3d> PluginPrivate::GetPose(
    const ignition::gazebo::EntityComponentManager &_ecm) {
  return link_.WorldPose(_ecm);
}

std::optional<ignition::math::Pose3d> PluginPrivate::GetTargetPose(
    const ignition::gazebo::EntityComponentManager &_ecm,
    const TargetModel &_target) {
  ignition::gazebo::Entity model =
      _ecm.EntityByComponents(ignition::gazebo::components::Name(_target.name),
                              ignition::gazebo::components::Model());
  if (model == ignition::gazebo::kNullEntity) {
    return std::nullopt;
  }
  ignition::gazebo::Link link = ignition::gazebo::Link(
      ignition::gazebo::Model(model).LinkByName(_ecm, _target.link));

  if (!link.Valid(_ecm)) {
    ignerr << "Link for model [" << _target.name << "] not available.";
    return std::nullopt;
  }
  return link.WorldPose(_ecm);
}

bool PluginPrivate::DropMeasurement(
    const ignition::gazebo::EntityComponentManager &_ecm,
    const TargetModel &_target) {
  auto target_pose = GetTargetPose(_ecm, _target);
  auto pose = GetPose(_ecm);

  if (!(target_pose && pose)) {
    return false;
  }
  ignition::math::Vector3d target_vec =
      target_pose.value().Pos() - pose.value().Pos();

  ignition::math::Vector3d target_normal_vec =
      target_pose.value().Rot().RotateVector(-1.0 *
                                             ignition::math::Vector3d::UnitZ);

  ignition::math::Vector3d normal_vec =
      pose.value().Rot().RotateVector(ignition::math::Vector3d::UnitX);

  double fov_angle = acos(target_vec.Dot(normal_vec) /
                          (target_vec.Length() * normal_vec.Length()));
  double viewing_angle =
      acos(target_normal_vec.Dot(normal_vec) /
           (target_normal_vec.Length() * normal_vec.Length()));
  bool is_visible = (fov_angle < sdf_params_.fov_angle) &&
                    (viewing_angle < sdf_params_.max_viewing_angle);
  if (!is_visible) {
    return true;
  }
  double p = uniform_distribution_(random_generator_);
  double p_dist = uniform_distribution_(random_generator_);
  double drop_prob_dist = DistanceDropProbability(target_vec.Length());

  bool is_dropped = p < sdf_params_.drop_probability || p_dist < drop_prob_dist;
  return is_dropped;
}

double PluginPrivate::DistanceDropProbability(double _distance) {
  double p = pow(_distance / sdf_params_.max_detection_distance,
                 sdf_params_.drop_probability_exp);
  return std::clamp(p, 0.0, 1.0);
}

void PluginPrivate::UpdateTargetComponents(
    ignition::gazebo::EntityComponentManager &_ecm) {
  for (auto &target : sdf_params_.target_models) {
    ignition::gazebo::Entity model =
        _ecm.EntityByComponents(ignition::gazebo::components::Name(target.name),
                                ignition::gazebo::components::Model());
    if (model == ignition::gazebo::kNullEntity) {
      return;
    }
    ignition::gazebo::Link link = ignition::gazebo::Link(
        ignition::gazebo::Model(model).LinkByName(_ecm, target.link));

    if (!link.Valid(_ecm)) {
      ignerr << "Link for model [" << target.name << "] not available.";
      return;
    }

    if (!_ecm.Component<ignition::gazebo::components::WorldPose>(
            link.Entity())) {
      _ecm.CreateComponent(link.Entity(),
                           ignition::gazebo::components::WorldPose());
      return;
    }
  }
}

void PluginPrivate::InitComponents(
    ignition::gazebo::EntityComponentManager &_ecm) {
  link_ = ignition::gazebo::Link(model_.LinkByName(_ecm, sdf_params_.link));
  if (!link_.Valid(_ecm)) {
    ignerr << "Link not available: " << sdf_params_.link << std::endl;
    return;
  }
  if (!_ecm.Component<ignition::gazebo::components::WorldPose>(
          link_.Entity())) {
    _ecm.CreateComponent(link_.Entity(),
                         ignition::gazebo::components::WorldPose());
  }
}

void PluginPrivate::PublishRanges(
    const ignition::gazebo::EntityComponentManager &_ecm,
    const std::chrono::steady_clock::duration &_sim_time) {
  auto dt = _sim_time - last_pub_time_;
  if (dt > std::chrono::steady_clock::duration::zero() && dt < update_period_) {
    return;
  }
  last_pub_time_ = _sim_time;
  hippo_gz_msgs::msg::RangeMeasurementArray range_array;
  for (auto &target : sdf_params_.target_models) {
    auto range = GetRange(target, _ecm);
    if (range) {
      if (DropMeasurement(_ecm, target)) {
        continue;
      }
      hippo_gz_msgs::msg::RangeMeasurement *meas =
          range_array.add_measurements();
      meas->set_id(target.id);
      meas->set_range(range.value());
    }
  }
  ranges_publisher_.Publish(range_array);
}
}  // namespace range_sensor

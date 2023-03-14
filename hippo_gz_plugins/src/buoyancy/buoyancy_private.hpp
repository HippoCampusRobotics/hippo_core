#pragma once
#include <ignition/msgs/fluid_pressure.pb.h>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <sdf/Element.hh>

namespace buoyancy {
class PluginPrivate {
 public:
  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf);

  bool InitModel(ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::Entity _entity);

  void ApplyBuoyancy(ignition::gazebo::EntityComponentManager &_ecm);

  std::chrono::steady_clock::duration update_period_{0};
  std::chrono::steady_clock::duration last_pub_time_{0};

 private:
  struct SdfParams {
    std::string link{"base_link"};
    double additional_buoyancy_force{0.0};
    double relative_compensation{1.0};
    // center of gravity relative to the origin of the link
    ignition::math::Vector3d origin{0.0, 0.0, 0.0};
    // distance over which buoyancy gets scaled down to zero at the water
    // surface.
    double height_scale_limit{0.1};
  } sdf_params_;

  void InitComponents(ignition::gazebo::EntityComponentManager &_ecm);

  ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
  std::string model_name_ = "unknown_model_name";
  ignition::gazebo::Link link_{ignition::gazebo::kNullEntity};
};
}  // namespace buoyancy

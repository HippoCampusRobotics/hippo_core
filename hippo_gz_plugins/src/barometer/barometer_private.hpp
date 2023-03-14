#pragma once
#include <ignition/msgs/fluid_pressure.pb.h>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Element.hh>

namespace barometer {
class PluginPrivate {
 public:
  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf);

  bool InitModel(ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::Entity _entity);
  void Publish(const ignition::gazebo::EntityComponentManager &_ecm,
               const ignition::msgs::Time &stamp);
  void Advertise();

  std::chrono::steady_clock::duration update_period_{0};
  std::chrono::steady_clock::duration last_pub_time_{0};

 private:
  struct SdfParams {
    std::string link{"base_link"};
    double update_rate{10.0};
    std::string base_topic{"pressure"};
    double atmospheric_pressure{101325.0};
    double water_surface_offset{0.0};
    ignition::math::Vector3d position{-1.0, 0.0, 0.0};
  } sdf_params_;

  void InitHeader();
  void InitComponents(ignition::gazebo::EntityComponentManager &_ecm);
  std::string TopicName();

  ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
  std::string model_name_ = "unknown_model_name";
  ignition::gazebo::Link link_{ignition::gazebo::kNullEntity};
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher publisher_;
  ignition::msgs::FluidPressure msg_;
};
}  // namespace barometer

#pragma once
#include <ignition/msgs/odometry.pb.h>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Element.hh>

namespace odometry {
class PluginPrivate {
 public:
  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf);

  bool InitModel(ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::Entity _entity);
  void Publish(const ignition::gazebo::EntityComponentManager &_ecm,
               const ignition::msgs::Time &stamp);
  void Advertise();

  void PublishAngularVelocity(
      const ignition::gazebo::EntityComponentManager &_ecm,
      const std::chrono::steady_clock::duration &_sim_time);

  std::chrono::steady_clock::duration update_period_{0};
  std::chrono::steady_clock::duration angular_velocity_update_period_{0};
  std::chrono::steady_clock::duration last_pub_time_{0};
  std::chrono::steady_clock::duration last_angular_velocity_pub_time_{0};

 private:
  struct SdfParams {
    std::string link{"base_link"};
    double update_rate{10.0};
    double angular_velocity_update_rate{250.0};
    std::string base_topic{"odometry"};
  } sdf_params_;

  void InitHeader();
  void InitComponents(ignition::gazebo::EntityComponentManager &_ecm);
  std::string OdometryTopicName();
  std::string AccelerationTopicName();
  std::string AngularVelocityTopicName();

  void PublishAcceleration(const ignition::gazebo::EntityComponentManager &_ecm,
                           const ignition::msgs::Time &_stamp);

  ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
  std::string model_name_ = "unknown_model_name";
  ignition::gazebo::Link link_{ignition::gazebo::kNullEntity};
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher odometry_pub_;
  ignition::transport::Node::Publisher linear_acceleration_pub_;
  ignition::transport::Node::Publisher angular_velocity_pub_;
  ignition::msgs::Odometry msg_;
};
}  // namespace odometry

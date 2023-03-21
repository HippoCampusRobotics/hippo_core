#pragma once

#include <hippo_gz_msgs/msg/range_measurement_array.pb.h>

#include <chrono>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>
#include <random>
#include <sdformat.hh>

namespace range_sensor {
class PluginPrivate {
 public:
  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf);
  bool InitModel(ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::Entity _entity);
  void PublishRanges(const ignition::gazebo::EntityComponentManager &_ecm,
                     const std::chrono::steady_clock::duration &_sim_time);
  void AdvertiseRanges() {
    ranges_publisher_ =
        node_.Advertise<hippo_gz_msgs::msg::RangeMeasurementArray>(
            RangesTopicName());
  }

  void UpdateTargetComponents(ignition::gazebo::EntityComponentManager &_ecm);

 private:
  struct TargetModel {
    int id;
    std::string name{""};
    std::string link{"base_link"};
  };

  struct SdfParams {
    std::string link{"base_link"};
    std::string ranges_base_topic{"ranges"};
    double update_rate{10.0};
    double range_noise_stddev{0.0};
    double fov_angle{90.0};
    double max_viewing_angle{140.0};
    double drop_probability{0.05};
    double max_detection_distance{5.0};
    double drop_probability_exp{2.0};
    std::vector<TargetModel> target_models;
  } sdf_params_;

  void InitComponents(ignition::gazebo::EntityComponentManager &_ecm);

  std::optional<double> GetRange(
      const TargetModel &_target,
      const ignition::gazebo::EntityComponentManager &_ecm);

  std::string TopicPrefix() { return "/" + model_name_; }

  std::string RangesTopicName() {
    return TopicPrefix() + "/" + sdf_params_.ranges_base_topic;
  }

  std::optional<ignition::math::Pose3d> GetPose(
      const ignition::gazebo::EntityComponentManager &_ecm);

  std::optional<ignition::math::Pose3d> GetTargetPose(
      const ignition::gazebo::EntityComponentManager &_ecm,
      const TargetModel &_target);

  bool DropMeasurement(const ignition::gazebo::EntityComponentManager &_ecm,
                       const TargetModel &_target);

  double DistanceDropProbability(double _distance);

  std::chrono::steady_clock::duration update_period_{0};
  std::chrono::steady_clock::duration last_pub_time_{0};

  ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Link link_{ignition::gazebo::kNullEntity};
  std::string model_name_;

  ignition::transport::Node node_;
  ignition::transport::Node::Publisher ranges_publisher_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> normal_distribution_;
  std::uniform_real_distribution<double> uniform_distribution_;
};
}  // namespace range_sensor

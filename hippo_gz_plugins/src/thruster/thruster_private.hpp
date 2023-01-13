#include <ignition/msgs/double.pb.h>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Element.hh>

namespace thruster {
template <typename T>
class FirstOrderFilter {
 public:
  FirstOrderFilter(double _tau_up, double _tau_down, T _state)
      : tau_up_(_tau_up), tau_down_(_tau_down), state_(_state) {}
  T Update(T _state, double _dt) {
    T output;
    double alpha;
    if (_state > state_) {
      alpha = exp(-_dt / tau_up_);
    } else {
      alpha = exp(-_dt / tau_down_);
    }
    output = alpha * state_ + (1.0 - alpha) * _state;
    state_ = output;
    return output;
  }

 private:
  double tau_up_;
  double tau_down_;
  T state_;
};

class PluginPrivate {
 public:
  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf);

  bool InitModel(ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::Entity _entity);
  void PublishRpm(const ignition::gazebo::EntityComponentManager &_ecm);
  void PublishThrust();
  void AdvertiseRpm() {
    rpm_publisher_ = node_.Advertise<ignition::msgs::Double>(RpmTopicName());
  }
  void AdvertiseThrust() {
    thrust_publisher_ =
        node_.Advertise<ignition::msgs::Double>(ThrustTopicName());
  }
  void SubscribeThrottleCmd() {
    node_.Subscribe(ThrottleCmdTopicName(), &PluginPrivate::OnThrottleCmd,
                    this);
  }
  void UpdateRotorVelocity(ignition::gazebo::EntityComponentManager &_ecm,
                           double dt);
  void ApplyWrench(ignition::gazebo::EntityComponentManager &_ecm);
  void ThrottleCmdTimedOut();

  std::chrono::steady_clock::duration update_period_{0};
  std::chrono::steady_clock::duration last_pub_time_{0};
  std::chrono::steady_clock::duration last_command_time_{0};
  bool throttle_cmd_updated_{false};

 private:
  struct SdfParams {
    std::string link{"base_link"};
    std::string joint;
    double publish_rate{50.0};
    std::string throttle_cmd_base_topic{"throttle_cmd"};
    std::string rpm_base_topic{"rpm"};
    std::string thrust_base_topic{"thrust"};
    int thruster_number{0};
    std::string turning_direction{"cw"};
    std::string propeller_direction{"cw"};
    double maximum_rpm{800.0};
    double rpm_scaler{10.0};
    double torque_coeff{0.0};
    double constant_coeff{0.0};
    double linear_coeff{0.0};
    double quadratic_coeff{0.0};
    double timeconstant_up{0.0};
    double timeconstant_down{0.0};
  } sdf_params_;

  void OnThrottleCmd(const ignition::msgs::Double &_msg);
  /**
   * @brief Assumes throttle [-1; 1] maps linearily to velocity
   * [-max_rotations_per_second, max_rotations_per_second]
   *
   * @param _throttle Normalized motor command in range [-1; 1]
   * @return double Rotational velocity [omega] = rad/s
   */
  double ThrottleToVelocity(double _throttle) {
    // simulate kind of deadband
    if (std::abs(_throttle) < 0.11) {
      return 0.0;
    }
    return _throttle * turning_direction_ * sdf_params_.maximum_rpm / 60.0 *
           3.14 * 2;
  }
  void InitComponents(ignition::gazebo::EntityComponentManager &_ecm);
  std::string ThrottleCmdTopicName() {
    return TopicPrefix() + "/" + sdf_params_.throttle_cmd_base_topic;
  }
  std::string RpmTopicName() {
    return TopicPrefix() + "/" + sdf_params_.rpm_base_topic;
  }
  std::string ThrustTopicName() {
    return TopicPrefix() + "/" + sdf_params_.thrust_base_topic;
  }
  std::string TopicPrefix() {
    return "/" + model_name_ + "/" + "thruster_" +
           std::to_string(sdf_params_.thruster_number);
  }
  ignition::math::Vector3d ThrusterForce();
  ignition::math::Vector3d Torque();
  void SetRotorVelocity(ignition::gazebo::EntityComponentManager &_ecm,
                        double velocity);
  double RotorVelocity(const ignition::gazebo::EntityComponentManager &_ecm);

  std::mutex thrust_cmd_mutex_;

  int turning_direction_;
  int propeller_direction_;

  std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;
  double rotor_velocity_setpoint_{0.0};
  double rotor_velocity_{0.0};

  ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
  std::string model_name_ = "unknown_model_name";
  ignition::gazebo::Link parent_link_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Link link_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity joint_entity_{ignition::gazebo::kNullEntity};

  ignition::transport::Node node_;
  ignition::transport::Node::Publisher rpm_publisher_;
  ignition::transport::Node::Publisher thrust_publisher_;
};

}  // namespace thruster

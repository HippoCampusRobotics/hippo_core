#include <eigen3/Eigen/Dense>
#include <hippo_common/param_utils.hpp>
#include <hippo_msgs/msg/actuator_controls.hpp>
#include <hippo_msgs/msg/attitude_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hippo_control/attitude_control/geometric_attitude_control.hpp"

using namespace hippo_control::attitude_control;
using namespace hippo_common;
using namespace hippo_msgs::msg;
using namespace rcl_interfaces::msg;
using namespace geometry_msgs::msg;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;

class AttitudeControlNode : public rclcpp::Node {
 public:
  AttitudeControlNode() : Node("attitude_controller") {
    RCLCPP_INFO(get_logger(), "Declaring parameters.");
    DeclareParams();
    InitPublishers();
    InitSubscriptions();
  }
  void DeclareParams() {
    std::string name;
    rcl_interfaces::msg::ParameterDescriptor descr;
    std::string descr_text;

    name = "feedthrough";
    descr_text =
        "Directly feed through roll, pitch and yaw without using any control "
        "law.";
    descr = param_utils::Description(descr_text, true);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      feedthrough_ = false;
      feedthrough_ = declare_parameter(name, feedthrough_, descr);
    }

    name = "gain.roll.p";
    descr_text = "Proportional gain for roll.";
    descr = param_utils::Description(descr_text);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      gain_roll_p_ = 1.0;
      gain_roll_p_ = declare_parameter(name, gain_roll_p_, descr);
      controller_.SetRollGainP(gain_roll_p_);
    }

    name = "gain.pitch.p";
    descr_text = "Proportional gain for pitch.";
    descr = param_utils::Description(descr_text);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      gain_pitch_p_ = 1.0;
      gain_pitch_p_ = declare_parameter(name, gain_pitch_p_, descr);
      controller_.SetPitchGainP(gain_pitch_p_);
    }

    name = "gain.yaw.p";
    descr_text = "Proportional gain for yaw.";
    descr = param_utils::Description(descr_text);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      gain_yaw_p_ = 1.0;
      gain_yaw_p_ = declare_parameter(name, gain_yaw_p_, descr);
      controller_.SetYawGainP(gain_yaw_p_);
    }

    name = "gain.roll.d";
    descr_text = "Derivative gain for roll.";
    descr = param_utils::Description(descr_text);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      gain_roll_d_ = 1.0;
      gain_roll_d_ = declare_parameter(name, gain_roll_d_, descr);
      controller_.SetRollGainD(gain_roll_d_);
    }

    name = "gain.pitch.d";
    descr_text = "Derivative gain for pitch.";
    descr = param_utils::Description(descr_text);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      gain_pitch_d_ = 1.0;
      gain_pitch_d_ = declare_parameter(name, gain_pitch_d_, descr);
      controller_.SetPitchGainD(gain_pitch_d_);
    }

    name = "gain.yaw.d";
    descr_text = "Derivative gain for yaw.";
    descr = param_utils::Description(descr_text);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      gain_yaw_d_ = 1.0;
      gain_yaw_d_ = declare_parameter(name, gain_yaw_d_, descr);
      controller_.SetYawGainD(gain_yaw_d_);
    }

    name = "scale_output";
    descr_text =
        "If true, the output gets evenly to be in range[-1, 1]. Otherwise it "
        "gets clipped";
    descr = param_utils::Description(descr_text);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      scale_output_ = false;
      scale_output_ = declare_parameter(name, scale_output_, descr);
    }

    misc_cb_handle_ = add_on_set_parameters_callback(
        std::bind(&AttitudeControlNode::OnSetMisc, this, _1));
    p_gains_cb_handle_ = add_on_set_parameters_callback(
        std::bind(&AttitudeControlNode::OnSetPgains, this, _1));
    d_gains_cb_handle_ = add_on_set_parameters_callback(
        std::bind(&AttitudeControlNode::OnSetDgains, this, _1));
  }

  void InitPublishers() {
    std::string topic;
    rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

    topic = "actuator_control";
    control_output_pub_ = create_publisher<ActuatorControls>(topic, qos);

    topic = "~/current_setpoint";
    setpoint_pub_ = create_publisher<AttitudeTarget>(topic, qos);
  }

  void InitSubscriptions() {
    std::string topic;
    rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

    topic = "attitude_target";
    target_sub_ = create_subscription<AttitudeTarget>(
        topic, qos,
        std::bind(&AttitudeControlNode::OnAttitudeTarget, this, _1));

    topic = "odometry";
    odometry_sub_ = create_subscription<Odometry>(
        topic, qos, std::bind(&AttitudeControlNode::OnOdometry, this, _1));
  }

  void OnAttitudeTarget(const AttitudeTarget::SharedPtr _msg) {
    if (_msg->header.frame_id != "map") {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "AttitudeTarget frame is [%s] but only [map] is handled. Ignoring...",
          _msg->header.frame_id.c_str());
          return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!(_msg->mask & _msg->IGNORE_ROLL_ANGLE)) {
      attitude_target_.attitude.x = _msg->attitude.x;
    }
    if (!(_msg->mask & _msg->IGNORE_PITCH_ANGLE)) {
      attitude_target_.attitude.y = _msg->attitude.y;
    }
    if (!(_msg->mask & _msg->IGNORE_YAW_ANGLE)) {
      attitude_target_.attitude.z = _msg->attitude.z;
    }
    if (!(_msg->mask & _msg->IGNORE_ROLL_RATE)) {
      attitude_target_.body_rate.x = _msg->body_rate.x;
    }
    if (!(_msg->mask & _msg->IGNORE_PITCH_RATE)) {
      attitude_target_.body_rate.y = _msg->body_rate.y;
    }
    if (!(_msg->mask & _msg->IGNORE_YAW_RATE)) {
      attitude_target_.body_rate.z = _msg->body_rate.z;
    }
    if (!(_msg->mask & _msg->IGNORE_THRUST)) {
      attitude_target_.thrust = _msg->thrust;
    }
    controller_.SetAngularVelocityTarget(attitude_target_.body_rate.x,
                                         attitude_target_.body_rate.y,
                                         attitude_target_.body_rate.z);
    controller_.SetOrientationTarget(attitude_target_.attitude.x,
                                     attitude_target_.attitude.y,
                                     attitude_target_.attitude.z);
    setpoint_pub_->publish(attitude_target_);
    if (feedthrough_) {
      ActuatorControls msg;
      msg.header.stamp = now();
      msg.control[msg.INDEX_ROLL] = attitude_target_.body_rate.x;
      msg.control[msg.INDEX_PITCH] = attitude_target_.body_rate.y;
      msg.control[msg.INDEX_YAW] = attitude_target_.body_rate.z;
      msg.control[msg.INDEX_THRUST] = attitude_target_.thrust;
      control_output_pub_->publish(msg);
    }
  }

  void OnOdometry(const Odometry::SharedPtr _msg) {
    if (feedthrough_) {
      return;
    }
    const auto &q_ros = _msg->pose.pose.orientation;
    const auto &omega = _msg->twist.twist.angular;

    Eigen::Quaterniond orientation{q_ros.w, q_ros.x, q_ros.y, q_ros.z};
    Eigen::Vector3d v_angular{omega.x, omega.y, omega.z};

    ActuatorControls msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      Eigen::Vector3d u =
          controller_.Update(orientation, v_angular, scale_output_);
      msg.control[ActuatorControls::INDEX_ROLL] = u.x();
      msg.control[ActuatorControls::INDEX_PITCH] = u.y();
      msg.control[ActuatorControls::INDEX_YAW] = u.z();
      msg.control[ActuatorControls::INDEX_THRUST] = attitude_target_.thrust;
    }
    control_output_pub_->publish(msg);
  }

  SetParametersResult OnSetMisc(
      const std::vector<rclcpp::Parameter> &parameters) {
    SetParametersResult result;
    result.successful = true;
    for (const rclcpp::Parameter &parameter : parameters) {
      std::lock_guard<std::mutex> lock(mutex_);

      if (param_utils::AssignIfMatch(parameter, "scale_output",
                                     scale_output_)) {
        continue;
      }
    }
    return result;
  }

  SetParametersResult OnSetPgains(
      const std::vector<rclcpp::Parameter> &parameters) {
    SetParametersResult result;
    result.successful = true;
    for (const rclcpp::Parameter &parameter : parameters) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (param_utils::AssignIfMatch(parameter, "gain.roll.p", gain_roll_p_)) {
        controller_.SetRollGainP(gain_roll_p_);
        continue;
      }
      if (param_utils::AssignIfMatch(parameter, "gain.pitch.p",
                                     gain_pitch_p_)) {
        controller_.SetPitchGainP(gain_pitch_p_);
        continue;
      }
      if (param_utils::AssignIfMatch(parameter, "gain.yaw.p", gain_yaw_p_)) {
        controller_.SetYawGainP(gain_yaw_p_);
        continue;
      }
    }
    return result;
  }
  SetParametersResult OnSetDgains(
      const std::vector<rclcpp::Parameter> &parameters) {
    SetParametersResult result;
    result.successful = true;
    for (const rclcpp::Parameter &parameter : parameters) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (param_utils::AssignIfMatch(parameter, "gain.roll.d", gain_roll_d_)) {
        controller_.SetRollGainD(gain_roll_d_);
        continue;
      }
      if (param_utils::AssignIfMatch(parameter, "gain.pitch.d", gain_yaw_d_)) {
        controller_.SetPitchGainD(gain_yaw_d_);
        continue;
      }
      if (param_utils::AssignIfMatch(parameter, "gain.yaw.d", gain_yaw_d_)) {
        controller_.SetYawGainD(gain_yaw_d_);
        continue;
      }
    }
    return result;
  }

 private:
  std::mutex mutex_;

  //////////////////////////////////////////////////////////////////////////////
  // ros params
  //////////////////////////////////////////////////////////////////////////////
  double gain_roll_p_;
  double gain_roll_d_;
  double gain_pitch_p_;
  double gain_pitch_d_;
  double gain_yaw_p_;
  double gain_yaw_d_;
  bool scale_output_;

  GeometricAttitudeControl controller_;
  AttitudeTarget attitude_target_;

  bool feedthrough_;

  //////////////////////////////////////////////////////////////////////////////
  // publisher
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<ActuatorControls>::SharedPtr control_output_pub_;
  rclcpp::Publisher<AttitudeTarget>::SharedPtr setpoint_pub_;

  //////////////////////////////////////////////////////////////////////////////
  // subscriber
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<AttitudeTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_sub_;

  OnSetParametersCallbackHandle::SharedPtr p_gains_cb_handle_;
  OnSetParametersCallbackHandle::SharedPtr d_gains_cb_handle_;
  OnSetParametersCallbackHandle::SharedPtr misc_cb_handle_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AttitudeControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

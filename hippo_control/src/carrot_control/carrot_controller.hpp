#pragma once
#include <hippo_msgs/msg/attitude_target.hpp>
#include <hippo_msgs/msg/float64_stamped.hpp>
#include <hippo_msgs/srv/set_path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <path_planning/path.hpp>
#include <path_planning/rviz_helper.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hippo_control {
namespace carrot_control {

class CarrotController : public rclcpp::Node {
 public:
  explicit CarrotController(rclcpp::NodeOptions const &_options);

 private:
  struct Params {
    bool updated{false};
    double look_ahead_distance{0.3};
    double depth_gain;
    std::string path_file{""};
  };
  void LoadDefaultWaypoints();
  std::string GetWaypointsFilePath();
  void DeclareParams();
  rcl_interfaces::msg::SetParametersResult OnParams(
      const std::vector<rclcpp::Parameter> &params);
  void InitPublishers();
  void InitSubscriptions();
  void PublishAttitudeTarget(const rclcpp::Time &_now,
                             const Eigen::Quaterniond &_attitude,
                             double _thrust);
  void OnSetPath(const hippo_msgs::srv::SetPath::Request::SharedPtr _req,
                 hippo_msgs::srv::SetPath::Response::SharedPtr _response);
  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr _msg);
  void OnThrust(const hippo_msgs::msg::Float64Stamped::SharedPtr msg);
  std::shared_ptr<path_planning::Path> path_{nullptr};
  rclcpp::Publisher<hippo_msgs::msg::AttitudeTarget>::SharedPtr attitude_pub_{
      nullptr};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_{
      nullptr};
  rclcpp::Service<hippo_msgs::srv::SetPath>::SharedPtr set_path_service_{
      nullptr};
  rclcpp::Subscription<hippo_msgs::msg::Float64Stamped>::SharedPtr thrust_sub_{
      nullptr};

  OnSetParametersCallbackHandle::SharedPtr params_cb_handle_{nullptr};
  Params params_;

  Eigen::Quaterniond attitude_{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d position_{0.0, 0.0, 0.0};
  Eigen::Vector3d target_position_{0.0, 0.0, 0.0};
  Eigen::Quaterniond target_attitude_{1.0, 0.0, 0.0, 0.0};
  double thrust_{0.3};

  std::shared_ptr<path_planning::RvizHelper> path_visualizer_{nullptr};
};

}  // namespace carrot_control
}  // namespace hippo_control

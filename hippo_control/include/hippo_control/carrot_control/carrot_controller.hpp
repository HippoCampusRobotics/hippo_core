#pragma once
#include <hippo_msgs/msg/attitude_target.hpp>
#include <hippo_msgs/srv/set_path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <path_planning/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hippo_control {
namespace carrot_control {

class CarrotController : public rclcpp::Node {
 public:
  explicit CarrotController(rclcpp::NodeOptions const &_options);

 private:
  struct Params {
    double look_ahead_distance{0.3};
  };
  void InitPublishers();
  void InitSubscriptions();
  void PublishAttitudeTarget(const rclcpp::Time &_now,
                             const Eigen::Quaterniond &_attitude,
                             double _thrust);
  void OnSetPath(const hippo_msgs::srv::SetPath::Request::SharedPtr _req,
                 hippo_msgs::srv::SetPath::Response::SharedPtr _response);
  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr _msg);
  std::shared_ptr<path_planning::Path> path_{nullptr};
  rclcpp::Publisher<hippo_msgs::msg::AttitudeTarget>::SharedPtr attitude_pub_{
      nullptr};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_{
      nullptr};
  rclcpp::Service<hippo_msgs::srv::SetPath>::SharedPtr set_path_service_{
      nullptr};
  Params params_;

  Eigen::Quaterniond attitude_{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d position_{0.0, 0.0, 0.0};
  Eigen::Vector3d target_position_{0.0, 0.0, 0.0};
  Eigen::Quaterniond target_attitude_{1.0, 0.0, 0.0, 0.0};
  double thrust_{2.0};
};

}  // namespace carrot_control
}  // namespace hippo_control

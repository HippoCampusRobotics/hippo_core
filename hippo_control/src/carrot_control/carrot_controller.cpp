#include "carrot_controller.hpp"

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hippo_common/convert.hpp>
#include <hippo_common/tf2_utils.hpp>
#include <hippo_common/yaml.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace hippo_control {
namespace carrot_control {

CarrotController::CarrotController(rclcpp::NodeOptions const &_options)
    : Node("carrot_controller", _options) {
  rclcpp::Node::SharedPtr rviz_node = create_sub_node("visualization");
  path_visualizer_ = std::make_shared<path_planning::RvizHelper>(rviz_node);
  DeclareParams();
  LoadDefaultWaypoints();
  set_path_service_ = create_service<hippo_msgs::srv::SetPath>(
      "~/set_path", std::bind(&CarrotController::OnSetPath, this,
                              std::placeholders::_1, std::placeholders::_2));
  InitPublishers();
  InitSubscriptions();
  RCLCPP_INFO(get_logger(), "Initialization done.");
}

std::string CarrotController::GetWaypointsFilePath() {
  if (!params_.path_file.empty()) {
    return params_.path_file;
  }

  const std::string pkg{"path_planning"};
  std::string file_path;
  try {
    file_path = ament_index_cpp::get_package_share_directory(pkg);
  } catch (const ament_index_cpp::PackageNotFoundError &) {
    RCLCPP_ERROR(
        get_logger(),
        "Failed to load default waypoints because [%s] could not be found.",
        pkg.c_str());
    return "";
  }
  file_path += "/config/bernoulli_default.yaml";
  return file_path;
}

void CarrotController::LoadDefaultWaypoints() {
  path_ = std::make_shared<path_planning::Path>();
  std::string file_path = GetWaypointsFilePath();
  try {
    path_->LoadFromYAML(file_path);
  } catch (const YAML::ParserException &) {
    RCLCPP_ERROR(get_logger(), "Failed to parse default waypoints at [%s]",
                 file_path.c_str());
    path_ = nullptr;
    return;
  } catch (const YAML::BadFile &) {
    RCLCPP_ERROR(get_logger(),
                 "Failed to load default waypoints at [%s]: bad file.",
                 file_path.c_str());
    path_ = nullptr;
    return;
  }
  RCLCPP_INFO(get_logger(), "Loaded default waypoints at [%s]",
              file_path.c_str());
  path_->SetLookAhead(params_.look_ahead_distance);
}

void CarrotController::InitPublishers() {
  std::string topic;

  topic = "attitude_target";
  attitude_pub_ = create_publisher<hippo_msgs::msg::AttitudeTarget>(topic, 10);
}

void CarrotController::InitSubscriptions() {
  std::string topic;

  topic = "odometry";
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      topic, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        OnOdometry(msg);
      });

  topic = "thrust";
  thrust_sub_ = create_subscription<hippo_msgs::msg::Float64Stamped>(
      topic, 10, [this](const hippo_msgs::msg::Float64Stamped::SharedPtr msg) {
        OnThrust(msg);
      });
}

void CarrotController::OnSetPath(
    const hippo_msgs::srv::SetPath::Request::SharedPtr _req,
    hippo_msgs::srv::SetPath::Response::SharedPtr _response) {
  RCLCPP_INFO(get_logger(), "Handling SetPath Request.");
  if (_req->path.waypoints.size() < 2) {
    _response->success = false;
    _response->reason = "Too few waypoints. At least 2 are required. Got " +
                        std::to_string(_req->path.waypoints.size());
    return;
  }

  std::vector<Eigen::Vector3d> waypoints;
  for (auto waypoint : _req->path.waypoints) {
    Eigen::Vector3d tmp;
    hippo_common::convert::RosToEigen(waypoint, tmp);
    waypoints.push_back(tmp);
  }
  path_ = std::make_shared<path_planning::Path>(waypoints, _req->path.is_loop);
  path_->SetLookAhead(params_.look_ahead_distance);
  _response->success = true;
}

void CarrotController::OnOdometry(
    const nav_msgs::msg::Odometry::SharedPtr _msg) {
  hippo_common::convert::RosToEigen(_msg->pose.pose.orientation, attitude_);
  hippo_common::convert::RosToEigen(_msg->pose.pose.position, position_);
  if (path_ == nullptr) {
    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 2000,
                                "No path has been set.");
    return;
  }
  if (params_.updated) {
    params_.updated = false;
    path_->SetLookAhead(params_.look_ahead_distance);
  }
  bool success = path_->Update(position_);
  if (!success) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not find target waypoint.");
    return;
  }
  target_position_ = path_->TargetPoint();
  Eigen::Vector3d heading{target_position_ - position_};
  heading.z() *= params_.depth_gain;
  target_attitude_ =
      hippo_common::tf2_utils::QuaternionFromHeading(heading, 0.0);
  PublishAttitudeTarget(_msg->header.stamp, target_attitude_, thrust_);

  if (path_visualizer_ != nullptr) {
    path_visualizer_->PublishPath(path_);
  }
}

void CarrotController::OnThrust(
    const hippo_msgs::msg::Float64Stamped::SharedPtr _msg) {
  thrust_ = _msg->data;
}

void CarrotController::PublishAttitudeTarget(
    const rclcpp::Time &_now, const Eigen::Quaterniond &_attitude,
    double _thrust) {
  if (attitude_pub_ == nullptr) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Attitude Target Publisher not available.");
    return;
  }
  hippo_msgs::msg::AttitudeTarget msg;
  msg.header.stamp = _now;
  msg.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(_attitude, msg.attitude);
  msg.thrust = _thrust;
  attitude_pub_->publish(msg);
}

}  // namespace carrot_control
}  // namespace hippo_control
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hippo_control::carrot_control::CarrotController)

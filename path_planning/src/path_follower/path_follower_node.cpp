// Copyright (C) 2023 Thies Lennart Alff

// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA

#include "path_follower_node.hpp"

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hippo_common/convert.hpp>
#include <hippo_common/tf2_utils.hpp>
#include <hippo_common/yaml.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace path_planning {

PathFollowerNode::PathFollowerNode(rclcpp::NodeOptions const &_options)
    : Node("path_follower", _options) {
  rclcpp::Node::SharedPtr rviz_node = create_sub_node("visualization");
  path_visualizer_ = std::make_shared<path_planning::RvizHelper>(rviz_node);
  DeclareParams();
  LoadDefaultWaypoints();
  InitServices();
  InitPublishers();
  InitSubscriptions();
  RCLCPP_INFO(get_logger(), "Initialization done.");
}

void PathFollowerNode::InitServices() {
  std::string name;

  name = "~/set_path";
  set_path_service_ = create_service<hippo_msgs::srv::SetPath>(
      name, std::bind(&PathFollowerNode::OnSetPath, this, std::placeholders::_1,
                      std::placeholders::_2));

  name = "~/start";
  start_service_ = create_service<std_srvs::srv::Trigger>(
      name, [this](const std_srvs::srv::Trigger::Request::SharedPtr request,
                   std_srvs::srv::Trigger::Response::SharedPtr response) {
        OnStart(request, response);
      });
}

std::string PathFollowerNode::GetWaypointsFilePath() {
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

void PathFollowerNode::LoadDefaultWaypoints() {
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

void PathFollowerNode::InitPublishers() {
  std::string topic;
  rclcpp::QoS qos =
      rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

  topic = "heading_target";
  heading_target_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(topic, qos);

  topic = "~/debug";
  debug_pub_ = create_publisher<hippo_msgs::msg::PathFollowerDebug>(topic, qos);
}

void PathFollowerNode::InitSubscriptions() {
  std::string topic;

  topic = "odometry";
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      topic, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        OnOdometry(msg);
      });
}

void PathFollowerNode::PublishDistanceError(const rclcpp::Time &_now,
                                            const Eigen::Vector3d &_error) {
  hippo_msgs::msg::PathFollowerDebug msg;
  msg.header.stamp = _now;
  using hippo_common::convert::EigenToRos;
  EigenToRos(_error, msg.error_vec);
  EigenToRos(support_vector_, msg.support_vec);
  EigenToRos(direction_vector_, msg.direction_vec);
  msg.mode = params_.mode;
  msg.look_ahead_distance = params_.look_ahead_distance;
  debug_pub_->publish(msg);
}

void PathFollowerNode::OnSetPath(
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

void PathFollowerNode::OnStart(
    const std_srvs::srv::Trigger::Request::SharedPtr _request,
    std_srvs::srv::Trigger::Response::SharedPtr _response) {
  RCLCPP_INFO(get_logger(), "Handling Start Request.");
  switch (params_.mode) {
    case mode::kStaticAxis: {
      SetDesiredStaticAxis();
      _response->success = true;
      _response->message = "Follow static axis.";
    } break;
    case mode::kPoseBasedAxis: {
      _response->success = StartPoseBasedAxis();
      _response->message = "Follow pose based axis.";
    } break;
    case mode::kStaticHeading: {
      SetStaticHeading();
      _response->success = true;
      _response->message = "Follow static heading";
    } break;
    case mode::kPoseBasedHeading: {
      direction_vector_ = orientation_ * Eigen::Vector3d::UnitX();
      _response->success = true;
      _response->message = "Follow pose based heading.";
    } break;
    case mode::kStaticPath: {
      _response->success = true;
      _response->message = "Follow static path.";
      // TODO? depends on waypoint selection implementation. If only y-projected
      // lookahead distance is used for selection, no action required. Otherwise
      // make sure to reset the path to the start
    } break;
    default:
      RCLCPP_ERROR(get_logger(), "Unhandled mode: %d", params_.mode);
      _response->success = false;
      _response->message = "Unhandled mode.";
      break;
  }
  RCLCPP_INFO_STREAM(get_logger(), _response->message);
}

bool PathFollowerNode::StartPoseBasedAxis() {
  Eigen::Vector3d heading = orientation_ * Eigen::Vector3d::UnitX();
  if (AxisCollidesWithWall(position_, heading)) {
    return false;
  }
  if (AxisCollidesWithSurface(position_, heading)) {
    return false;
  }
  SetDesiredDynamicAxis(position_, heading);
  return true;
}

void PathFollowerNode::SetDesiredStaticAxis() {
  support_vector_ = Eigen::Vector3d{params_.static_axis.position.x,
                                    params_.static_axis.position.y,
                                    params_.static_axis.position.z};
  direction_vector_ = Eigen::Vector3d{params_.static_axis.heading.x,
                                      params_.static_axis.heading.y,
                                      params_.static_axis.heading.z};
  direction_vector_.normalize();
}

void PathFollowerNode::SetDesiredDynamicAxis(
    const Eigen::Vector3d &_support_vector, const Eigen::Vector3d &_direction) {
  support_vector_ = _support_vector;
  direction_vector_ = _direction;
  direction_vector_.normalize();
}

void PathFollowerNode::SetStaticHeading() {
  direction_vector_ =
      Eigen::Vector3d{params_.static_heading.x, params_.static_heading.y,
                      params_.static_heading.z};
  direction_vector_.normalize();
}

Eigen::Vector3d PathFollowerNode::ClosestPointToAxis() {
  Eigen::Vector3d v = position_ - support_vector_;
  return direction_vector_ * v.dot(direction_vector_) + support_vector_;
}

bool PathFollowerNode::AxisCollidesWithWall(
    const Eigen::Vector3d &_support_vector,
    const Eigen::Vector3d &_direction_vector) {
  // left wall
  if (AxisCollidesWithWall(_support_vector, _direction_vector,
                           Eigen::Vector3d{params_.left_wall, 0.0, 0.0},
                           Eigen::Vector3d{1.0, 0.0, 0.0})) {
    return true;
  }
  // right wall
  if (AxisCollidesWithWall(_support_vector, _direction_vector,
                           Eigen::Vector3d{params_.right_wall, 0.0, 0.0},
                           Eigen::Vector3d{-1.0, 0.0, 0.0})) {
    return true;
  }
  // bottom_wall
  if (AxisCollidesWithWall(_support_vector, _direction_vector,
                           Eigen::Vector3d{0.0, 0.0, params_.bottom_wall},
                           Eigen::Vector3d{0.0, 0.0, 1.0})) {
    return true;
  }
  return false;
}

bool PathFollowerNode::AxisCollidesWithWall(
    const Eigen::Vector3d &support_vector,
    const Eigen::Vector3d &direction_vector, const Eigen::Vector3d &point,
    const Eigen::Vector3d &normal) {
  double distance = params_.domain_end - support_vector.y();
  double scaler = distance / direction_vector.y();
  Eigen::Vector3d final_position{scaler * direction_vector + support_vector};

  if ((final_position - point).dot(normal) <= 0) {
    return true;
  }
  return false;
}

bool PathFollowerNode::AxisCollidesWithSurface(
    const Eigen::Vector3d &_support_vector,
    const Eigen::Vector3d &_direction_vector) {
  return AxisCollidesWithWall(_support_vector, _direction_vector,
                              Eigen::Vector3d{0.0, 0.0, params_.surface},
                              Eigen::Vector3d{0.0, 0.0, -1.0});
}

void PathFollowerNode::OnOdometry(
    const nav_msgs::msg::Odometry::SharedPtr _msg) {
  hippo_common::convert::RosToEigen(_msg->pose.pose.orientation, orientation_);
  hippo_common::convert::RosToEigen(_msg->pose.pose.position, position_);
  if (path_ == nullptr) {
    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 2000,
                                "No path has been set.");
    return;
  }
  switch (params_.mode) {
    case mode::kPoseBasedAxis:
    case mode::kStaticAxis: {
      Eigen::Vector3d closest_point = ClosestPointToAxis();
      Eigen::Vector3d target_point =
          params_.look_ahead_distance * direction_vector_ + closest_point;
      target_heading_ = (target_point - position_).normalized();

      Eigen::Vector3d position_error = closest_point - position_;
      PublishDistanceError(_msg->header.stamp, position_error);
      // TODO: for a posed base axis we need to express the position error in
      // a frame aligned with the axis, i.e. e_x = direction_vector
      break;
    }
    case mode::kPoseBasedHeading:
    case mode::kStaticHeading:
      target_heading_ = direction_vector_;
      break;
    case mode::kStaticPath:
      // TODO
      break;
  }
  PublishHeadingTarget(_msg->header.stamp, target_heading_);

  // bool success = path_->Update(position_);
  // if (!success) {
  //   RCLCPP_ERROR_STREAM(get_logger(), "Could not find target waypoint.");
  //   return;
  // }
  // target_position_ = path_->TargetPoint();
  // Eigen::Vector3d heading{target_position_ - position_};
  // heading.z() *= params_.depth_gain;
  if (path_visualizer_ != nullptr) {
    path_visualizer_->PublishPath(path_);
  }
}

void PathFollowerNode::PublishHeadingTarget(const rclcpp::Time &_now,
                                            const Eigen::Vector3d &_heading) {
  if (heading_target_pub_ == nullptr) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Attitude Target Publisher not available.");
    return;
  }
  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = _now;
  msg.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(_heading, msg.vector);
  heading_target_pub_->publish(msg);
}

}  // namespace path_planning
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(path_planning::PathFollowerNode)

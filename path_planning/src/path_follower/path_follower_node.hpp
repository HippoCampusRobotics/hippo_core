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

#pragma once

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hippo_msgs/msg/float64_stamped.hpp>
#include <hippo_msgs/srv/set_path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <path_planning/path.hpp>
#include <path_planning/rviz_helper.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

namespace path_planning {

class PathFollowerNode : public rclcpp::Node {
 public:
  explicit PathFollowerNode(rclcpp::NodeOptions const &_options);

 private:
  struct Params {
    bool updated{false};
    double depth_gain;
    double look_ahead_distance;
    bool ignore_z_distance;
    std::string path_file;
  };
  void DeclareParams();
  void InitPublishers();
  void InitSubscriptions();
  void PublishHeadingTarget(const rclcpp::Time &now,
                            const Eigen::Vector3d &heading);
  void PublishDistanceError(const rclcpp::Time &now, double error);
  void LoadDefaultWaypoints();
  std::string GetWaypointsFilePath();

  //////////////////////////////////////////////////////////////////////////////
  // Callbacks
  //////////////////////////////////////////////////////////////////////////////
  //   void OnHeadingTarget(const hippo_msgs::msg::HeadingTarget::SharedPtr);
  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr);
  void OnSetPath(const hippo_msgs::srv::SetPath::Request::SharedPtr _req,
                 hippo_msgs::srv::SetPath::Response::SharedPtr _response);

  rcl_interfaces::msg::SetParametersResult OnParameters(
      const std::vector<rclcpp::Parameter> &parameters);

  //////////////////////////////////////////////////////////////////////////////
  // Publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      heading_target_pub_;
  rclcpp::Publisher<hippo_msgs::msg::Float64Stamped>::SharedPtr
      distance_error_debug_pub_;

  //////////////////////////////////////////////////////////////////////////////
  // Subscriptions
  //////////////////////////////////////////////////////////////////////////////
  //   rclcpp::Subscription<hippo_msgs::msg::HeadingTarget>::SharedPtr
  //       heading_target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  rclcpp::Service<hippo_msgs::srv::SetPath>::SharedPtr set_path_service_;

  Eigen::Quaterniond orientation_{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d position_{0.0, 0.0, 0.0};
  Eigen::Vector3d target_position_{0.0, 0.0, 0.0};
  Eigen::Vector3d target_heading_{1.0, 0.0, 0.0};
  std::shared_ptr<Path> path_{nullptr};
  std::shared_ptr<RvizHelper> path_visualizer_;

  OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;

  Params params_;
};

}  // namespace path_planning

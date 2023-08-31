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
#include <hippo_msgs/msg/path_follower_debug.hpp>
#include <hippo_msgs/srv/set_path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <path_planning/path.hpp>
#include <path_planning/rviz_helper.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace path_planning {
namespace mode {
enum Mode {
  kStaticAxis = 0,
  kPoseBasedAxis = 1,
  kStaticHeading = 2,
  kPoseBasedHeading = 3,
  kStaticPath = 4,
};
}  // namespace mode
class PathFollowerNode : public rclcpp::Node {
 public:
  enum class Mode {};
  explicit PathFollowerNode(rclcpp::NodeOptions const &_options);

 private:
  struct Vec3d {
    double x;
    double y;
    double z;
  };
  struct Axis {
    Vec3d position;
    Vec3d heading;
  };
  struct Params {
    bool updated{false};
    double depth_gain;
    double look_ahead_distance;
    bool ignore_z_distance;
    int mode;
    Axis static_axis;
    Vec3d static_heading;
    std::string path_file;
    double left_wall;
    double right_wall;
    double bottom_wall;
    double surface;
    double domain_end;
  };
  void DeclareParams();
  void InitPublishers();
  void InitSubscriptions();
  void InitServices();
  void PublishHeadingTarget(const rclcpp::Time &now,
                            const Eigen::Vector3d &heading);
  void PublishDistanceError(const rclcpp::Time &now,
                            const Eigen::Vector3d &error);
  bool AxisCollidesWithWall(const Eigen::Vector3d &support_vector,
                            const Eigen::Vector3d &direction_vector);
  bool AxisCollidesWithWall(const Eigen::Vector3d &support_vector,
                            const Eigen::Vector3d &direction_vector,
                            const Eigen::Vector3d &point,
                            const Eigen::Vector3d &normal);
  bool AxisCollidesWithSurface(const Eigen::Vector3d &support_vector,
                               const Eigen::Vector3d &direction_vector);
  void LoadDefaultWaypoints();
  std::string GetWaypointsFilePath();
  /**
   * Set desired axis based on statically set ros parameters
   */
  void SetDesiredStaticAxis();
  void SetDesiredDynamicAxis(const Eigen::Vector3d &support_vector,
                             const Eigen::Vector3d &direction);
  /**
   * Set desired heading based on statically set ros parameters
   */
  void SetStaticHeading();
  Eigen::Vector3d ClosestPointToAxis();

  bool StartPoseBasedAxis();

  //////////////////////////////////////////////////////////////////////////////
  // Callbacks
  //////////////////////////////////////////////////////////////////////////////
  //   void OnHeadingTarget(const hippo_msgs::msg::HeadingTarget::SharedPtr);
  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr);
  void OnSetPath(const hippo_msgs::srv::SetPath::Request::SharedPtr _req,
                 hippo_msgs::srv::SetPath::Response::SharedPtr _response);
  void OnStart(const std_srvs::srv::Trigger::Request::SharedPtr request,
               std_srvs::srv::Trigger::Response::SharedPtr response);

  rcl_interfaces::msg::SetParametersResult OnParameters(
      const std::vector<rclcpp::Parameter> &parameters);

  //////////////////////////////////////////////////////////////////////////////
  // Publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      heading_target_pub_;
  rclcpp::Publisher<hippo_msgs::msg::PathFollowerDebug>::SharedPtr debug_pub_;

  //////////////////////////////////////////////////////////////////////////////
  // Subscriptions
  //////////////////////////////////////////////////////////////////////////////
  //   rclcpp::Subscription<hippo_msgs::msg::HeadingTarget>::SharedPtr
  //       heading_target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  rclcpp::Service<hippo_msgs::srv::SetPath>::SharedPtr set_path_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;

  Eigen::Quaterniond orientation_{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d position_{0.0, 0.0, 0.0};
  Eigen::Vector3d target_position_{0.0, 0.0, 0.0};
  Eigen::Vector3d target_heading_{1.0, 0.0, 0.0};
  std::shared_ptr<Path> path_{nullptr};
  std::shared_ptr<RvizHelper> path_visualizer_;

  Eigen::Vector3d support_vector_{0.0, 0.0, 0.0};
  Eigen::Vector3d direction_vector_{1.0, 0.0, 0.0};

  OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;

  Params params_;
};

}  // namespace path_planning

#include <rapid_trajectories/trajectory_generator/generator.h>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

namespace rapid_trajectories {
class RvizHelper {
 public:
  RvizHelper(rclcpp::Node::SharedPtr _node);
  void PublishTrajectory(
      const trajectory_generator::RapidTrajectoryGenerator &_trajectory,
      const double _t_final);

  void PublishTarget(const geometry_msgs::msg::Point &_target);

  void PublishStart(const geometry_msgs::msg::Point &_target);

 private:
 void InitTargetMarker();
 void InitStartMarker();
 void InitThrustMarker();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_pub_;
  visualization_msgs::msg::Marker target_marker_;
  visualization_msgs::msg::Marker start_marker_;
  visualization_msgs::msg::Marker thrust_marker_;
  int n_trajectory_samples_{50};
};
}  // namespace rapid_trajectories

#include <chrono>
#include <eigen3/Eigen/Dense>
#include <hippo_msgs/msg/attitude_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rapid_trajectories/rviz_helper.hpp>
#include <rapid_trajectories/trajectory_generator/generator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace rapid_trajectories {
using namespace trajectory_generator;
using namespace hippo_msgs::msg;
using namespace nav_msgs::msg;
using namespace std::chrono_literals;
using std::placeholders::_1;

class SingleTrackerNode : public rclcpp::Node {
 public:
  SingleTrackerNode() : Node("single_tracker") {
    RCLCPP_INFO(get_logger(), "Node created.");
    rclcpp::Node::SharedPtr rviz_node = create_sub_node("visualization");
    rviz_helper_ = std::make_shared<RvizHelper>(rviz_node);
    InitPublishers();
    InitSubscribers();
    trajectory_timer_ = create_wall_timer(
        20ms, std::bind(&SingleTrackerNode::UpdateTrajectory, this));
  }
  void InitPublishers() {
    std::string topic;
    rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

    topic = "body_rates_setpoint";
    body_rates_pub_ = create_publisher<AttitudeTarget>(topic, qos);
  }

  void InitSubscribers() {
    std::string topic;
    rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

    topic = "odometry";
    state_sub_ = create_subscription<Odometry>(
        topic, qos, std::bind(&SingleTrackerNode::OnOdometry, this, _1));
  }

  void UpdateTrajectory() {
    std::vector<Eigen::Vector3d> target_positions{target_position_};
    std::vector<Eigen::Vector3d> target_velocities{target_velocity_};
    std::vector<Eigen::Vector3d> target_accelerations{target_acceleration_};
    DeleteTrajectories();
    GenerateTrajectories(100.0, target_positions, target_velocities,
                         target_accelerations);
    try {
      rviz_helper_->PublishTrajectory(generators_.at(0), 100.0);
    } catch (const std::exception &e) {
      RCLCPP_WARN_STREAM(get_logger(), e.what() << '\n');
    }
    geometry_msgs::msg::Point p;
    p.x = target_position_.x();
    p.y = target_position_.y();
    p.z = target_position_.z();
    rviz_helper_->PublishTarget(p);
    p.x = position_.x();
    p.y = position_.y();
    p.z = position_.z();
    rviz_helper_->PublishStart(p);
  }

 private:
  void OnOdometry(const Odometry::SharedPtr _msg) {
    if (_msg->header.frame_id != "map") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Odometry message's frame id is [%s], but only "
                           "[map] is handled. Ignoring...",
                           _msg->header.frame_id.c_str());
      return;
    }
    position_.x() = _msg->pose.pose.position.x;
    position_.y() = _msg->pose.pose.position.y;
    position_.z() = _msg->pose.pose.position.z;

    velocity_.x() = _msg->twist.twist.linear.x;
    velocity_.y() = _msg->twist.twist.linear.y;
    velocity_.z() = _msg->twist.twist.linear.z;
  }

  void GenerateTrajectories(
      double _duration, std::vector<Eigen::Vector3d> &_target_positions,
      std::vector<Eigen::Vector3d> &_target_velocities,
      std::vector<Eigen::Vector3d> &_target_accelerations) {
    // make sure all target vectors have the same size
    assert(_target_positions.size() == _target_velocities.size() &&
           _target_positions.size() == _target_accelerations.size());

    RapidTrajectoryGenerator::InputFeasibilityResult input_feasibility;

    for (unsigned int i = 0; i < _target_positions.size(); ++i) {
      RapidTrajectoryGenerator trajectory{
          position_, velocity_, Eigen::Vector3d::Zero(),
          trajectory_settings_.mass, trajectory_settings_.damping};
      trajectory.SetGoalPosition(_target_positions[i]);
      trajectory.SetGoalVelocity(_target_velocities[i]);
      trajectory.SetGoalAcceleration(_target_accelerations[i]);
      trajectory.Generate(_duration);
      input_feasibility = trajectory.CheckInputFeasibility(
          trajectory_settings_.thrust_min, trajectory_settings_.thrust_max,
          trajectory_settings_.omega_max, trajectory_settings_.timestep_min);
      if (input_feasibility ==
          RapidTrajectoryGenerator::InputFeasibilityResult::InputFeasible) {
        generators_.push_back(trajectory);
      } else {
        RCLCPP_WARN(get_logger(), "Infeasible reason: %d", input_feasibility);
      }
    }
  }

  void DeleteTrajectories() { generators_.clear(); }

  static constexpr double kGravity{-9.81};
  struct TrajectorySettings {
    double thrust_min{0.0};
    double thrust_max{16.0};
    double omega_max{3.0};
    double mass{2.6};
    double damping{5.4};
    double timestep_min{0.02};
  } trajectory_settings_;
  rclcpp::Publisher<AttitudeTarget>::SharedPtr body_rates_pub_;
  rclcpp::Subscription<Odometry>::SharedPtr state_sub_;
  std::vector<RapidTrajectoryGenerator> generators_;
  Eigen::Vector3d position_{0.0, 0.0, 0.0};
  Eigen::Vector3d velocity_{0.0, 0.0, 0.0};

  rclcpp::TimerBase::SharedPtr trajectory_timer_;

  std::shared_ptr<RvizHelper> rviz_helper_;

  Eigen::Vector3d target_position_{0.0, 0.0, 0.0};
  Eigen::Vector3d target_velocity_{1.0, 0.0, 0.0};
  Eigen::Vector3d target_acceleration_{0.0, 0.0, 0.0};
};

}  // namespace rapid_trajectories

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rapid_trajectories::SingleTrackerNode>());
  rclcpp::shutdown();
  return 0;
}

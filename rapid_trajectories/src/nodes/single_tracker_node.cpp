#include <rapid_trajectories/trajectory_generator/generator.h>

#include <eigen3/Eigen/Dense>
#include <hippo_msgs/msg/attitude_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace rapid_trajectories {
using namespace trajectory_generator;
using namespace hippo_msgs::msg;
using namespace nav_msgs::msg;
using std::placeholders::_1;

class SingleTrackerNode : public rclcpp::Node {
 public:
  SingleTrackerNode() : Node("single_tracker") {
    RCLCPP_INFO(get_logger(), "Node created.");
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

  void UpdateTrajectory() {}

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

    Vec3 ps{position_.x(), position_.y(), position_.z()};
    Vec3 vs{velocity_.x(), velocity_.y(), velocity_.z()};
    Vec3 as{0.0, 0.0, 0.0};
    Vec3 gravity{0.0, 0.0, kGravity};

    RapidTrajectoryGenerator::InputFeasibilityResult input_feasibility;

    for (unsigned int i = 0; i < _target_positions.size(); ++i) {
      Vec3 pf{_target_positions[i].x(), _target_positions[i].y(),
              _target_positions[i].z()};
      Vec3 vf{_target_velocities[i].x(), _target_velocities[i].y(),
              _target_velocities[i].z()};
      Vec3 af{_target_accelerations[i].x(), _target_accelerations[i].y(),
              _target_accelerations[i].z()};

      RapidTrajectoryGenerator trajectory{ps, vs, as, gravity};
      trajectory.SetGoalPosition(pf);
      trajectory.SetGoalVelocity(vf);
      trajectory.SetGoalAcceleration(af);
      trajectory.Generate(_duration);
      input_feasibility = trajectory.CheckInputFeasibility(
          input_limits_.thrust_min, input_limits_.thrust_max,
          input_limits_.omega_max, input_limits_.timestep_min);
      if (input_feasibility ==
          RapidTrajectoryGenerator::InputFeasibilityResult::InputFeasible) {
        generators_.push_back(trajectory);
      }
    }
  }

  void DeleteTrajectories() { generators_.clear(); }

  static constexpr double kGravity{-9.81};
  struct InputLimits {
    double thrust_min{0.0};
    double thrust_max{8.0};
    double omega_max{3.0};
    double timestep_min{0.02};
  } input_limits_;
  rclcpp::Publisher<AttitudeTarget>::SharedPtr body_rates_pub_;
  rclcpp::Subscription<Odometry>::SharedPtr state_sub_;
  std::vector<RapidTrajectoryGenerator> generators_;
  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;

  Eigen::Vector3d target_position_;
  Eigen::Vector3d target_velocity_;
  Eigen::Vector3d target_acceleration_;
};

}  // namespace rapid_trajectories

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::shutdown();
  return 0;
}

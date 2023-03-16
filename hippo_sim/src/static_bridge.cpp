#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hippo_msgs/msg/actuator_controls.hpp>
#include <hippo_msgs/msg/angular_velocity.hpp>
#include <hippo_msgs/msg/esc_rpms.hpp>
#include <hippo_msgs/msg/thruster_forces.hpp>
#include <ignition/transport/Node.hh>
#include <rclcpp/node_interfaces/node_topics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/convert.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std_msgs::msg;
using namespace hippo_msgs::msg;
using namespace ignition;
using namespace nav_msgs::msg;
namespace gz_msgs = ignition::msgs;
using std::placeholders::_1;

class Bridge {
 public:
  Bridge() {
    node_topics = (rclcpp::node_interfaces::NodeTopics *)ros_node_
                      ->get_node_topics_interface()
                      .get();

    CreateGroundTruthBridge();
    CreateImuBridge();
    CreateThrusterBridge();
    CreateBarometerBridge();
    CreateLinearAccelerationBridge();
    CreateAngularVelocityBridge();
  }

  void CreateAngularVelocityBridge() {
    std::string name;
    name = node_topics->resolve_topic_name("angular_velocity");
    angular_velocity_pub_ =
        ros_node_->create_publisher<hippo_msgs::msg::AngularVelocity>(
            name, rclcpp::SystemDefaultsQoS());
    gz_node_->Subscribe(name, &Bridge::OnAngularVelocity, this);
  }

  void CreateLinearAccelerationBridge() {
    std::string name;
    name = node_topics->resolve_topic_name("acceleration");
    linear_acceleration_pub_ =
        ros_node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            name, rclcpp::SystemDefaultsQoS());
    gz_node_->Subscribe(name, &Bridge::OnLinearAcceleration, this);
  }

  void CreateBarometerBridge() {
    std::string name;
    name = node_topics->resolve_topic_name("pressure");
    pressure_pub_ =
        ros_node_->create_publisher<sensor_msgs::msg::FluidPressure>(
            name, rclcpp::SystemDefaultsQoS());
    gz_node_->Subscribe(name, &Bridge::OnPressure, this);
  }

  void CreateGroundTruthBridge() {
    std::string topic_name;
    rclcpp::SystemDefaultsQoS qos;
    qos.keep_last(100);

    // ros publisher
    topic_name = node_topics->resolve_topic_name("ground_truth/pose");
    pose_pub_ = ros_node_->create_publisher<PoseStamped>(topic_name, qos);

    // gazebo subscriber
    gz_node_->Subscribe(topic_name, &Bridge::OnPose, this);
    RCLCPP_INFO(ros_node_->get_logger(), "Create gz subscription: [%s]",
                topic_name.c_str());

    topic_name = node_topics->resolve_topic_name("ground_truth/odometry");
    odometry_pub_ = ros_node_->create_publisher<Odometry>(topic_name, qos);
    gz_node_->Subscribe(topic_name, &Bridge::OnOdometry, this);
  }

  void CreateImuBridge() {
    std::string topic_name;
    rclcpp::SystemDefaultsQoS qos;
    qos.keep_last(100);

    // ros publisher
    topic_name = node_topics->resolve_topic_name("imu");
    imu_pub_ = ros_node_->create_publisher<Imu>(topic_name, qos);

    // gazebo subscriber
    gz_node_->Subscribe(topic_name, &Bridge::OnImu, this);
  }

  void CreateThrusterBridge() {
    rclcpp::SystemDefaultsQoS qos;
    rpm_pub_ = ros_node_->create_publisher<EscRpms>(
        node_topics->resolve_topic_name("esc_rpm"), qos);
    thruster_forces_pub_ =
        ros_node_->create_publisher<ThrusterForces>("thrusts", qos);
    for (size_t i = 0; i < ActuatorControls().control.size(); i++) {
      std::string topic_name;
      qos.keep_last(50);
      std::string topic_prefix =
          node_topics->resolve_topic_name("thruster_") + std::to_string(i);
      topic_name = topic_prefix + "/throttle_cmd";

      // gazebo publisher
      throttle_cmd_pubs_[i] = gz_node_->Advertise<gz_msgs::Double>(topic_name);

      topic_name = topic_prefix + "/rpm";
      std::function<void(const gz_msgs::Double &)> f =
          std::bind(&Bridge::OnThrusterRpm, this, _1, i);
      gz_node_->Subscribe(topic_name, f);
      topic_name = topic_prefix + "/thrust";
      f = std::bind(&Bridge::OnThrust, this, _1, i);
      gz_node_->Subscribe(topic_name, f);
    }
    thrust_sub_ = ros_node_->create_subscription<ActuatorControls>(
        "thruster_command", qos,
        std::bind(&Bridge::OnThrusterCommand, this, _1));
  }

  void OnAngularVelocity(const gz_msgs::Twist &_msg) {
    hippo_msgs::msg::AngularVelocity ros_msg;
    ros_msg.header.stamp = ros_node_->now();
    ros_msg.body_rates[0] = _msg.angular().x();
    ros_msg.body_rates[1] = _msg.angular().y();
    ros_msg.body_rates[2] = _msg.angular().z();
    ros_msg.body_rates_derivative[0] = _msg.linear().x();
    ros_msg.body_rates_derivative[1] = _msg.linear().y();
    ros_msg.body_rates_derivative[2] = _msg.linear().z();
    angular_velocity_pub_->publish(ros_msg);
  }

  void OnLinearAcceleration(const gz_msgs::Vector3d &_msg) {
    geometry_msgs::msg::Vector3Stamped ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_msg, ros_msg.vector);
    ros_gz_bridge::convert_gz_to_ros(_msg.header(), ros_msg.header);
    linear_acceleration_pub_->publish(ros_msg);
  }

  void OnPressure(const gz_msgs::FluidPressure &_msg) {
    sensor_msgs::msg::FluidPressure ros_msg;
    ros_msg.header.frame_id = "map";
    ros_msg.header.stamp = ros_node_->now();
    ros_msg.fluid_pressure = _msg.pressure();
    pressure_pub_->publish(ros_msg);
  }

  void OnImu(const gz_msgs::IMU &_msg) {
    sensor_msgs::msg::Imu ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_msg, ros_msg);
    ros_msg.header.stamp = ros_node_->now();
    imu_pub_->publish(ros_msg);
  }

  void OnThrusterRpm(const gz_msgs::Double &_msg, size_t _i) {
    if (_i > thrusters_rpm_msg_.rpms.size() - 1) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    thrusters_rpm_msg_.rpms[_i] = _msg.data();
    if (_i == 0) {
      thrusters_rpm_msg_.header.stamp = ros_node_->get_clock()->now();
      rpm_pub_->publish(thrusters_rpm_msg_);
    }
  }

  void OnThrust(const gz_msgs::Double &_msg, size_t _i) {
    if (_i > thruster_forces_msg_.force.size() - 1) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    thruster_forces_msg_.force[_i] = _msg.data();
    if (_i == 0) {
      thruster_forces_msg_.header.stamp = ros_node_->now();
      thruster_forces_pub_->publish(thruster_forces_msg_);
    }
  }

  void OnPose(const gz_msgs::Pose &_msg) {
    geometry_msgs::msg::PoseStamped ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_msg, ros_msg);
    ros_msg.header.stamp = ros_node_->now();
    pose_pub_->publish(ros_msg);
  }

  void OnOdometry(const gz_msgs::Odometry &_msg) {
    Odometry ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_msg, ros_msg);
    ros_msg.header.stamp = ros_node_->now();
    odometry_pub_->publish(ros_msg);
  }

  void OnThrusterCommand(const ActuatorControls::SharedPtr _msg) {
    if (!(_msg->control.size() == throttle_cmd_pubs_.size())) {
      RCLCPP_ERROR(ros_node_->get_logger(),
                   "ActuatControls and publisher map do not have same size!");
      return;
    }
    for (unsigned int i = 0; i < throttle_cmd_pubs_.size(); ++i) {
      gz_msgs::Double gz_msg;
      gz_msg.set_data(_msg->control[i]);
      throttle_cmd_pubs_[i].Publish(gz_msg);
    }
  }

  void Run() { rclcpp::spin(ros_node_); }

 private:
  rclcpp::Node::SharedPtr ros_node_ = std::make_shared<rclcpp::Node>("bridge");
  std::shared_ptr<transport::Node> gz_node_ =
      std::make_shared<transport::Node>();
  rclcpp::node_interfaces::NodeTopics *node_topics;

  std::mutex mutex_;

  EscRpms thrusters_rpm_msg_;
  ThrusterForces thruster_forces_msg_;

  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<ThrusterForces>::SharedPtr thruster_forces_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      linear_acceleration_pub_;
  rclcpp::Publisher<hippo_msgs::msg::AngularVelocity>::SharedPtr
      angular_velocity_pub_;

  std::map<int, transport::Node::Publisher> throttle_cmd_pubs_;
  rclcpp::Publisher<EscRpms>::SharedPtr rpm_pub_;
  rclcpp::Subscription<ActuatorControls>::SharedPtr thrust_sub_;
};

int main(int _argc, char **_argv) {
  rclcpp::init(_argc, _argv);
  auto bridge = Bridge();
  bridge.Run();
}

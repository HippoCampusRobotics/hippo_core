#include <hippo_msgs/msg/actuator_controls.hpp>
#include <hippo_msgs/msg/esc_rpms.hpp>
#include <ignition/transport/Node.hh>
#include <rclcpp/node_interfaces/node_topics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/convert.hpp>

using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std_msgs::msg;
using namespace hippo_msgs::msg;
using namespace ignition;
using namespace nav_msgs::msg;
namespace gz_msgs = ignition::msgs;
using std::placeholders::_1;
using std::placeholders::_2;

class Bridge {
 public:
  Bridge() {
    node_topics = (rclcpp::node_interfaces::NodeTopics *)ros_node_
                      ->get_node_topics_interface()
                      .get();

    CreateGroundTruthBridge();
    CreateImuBridge();
    CreateThrusterBridge();
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
    for (size_t i = 0; i < ActuatorControls().control.size(); i++) {
      std::string topic_name;
      qos.keep_last(50);
      std::string topic_prefix =
          node_topics->resolve_topic_name("thruster_") + std::to_string(i);
      topic_name = topic_prefix + "/thrust";

      // gazebo publisher
      thrust_pubs_[i] = gz_node_->Advertise<gz_msgs::Double>(topic_name);
      topic_name = topic_prefix + "/rpm";
      std::function<void(const gz_msgs::Double &)> f =
          std::bind(&Bridge::OnThrusterRpm, this, _1, i);
      gz_node_->Subscribe(topic_name, f);
    }

    thrust_sub_ = ros_node_->create_subscription<ActuatorControls>(
        "thruster_command", qos,
        std::bind(&Bridge::OnThrusterCommand, this, _1));
  }

  void OnImu(const gz_msgs::IMU &_msg) {
    sensor_msgs::msg::Imu ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_msg, ros_msg);
    imu_pub_->publish(ros_msg);
  }

  void OnThrusterRpm(const gz_msgs::Double &_msg, size_t _i) {
    if (_i > thrusters_rpm_msg_.rpms.size()) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    thrusters_rpm_msg_.rpms[_i] = _msg.data();
    if (_i == 0) {
      thrusters_rpm_msg_.header.stamp = ros_node_->get_clock()->now();
      rpm_pub_->publish(thrusters_rpm_msg_);
    }
  }

  void OnPose(const gz_msgs::Pose &_msg) {
    geometry_msgs::msg::PoseStamped ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_msg, ros_msg);
    pose_pub_->publish(ros_msg);
  }

  void OnOdometry(const gz_msgs::Odometry &_msg) {
    Odometry ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_msg, ros_msg);
    odometry_pub_->publish(ros_msg);
  }

  void OnThrusterCommand(const ActuatorControls::SharedPtr _msg) {
    if (!(_msg->control.size() == thrust_pubs_.size())) {
      RCLCPP_ERROR(ros_node_->get_logger(),
                   "ActuatControls and publisher map do not have same size!");
      return;
    }
    for (unsigned int i = 0; i < thrust_pubs_.size(); ++i) {
      gz_msgs::Double gz_msg;
      gz_msg.set_data(_msg->control[i]);
      thrust_pubs_[i].Publish(gz_msg);
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

  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_pub_;
  std::map<int, transport::Node::Publisher> thrust_pubs_;
  rclcpp::Publisher<EscRpms>::SharedPtr rpm_pub_;
  rclcpp::Subscription<ActuatorControls>::SharedPtr thrust_sub_;
};

int main(int _argc, char **_argv) {
  rclcpp::init(_argc, _argv);
  auto bridge = Bridge();
  bridge.Run();
}

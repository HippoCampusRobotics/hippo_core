#include <chrono>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <hippo_common/param_utils.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>

class FakeVision : public rclcpp::Node {
 public:
  FakeVision() : Node("fake_vision") {
    DeclareParameters();
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "vision_pose", rclcpp::SystemDefaultsQoS());

    ground_truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "ground_truth/odometry", rclcpp::SystemDefaultsQoS(),
        std::bind(&FakeVision::OnOdometry, this, std::placeholders::_1));

    update_timer_ =
        rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(1),
                             std::bind(&FakeVision::Update, this));
  }

 private:
  struct GeneralParams {
    int delay_ms{100};
    struct StdDev {
      double x{0.05};
      double y{0.05};
      double z{0.05};
    } std_dev;
  };
  void DeclareParameters() {
    std::string name;
    std::string descr;
    rcl_interfaces::msg::ParameterDescriptor param;

    name = "delay_ms";
    descr =
        "How long the visual data is delayed (i.e. the processing time of the "
        "camera data).";
    param = hippo_common::param_utils::DescriptionLimit(descr, 0, 1000);
    params_.delay_ms = declare_parameter<int>(name, 100, param);

    name = "std_dev.x";
    descr = "Standard deviance of gaussian noise for the corresponding axis.";
    param = hippo_common::param_utils::DescriptionLimit(descr, 0.0, 0.5);
    params_.std_dev.x = declare_parameter<double>(name, 0.05, param);

    name = "std_dev.y";
    descr = "Standard deviance of gaussian noise for the corresponding axis.";
    param = hippo_common::param_utils::DescriptionLimit(descr, 0.0, 0.5);
    params_.std_dev.y = declare_parameter<double>(name, 0.05, param);

    name = "std_dev.z";
    descr = "Standard deviance of gaussian noise for the corresponding axis.";
    param = hippo_common::param_utils::DescriptionLimit(descr, 0.0, 0.5);
    params_.std_dev.z = declare_parameter<double>(name, 0.05, param);

    params_cb_handle_ = add_on_set_parameters_callback(
        std::bind(&FakeVision::OnParams, this, std::placeholders::_1));
  }

  void Update() {
    if (pose_msgs_.empty()) {
      return;
    }
    geometry_msgs::msg::PoseWithCovarianceStamped p = pose_msgs_.front();
    double t_ms = rclcpp::Time(p.header.stamp).nanoseconds() * 1e-6;
    if (t_ms + params_.delay_ms <= now().nanoseconds() * 1e-6) {
      pose_pub_->publish(p);
      pose_msgs_.pop();
    }
  }

  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr _msg) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header = _msg->header;
    pose_msg.pose.pose = _msg->pose.pose;
    pose_msgs_.push(pose_msg);
  }

  rcl_interfaces::msg::SetParametersResult OnParams(
      const std::vector<rclcpp::Parameter> &_parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Unhandled";

    for (const auto &parameter : _parameters) {
      if (hippo_common::param_utils::AssignIfMatch(parameter, "delay_ms",
                                                   params_.delay_ms)) {
        result.reason = "Set delay_ms.";
        continue;
      }
      if (hippo_common::param_utils::AssignIfMatch(parameter, "std_dev.x",
                                                   params_.std_dev.x)) {
        result.reason = "Set std_dev.x";
        continue;
      }
      if (hippo_common::param_utils::AssignIfMatch(parameter, "std_dev.y",
                                                   params_.std_dev.y)) {
        result.reason = "Set std_dev.y";
        continue;
      }
      if (hippo_common::param_utils::AssignIfMatch(parameter, "std_dev.z",
                                                   params_.std_dev.z)) {
        result.reason = "Set std_dev.z";
        continue;
      }
    }
    return result;
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  std::queue<geometry_msgs::msg::PoseWithCovarianceStamped> pose_msgs_;
  GeneralParams params_;
  OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeVision>());
  rclcpp::shutdown();
  return 0;
}

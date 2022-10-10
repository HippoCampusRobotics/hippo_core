#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <hippo_common/tf2_utils.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

using nav_msgs::msg::Odometry;
using sensor_msgs::msg::FluidPressure;
using std::placeholders::_1;
using namespace hippo_common;

class SimulatedBaro : public rclcpp::Node {
 public:
  SimulatedBaro() : Node("barometer") {
    rclcpp::SystemDefaultsQoS qos;
    publisher_ = create_publisher<FluidPressure>("pressure", qos);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ = create_subscription<Odometry>(
        "ground_truth/odometry", qos,
        std::bind(&SimulatedBaro::OnOdometry, this, _1));
  }
  void OnOdometry(const Odometry::SharedPtr _msg) {
    auto base_link_to_baro_local = tf_buffer_->lookupTransform(
        tf2_utils::frame_id::Barometer(this),
        tf2_utils::frame_id::BaseLink(this), tf2::TimePointZero);
    auto map_to_base_link_tf = tf_buffer_->lookupTransform(
        tf2_utils::frame_id::BaseLink(this),
        tf2_utils::frame_id::InertialFrame(), tf2::TimePointZero);
    geometry_msgs::msg::Vector3 barometer_offset;
    tf2::doTransform(base_link_to_baro_local.transform.translation,
                     barometer_offset, map_to_base_link_tf);
    double z =
        _msg->pose.pose.position.z + barometer_offset.z - sea_level_offset_;
    FluidPressure msg_out;
    msg_out.header = _msg->header;
    msg_out.fluid_pressure = z * kPascalPerMeter + pressure_offset_;
    publisher_->publish(msg_out);
  }

 private:
  static constexpr double kPascalPerMeter{1.0e4};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  rclcpp::Publisher<FluidPressure>::SharedPtr publisher_;
  rclcpp::Subscription<Odometry>::SharedPtr subscription_;
  // atmospheric pressure at water surface
  std::atomic<double> pressure_offset_{101325.0};
  // height offset of the water surface relative to the inertial frame
  std::atomic<double> sea_level_offset_{0.0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimulatedBaro>();
  rclcpp::spin(node);
  return 0;
}

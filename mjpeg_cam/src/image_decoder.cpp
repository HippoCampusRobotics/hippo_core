#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageDecoder : public rclcpp::Node {
 public:
  ImageDecoder(const rclcpp::NodeOptions &options)
      : Node("image_decoder", options) {
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", qos);
    image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        "image_raw/compressed", qos,
        [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
          OnImage(msg);
        });
  }

 private:
  void OnImage(const sensor_msgs::msg::CompressedImage::SharedPtr _msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(_msg);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    if (!image_pub_) {
      RCLCPP_ERROR(get_logger(), "Publisher not initialized.");
      return;
    }
    image_pub_->publish(*cv_ptr->toImageMsg());
  }
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::experimental::executors::EventsExecutor exec;
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ImageDecoder>(options);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

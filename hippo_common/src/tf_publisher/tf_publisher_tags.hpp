#include <tf2_ros/static_transform_broadcaster.h>

#include <rclcpp/rclcpp.hpp>

namespace hippo_common {
class TfPublisherTags : public rclcpp::Node {
 public:
  explicit TfPublisherTags(const rclcpp::NodeOptions &);

 private:
  struct Params {
    std::string tag_poses_file;
  };

  void BroadCastStatic();

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  Params params_;
};

}  // namespace hippo_common
